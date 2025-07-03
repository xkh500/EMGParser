#include "pch.h"       // 如果启用了预编译头
#include "EMGParser.h"
#include <vector>
#include <mutex>
#include <cstring>

// —— 协议常量 ——
static const uint8_t HDR_ACC = 0x51, ID_ACC = 0xF1;
static const uint8_t HDR_EMG = 0x53, ID_EMG = 0xF3;
static const uint8_t HDR_SPO2 = 0x55, ID_SPO2 = 0xF5;
static const uint8_t HDR_TEMP = 0x57, ID_TEMP = 0xF7;
static const uint8_t HDR_MAG = 0x59, ID_MAG = 0xF9;
static const uint8_t TAIL = 0x5C;

// —— 状态机状态 ——
enum State {
    ST_WAIT_HDR,
    ST_WAIT_ID,
    ST_READ_PAY,
    ST_READ_CHK,
    ST_WAIT_TAIL
};

// —— 全局解析状态 ——
static State                   g_state;
static uint8_t                 g_hdr, g_id;
static int                     g_toRead;
static uint8_t                 g_chkRecv;
static std::vector<uint8_t>    g_payload;

// —— 后台缓存的最新数据（已换算的物理量） ——
static float    g_acc[3] = { 0 }, g_emg = 0, g_spo2[2] = { 0 }, g_temperature = 0, g_mag[3] = { 0 };
static float    g_mag_offset[3] = { 0 };  // 磁力零漂补偿
static int      g_mag_calib_count = 0;
static std::mutex            g_mutex;

// —— 符号扩展 24-bit big-endian → 32-bit signed ——
static inline int conv24(const uint8_t* p) {
    // Python Demo decoding uses big-endian order: p[0] MSB, p[1] mid, p[2] LSB
    int v = (p[0] << 16) | (p[1] << 8) | p[2];
    if (v & 0x800000) v |= 0xFF000000;
    return v;
}
// —— 符号扩展 16-bit big-endian → signed 32-bit ——
static inline int conv16(const uint8_t* p) {
    // big-endian: p[0] MSB, p[1] LSB
    int16_t s = int16_t((p[0] << 8) | p[1]);
    return int(s);
}
// —— 计算 8 位校验 ——
static inline uint8_t calcChecksum(const std::vector<uint8_t>& buf) {
    uint32_t sum = 0;
    for (auto b : buf) sum += b;
    return uint8_t(sum & 0xFF);
}

// —— 解析一帧载荷到全局缓存 ——
static void parseFrame() {
    const uint8_t* p = g_payload.data();
    std::lock_guard<std::mutex> lk(g_mutex);
    switch (g_hdr) {
    case HDR_ACC: {
        const float acc_scale = 8.0f / 16384.0f;  // Python Demo uses 16-bit, scale = ±8g / 2^14
        // big-endian 16-bit extraction to match Python Demo
        int16_t x = int16_t((p[0] << 8) | p[1]);
        int16_t y = int16_t((p[2] << 8) | p[3]);
        int16_t z = int16_t((p[4] << 8) | p[5]);
        g_acc[0] = x * acc_scale;
        g_acc[1] = y * acc_scale;
        g_acc[2] = z * acc_scale;
        break;
    }
    case HDR_EMG: {
        // 24-bit big-endian
        g_emg = conv24(p + 0) * (3.3f / 8388608.0f);
        break;
    }
    case HDR_SPO2: {
        g_spo2[0] = conv24(p + 0);
        g_spo2[1] = conv24(p + 3);
        break;
    }
    case HDR_TEMP: {
        g_temperature = conv16(p + 0) / 100.0f;
        break;
    }
    case HDR_MAG: {
        int16_t rx = int16_t((p[0] << 8) | p[1]);
        int16_t ry = int16_t((p[2] << 8) | p[3]);
        int16_t rz = int16_t((p[4] << 8) | p[5]);
        if (g_mag_calib_count < 10) {
            g_mag_offset[0] += rx;
            g_mag_offset[1] += ry;
            g_mag_offset[2] += rz;
            g_mag_calib_count++;
            if (g_mag_calib_count == 10) {
                g_mag_offset[0] /= 10.0f;
                g_mag_offset[1] /= 10.0f;
                g_mag_offset[2] /= 10.0f;
            }
        }
        float scale = 0.005f;
        g_mag[0] = (rx - g_mag_offset[0]) * scale;
        g_mag[1] = (ry - g_mag_offset[1]) * scale;
        g_mag[2] = (rz - g_mag_offset[2]) * scale;
        break;
    }
    default:
        break;
    }
}

extern "C" {

    // 初始化解析器状态
    EMGPARSER_API void EMGParser_Init() {
        g_state = ST_WAIT_HDR;
        g_payload.clear();
        g_mag_offset[0] = g_mag_offset[1] = g_mag_offset[2] = 0;
        g_mag_calib_count = 0;
    }

    // 逐字节调用，返回 true 表示已完成一帧解析
    EMGPARSER_API bool EMGParser_ParseByte(uint8_t b) {
        switch (g_state) {
        case ST_WAIT_HDR:
            if (b == HDR_ACC || b == HDR_EMG || b == HDR_SPO2 || b == HDR_TEMP || b == HDR_MAG) {
                g_hdr = b; g_state = ST_WAIT_ID;
            }
            break;
        case ST_WAIT_ID:
            g_id = b;
            if ((g_hdr == HDR_ACC && g_id == ID_ACC) ||
                (g_hdr == HDR_EMG && g_id == ID_EMG) ||
                (g_hdr == HDR_SPO2 && g_id == ID_SPO2) ||
                (g_hdr == HDR_TEMP && g_id == ID_TEMP) ||
                (g_hdr == HDR_MAG && g_id == ID_MAG)) {
                switch (g_hdr) {
                case HDR_ACC:  g_toRead = 9; break;  // only first 6 bytes for 16-bit
                case HDR_EMG:  g_toRead = 3; break;
                case HDR_SPO2: g_toRead = 6; break;
                case HDR_TEMP: g_toRead = 2; break;
                case HDR_MAG:  g_toRead = 6; break;
                }
                g_payload.clear(); g_state = ST_READ_PAY;
            }
            else {
                g_state = ST_WAIT_HDR;
            }
            break;
        case ST_READ_PAY:
            g_payload.push_back(b);
            if (--g_toRead == 0) g_state = ST_READ_CHK;
            break;
        case ST_READ_CHK:
            g_chkRecv = b;
            g_state = ST_WAIT_TAIL;
            break;
        case ST_WAIT_TAIL:
            if (b == TAIL) {
                std::vector<uint8_t> tmp = { g_hdr, g_id };
                tmp.insert(tmp.end(), g_payload.begin(), g_payload.end());
                if (g_hdr == HDR_MAG || calcChecksum(tmp) == g_chkRecv) {
                    //跳过磁力计帧的校验和比较
                    parseFrame();
                    g_state = ST_WAIT_HDR;
                    return true;
                }
            }
            g_state = ST_WAIT_HDR;
            break;
        }
        return false;
    }

    // 获取最新数据：各参数直接合并到数组或单值（已换算为物理单位）
    EMGPARSER_API void EMGParser_GetData(
        float acc[3],
        float* emg,
        float spo2[2],
        float* temperature,
        float mag[3]
    ) {
        std::lock_guard<std::mutex> lk(g_mutex);
        memcpy(acc, g_acc, sizeof(g_acc));
        *emg = g_emg;
        memcpy(spo2, g_spo2, sizeof(g_spo2));
        *temperature = g_temperature;
        memcpy(mag, g_mag, sizeof(g_mag));
    }
}

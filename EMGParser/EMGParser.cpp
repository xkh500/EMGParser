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
    ST_READ_CHK_LO,
    ST_READ_CHK_HI,
    ST_WAIT_TAIL
};

// —— 全局解析状态 ——
static State                   g_state;
static uint8_t                 g_hdr, g_id;
static int                     g_toRead;
static uint16_t                g_chkRecv;
static std::vector<uint8_t>    g_payload;

// —— 后台缓存的最新数据（全部整数） ——
static int    g_acc[3] = { 0, 0, 0 };
static int    g_emgRaw = 0;
static int    g_spo2[2] = { 0, 0 };
static int    g_temperature = 0;
static int    g_mag[3] = { 0, 0, 0 };
static std::mutex            g_mutex;

// —— 符号扩展 24-bit → 32-bit signed ——
static inline int conv24(const uint8_t* p) {
    int v = (p[2] << 16) | (p[1] << 8) | p[0];
    if (v & 0x800000) v |= 0xFF000000;
    return v;
}
// —— 16-bit → signed ——
static inline int conv16(const uint8_t* p) {
    return int((p[1] << 8) | p[0]);
}
// —— 计算 16 位校验 ——
static inline uint16_t calcChecksum(const std::vector<uint8_t>& buf) {
    uint32_t sum = 0;
    for (auto b : buf) sum += b;
    return uint16_t(sum & 0xFFFF);
}

// —— 解析一帧载荷到全局缓存 ——
static void parseFrame() {
    const uint8_t* p = g_payload.data();
    std::lock_guard<std::mutex> lk(g_mutex);
    switch (g_hdr) {
    case HDR_ACC:
        g_acc[0] = conv24(p + 0);
        g_acc[1] = conv24(p + 3);
        g_acc[2] = conv24(p + 6);
        break;
    case HDR_EMG:
        g_emgRaw = conv24(p);
        break;
    case HDR_SPO2:
        g_spo2[0] = conv24(p + 0);
        g_spo2[1] = conv24(p + 3);
        break;
    case HDR_TEMP:
        g_temperature = conv16(p);
        break;
    case HDR_MAG:
        g_mag[0] = conv16(p + 0);
        g_mag[1] = conv16(p + 2);
        g_mag[2] = conv16(p + 4);
        break;
    default:
        break;
    }
}

extern "C" {

    // 初始化解析器状态
    EMGPARSER_API void EMGParser_Init() {
        g_state = ST_WAIT_HDR;
        g_payload.clear();
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
                case HDR_ACC:  g_toRead = 9; break;
                case HDR_EMG:  g_toRead = 3; break;
                case HDR_SPO2: g_toRead = 6; break;
                case HDR_TEMP: g_toRead = 2; break;
                case HDR_MAG:  g_toRead = 6; break;
                }
                g_payload.clear(); g_state = ST_READ_PAY;
            }
            else g_state = ST_WAIT_HDR;
            break;
        case ST_READ_PAY:
            g_payload.push_back(b);
            if (--g_toRead == 0) g_state = ST_READ_CHK_LO;
            break;
        case ST_READ_CHK_LO:
            g_chkRecv = b; g_state = ST_READ_CHK_HI; break;
        case ST_READ_CHK_HI:
            g_chkRecv |= uint16_t(b) << 8; g_state = ST_WAIT_TAIL; break;
        case ST_WAIT_TAIL:
            if (b == TAIL) {
                std::vector<uint8_t> tmp = { g_hdr, g_id };
                tmp.insert(tmp.end(), g_payload.begin(), g_payload.end());
                if (calcChecksum(tmp) == g_chkRecv) parseFrame();
            }
            g_state = ST_WAIT_HDR; break;
        }
        return false;
    }

    // 获取最新数据：各参数直接合并到数组或单值（全部整数）
    EMGPARSER_API bool EMGParser_GetData(
        int   acc[3],      // 加速度原始值（LSB）
        int* emgRaw,      // 肌电原始值（LSB）
        int   spo2[2],     // 血氧 Red/IR 原始值（LSB）
        int* temperature, // 温度原始值（LSB）
        int   mag[3]       // 磁力计原始值（LSB）
    ) {
        if (!acc || !emgRaw || !spo2 || !temperature || !mag) return false;
        std::lock_guard<std::mutex> lk(g_mutex);
        memcpy(acc, g_acc, sizeof(g_acc));
        *emgRaw = g_emgRaw;
        memcpy(spo2, g_spo2, sizeof(g_spo2));
        *temperature = g_temperature;
        memcpy(mag, g_mag, sizeof(g_mag));
        return true;
    }

}  // extern "C"

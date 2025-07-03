#pragma once

#ifdef EMGPARSER_EXPORTS
#define EMGPARSER_API __declspec(dllexport)
#else
#define EMGPARSER_API __declspec(dllimport)
#endif

#include <cstdint>

extern "C" {

    // 初始化解析器状态（第一次调用 ParseByte 之前必须调用）
    EMGPARSER_API void EMGParser_Init();

    // 逐字节调用；返回 true 表示接收到一整帧并已完成解析
    EMGPARSER_API bool EMGParser_ParseByte(uint8_t b);

    /**
     * 获取已换算后的传感器数据（单位：g、V、°C、µT 等）
     * 必须在 ParseByte 返回 true 后调用，获取对应帧数据
     * @param acc         [out] 加速度数组，单位 g，长度 3
     * @param emg         [out] 肌电数据，单位 V
     * @param spo2        [out] 血氧原始值，长度 2
     * @param temperature [out] 温度，单位 °C
     * @param mag         [out] 磁力计数据，单位 µT，已去零漂，长度 3
     */
    EMGPARSER_API void EMGParser_GetData(
        float acc[3],
        float* emg,
        float spo2[2],
        float* temperature,
        float mag[3]
    );

}

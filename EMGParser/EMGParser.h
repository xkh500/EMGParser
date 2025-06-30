#pragma once

#ifdef EMGPARSER_EXPORTS
#define EMGPARSER_API __declspec(dllexport)
#else
#define EMGPARSER_API __declspec(dllimport)
#endif

#include <stdint.h>

extern "C" {

    // 初始化解析器状态（第一次调用 ParseByte 之前必须调用）
    EMGPARSER_API void EMGParser_Init();

    // 逐字节调用；返回 true 表示接收到一整帧并已完成解析
    EMGPARSER_API bool EMGParser_ParseByte(uint8_t b);

    // 当 ParseByte 返回 true 后，调用此函数拷贝出最新一帧数据
    // 参数依次为：
    //   acc: 长度 3 的整型数组 (3×24-bit 原始加速度 LSB)
    //   emgRaw: 指向肌电原始值 (24-bit signed LSB)
    //   spo2: 长度 2 的整型数组 (2×24-bit 原始血氧信号 LSB)
    //   temperature: 指向温度原始值 (16-bit signed LSB)
    //   mag: 长度 3 的整型数组 (3×16-bit 原始磁力计 LSB)
    EMGPARSER_API bool EMGParser_GetData(
        int   acc[3],
        int* emgRaw,
        int   spo2[2],
        int* temperature,
        int   mag[3]
    );

} // extern "C"

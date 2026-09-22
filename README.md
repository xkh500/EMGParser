# EMGParser

A dependency-free C++ DLL that parses the byte stream from an EMG / SpO2 / IMU
sensor module into physical units, plus a LabVIEW project that calls into it.

## Usage

Feed the parser one byte at a time as they arrive on the serial port. It
reassembles frames, validates the checksum, scales the raw values, and exposes
the latest reading through a small C ABI.

```cpp
EMGParser_Init();
while (byte_available()) {
    if (EMGParser_ParseByte(read_byte())) {
        float acc[3], emg, spo2[2], temperature, mag[3];
        EMGParser_GetData(acc, &emg, spo2, &temperature, mag);
    }
}
```

## Frame format

| Header | ID     | Payload | Meaning                     | Unit |
|--------|--------|---------|-----------------------------|------|
| `0x51` | `0xF1` | 9 bytes | Acceleration (first 6 used) | g    |
| `0x53` | `0xF3` | 3 bytes | EMG                         | V    |
| `0x55` | `0xF5` | 6 bytes | SpO2 (2 × 24-bit)           | raw  |
| `0x57` | `0xF7` | 2 bytes | Temperature                 | °C   |
| `0x59` | `0xF9` | 6 bytes | Magnetometer (3 × 16-bit)   | µT   |

Every frame ends with a `0x5C` tail; the byte before it is an 8-bit sum
checksum over header + id + payload. Magnetometer frames skip the checksum
comparison. Multi-byte values are big-endian. The magnetometer zero offset is
averaged over the first 10 frames and subtracted from subsequent readings.

## API

```cpp
void EMGParser_Init();
bool EMGParser_ParseByte(uint8_t b);   // true when a full frame was parsed
void EMGParser_GetData(float acc[3], float* emg, float spo2[2],
                       float* temperature, float mag[3]);
```

## Layout

- `EMGParser/` — the DLL: `EMGParser.cpp` (state machine + scaling),
  `EMGParser.h` (C ABI), Visual Studio project
- `EMGParser((labview)/` — LabVIEW project that loads the built DLL

## Build

Open `EMGParser.sln` in Visual Studio and build the `EMGParser` target as x64.

## License

MIT — see [LICENSE](LICENSE).

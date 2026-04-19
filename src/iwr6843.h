#pragma once
#include <stdint.h>

#define IWR_TLV_COMPRESSED_POINTS  1020  // MMWDEMO_OUTPUT_MSG_COMPRESSED_POINTS
#define IWR_TLV_TARGET_LIST_3D1010  // MMWDEMO_OUTPUT_MSG_TRACKERPROC_3D_TARGET_LIST
#define IWR_TLV_TARGET_INDEX       1011  // MMWDEMO_OUTPUT_MSG_TRACKERPROC_TARGET_INDEX
#define IWR_TLV_TARGET_HEIGHT      1012  // MMWDEMO_OUTPUT_MSG_TRACKERPROC_TARGET_HEIGHT
#define IWR_TLV_PRESENCE          1021  // MMWDEMO_OUTPUT_MSG_PRESCENCE_INDICATION
#define IWR_TLV_VITAL_SIGNS        1040  // MMWDEMO_OUTPUT_MSG_VITALSIGNS

typedef struct __attribute__((packed)) {
    uint64_t magic;              // 0x0708050603040102
    uint32_t version;
    uint32_t totalPacketLen;     // padded to next multiple of 32
    uint32_t platform;
    uint32_t frameNumber;  
    uint32_t timeCpuCycles;
    uint32_t numDetectedObj;
    uint32_t numTLVs;
    uint32_t subFrameNumber;
} iwr_frame_header_t;             // 40 bytes

typedef struct __attribute__((packed)) {
    uint32_t type;
    uint32_t length;              // payload length only, excludes this 8-byte header
} iwr_tlv_header_t;               // 8 bytes

typedef struct __attribute__((packed)) {
    float elevUnit;
    float azUnit;
    float dopplerUnit;
    float rangeUnit;
    float snrUnit;
} iwr_point_unit_t;               // 20 bytes

typedef struct __attribute__((packed)) {
    int8_t   elevation;
    int8_t   azimuth;
    int16_t  doppler;
    uint16_t range;
    uint16_t snr;
} iwr_compressed_point_t;         // 8 bytes

typedef struct __attribute__((packed)) {
    uint32_t tid;
    float    posX, posY, posZ;
    float    velX, velY, velZ;
    float    accX, accY, accZ;
    float    ec[16];
    float    g;
    float    confidenceLevel;
} iwr_target_t;                   // 112 bytes

typedef struct __attribute__((packed)) {
    uint16_t id;
    uint16_t rangeBin;
    float    breathDeviation;
    float    heartRate;
    float    breathRate;
    float    heartWaveform[15];
    float    breathWaveform[15];
} iwr_vital_signs_t;              // 136 bytes

void iwr6843_init(void);

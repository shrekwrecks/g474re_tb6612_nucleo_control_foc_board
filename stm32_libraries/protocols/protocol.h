#pragma once
#include <stdint.h>
#include "cobs.h"
#include "protocol_generated.h"

#define PROTO_PAYLOAD_MAX 16u // max data bytes per packet
#define PROTO_HEADER_SIZE 5u  // 1 id + 4 timestamp
#define PROTO_FRAME_MAX 21u   // PROTO_HEADER_SIZE + PROTO_PAYLOAD_MAX
#define PROTO_ENCODED_MAX 23u // COBS worst-case: FRAME_MAX + 1 overhead + 1 delim

typedef struct
{
    miso_packet_id_t id;
    uint32_t timestamp;
    uint8_t data[PROTO_PAYLOAD_MAX];
    uint8_t len;
} packet_t;
// ----------------------------------------------------------- encode (inline)
static inline uint16_t encode_packet(uint8_t *out,
                                     miso_packet_id_t id,
                                     uint32_t timestamp,
                                     const uint8_t *data,
                                     uint8_t len)
{

    uint8_t frame[PROTO_FRAME_MAX];
    frame[0] = (uint8_t)id;
    frame[1] = (uint8_t)(timestamp);
    frame[2] = (uint8_t)(timestamp >> 8);
    frame[3] = (uint8_t)(timestamp >> 16);
    frame[4] = (uint8_t)(timestamp >> 24);
    __builtin_memcpy(&frame[5], data, len); /* single copy from caller's buffer */

    return (uint16_t)cobs_encode(frame, 5u + len, out);
}

static inline uint16_t decode_packet(const uint8_t *in, uint16_t len,
                                     mosi_packet_id_t *id,
                                     uint32_t *timestamp,
                                     const uint8_t **data)
{
    static uint8_t frame[PROTO_FRAME_MAX];
    uint16_t frame_len = (uint16_t)cobs_decode(in, len, frame);
    if (frame_len < 5)
        return 0;

    *id = (mosi_packet_id_t)frame[0];
    *timestamp = (uint32_t)frame[1] | (uint32_t)frame[2] << 8 | (uint32_t)frame[3] << 16 | (uint32_t)frame[4] << 24;
    *data = &frame[5];

    return frame_len - 5;
}
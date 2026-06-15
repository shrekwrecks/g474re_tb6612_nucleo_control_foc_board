#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /*
     * COBS (Consistent Overhead Byte Stuffing)
     * Index-based implementation for clarity.
     * Compatible with standard COBS (same behavior as cmcqueen/cobs-c).
     *
     * No globals, no heap, re-entrant.
     */

    /* ─────────────────────────────────────────────
     *  cobs_encode  — unchanged, shown for context
     * ───────────────────────────────────────────── */

    /* ─────────────────────────────────────────────
     *  cobs_encode  — unchanged, shown for context
     * ───────────────────────────────────────────── */
    static inline size_t
    cobs_encode(const uint8_t *input,
                size_t length,
                uint8_t *output)
    {
        size_t read_index = 0;
        size_t write_index = 1;
        size_t code_index = 0;
        uint8_t code = 1;

        while (read_index < length)
        {
            if (input[read_index] == 0)
            {
                output[code_index] = code;
                code_index = write_index++;
                code = 1;
                read_index++;
            }
            else
            {
                output[write_index++] = input[read_index++];
                code++;
                if (code == 0xFF)
                {
                    output[code_index] = code;
                    code_index = write_index++;
                    code = 1;
                }
            }
        }
        output[code_index] = code;
        return write_index;
    }

    static inline size_t cobs_decode(const uint8_t *input,
                                     size_t length,
                                     uint8_t *output)
    {
        size_t read_index = 0;
        size_t write_index = 0;

        while (read_index < length)
        {
            uint8_t code = input[read_index];

            if (code == 0)
                break; // frame delimiter or corruption

            read_index++;

            for (uint8_t i = 1; i < code; i++)
            {
                if (read_index >= length)
                    return 0; // malformed frame

                output[write_index++] = input[read_index++];
            }

            if (code != 0xFF && read_index < length)
            {
                output[write_index++] = 0;
            }
        }

        return write_index;
    }

#ifdef __cplusplus
}
#endif
#ifndef BITMAP_CONVERTER_H
#define BITMAP_CONVERTER_H

#include <stdint.h>

// Convert RGB565 color to monochrome bit (1 = white, 0 = black)
inline uint8_t rgb565_to_mono(uint16_t rgb565) {
    if (rgb565 == 0x0000) return 0;

    // Extract RGB components (RGB565: RRRRR GGGGGG BBBBB)
    uint8_t r = (rgb565 >> 11) & 0x1F;
    uint8_t g = (rgb565 >> 5) & 0x3F;
    uint8_t b = rgb565 & 0x1F;

    // Scale to 0-255
    r = (r << 3) | (r >> 2);
    g = (g << 2) | (g >> 4);
    b = (b << 3) | (b >> 2);

    // Calculate luminance (standard formula)
    uint16_t lum = (299 * r + 587 * g + 114 * b) / 1000;

    return (lum > 127) ? 1 : 0;
}

// Convert a full RGB565 sprite array to monochrome format
// Input: rgb565_data - array of uint16_t in RGB565 format
// Input: pixel_count - number of pixels (width * height)
// Output: mono_data - buffer to store monochrome bytes (must be at least pixel_count/8 + 1 bytes)
void convert_sprite_to_monochrome(const uint16_t* rgb565_data, uint32_t pixel_count, uint8_t* mono_data) {
    uint32_t byte_idx = 0;
    uint8_t current_byte = 0;
    uint8_t bit_pos = 0;

    for (uint32_t i = 0; i < pixel_count; i++) {
        uint8_t bit = rgb565_to_mono(rgb565_data[i]);
        current_byte |= (bit << bit_pos);
        bit_pos++;

        if (bit_pos == 8) {
            mono_data[byte_idx++] = current_byte;
            current_byte = 0;
            bit_pos = 0;
        }
    }

    // Write remaining bits
    if (bit_pos > 0) {
        mono_data[byte_idx] = current_byte;
    }
}

#endif

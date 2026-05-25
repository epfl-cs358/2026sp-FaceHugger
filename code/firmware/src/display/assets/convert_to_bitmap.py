#!/usr/bin/env python3
"""Convert images to Adafruit GFX bitmap format for SSD1306 OLED"""

from PIL import Image
import os

WIDTH = 128
HEIGHT = 64


def image_to_bitmap(img_path, output_name):
    img = Image.open(img_path).convert('1')
    img = img.resize((WIDTH, HEIGHT))

    # Adafruit GFX drawBitmap expects row-major, 1 bit/pixel, MSB-leftmost.
    row_bytes = (WIDTH + 7) // 8
    bitmap = bytearray(row_bytes * HEIGHT)
    pixels = img.load()

    for y in range(HEIGHT):
        for x in range(WIDTH):
            if pixels[x, y]:
                byte_idx = y * row_bytes + (x // 8)
                bit_idx = 7 - (x % 8)
                bitmap[byte_idx] |= (1 << bit_idx)

    header = f"// Auto-generated bitmap for {output_name}\n"
    header += f"// Size: {WIDTH}x{HEIGHT} pixels\n\n"
    header += f"const uint8_t {output_name}[] PROGMEM = {{\n"

    for i in range(0, len(bitmap), 16):
        header += "    "
        header += ", ".join(f"0x{b:02x}" for b in bitmap[i:i+16])
        header += ",\n" if i + 16 < len(bitmap) else "\n"

    header += "};\n"
    return header


if __name__ == "__main__":
    # 1. Get the directory where this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))

    images = [
        ("eyes_front.png", "eyes_front"),
        ("eyes_right.png", "eyes_right"),
        ("eyes_left.png", "eyes_left"),
        ("eyes_confused_1.png", "eyes_confused_1"),
        ("eyes_confused_2.png", "eyes_confused_2"),
    ]

    # 2. Save the output file in the same directory as the script
    output_path = os.path.join(script_dir, "eye_sprites.h")
    output = open(output_path, "w")
    output.write("#ifndef EYE_SPRITES_H\n#define EYE_SPRITES_H\n\n")

    for img_file, var_name in images:
        # 3. Create the full absolute path to the image
        full_img_path = os.path.join(script_dir, img_file)
        
        if os.path.exists(full_img_path):
            print(f"Converting {img_file}...")
            # Pass the full path to your conversion function
            header = image_to_bitmap(full_img_path, var_name)
            output.write(header + "\n")
        else:
            print(f"⚠️  {img_file} not found at {full_img_path}")

    output.write("#endif\n")
    output.close()
    print("✓ Generated eye_sprites.h")
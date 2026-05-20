#!/usr/bin/env python3
"""Convert images to Adafruit GFX bitmap format for SSD1306 OLED"""

from PIL import Image
import sys
import os

def image_to_bitmap(img_path, output_name):
    """Convert image to 128x64 bitmap for SSD1306"""
    img = Image.open(img_path).convert('1')  # 1-bit black/white
    img = img.resize((128, 64))

    # Bitmap: 128x64 = 1024 bytes
    # Each byte = 8 vertical pixels (bit 0 at top)
    bitmap = bytearray(1024)
    pixels = img.load()

    for y in range(64):
        for x in range(128):
            if pixels[x, y]:  # White pixel
                byte_idx = (y // 8) * 128 + x
                bit_idx = y % 8
                bitmap[byte_idx] |= (1 << bit_idx)

    # Generate C++ header
    header = f"// Auto-generated bitmap for {output_name}\n"
    header += f"// Size: 128x64 pixels\n\n"
    header += f"const uint8_t {output_name}[] PROGMEM = {{\n"

    for i in range(0, len(bitmap), 16):
        header += "    "
        header += ", ".join(f"0x{b:02x}" for b in bitmap[i:i+16])
        if i + 16 < len(bitmap):
            header += ",\n"
        else:
            header += "\n"

    header += "};\n"
    return header

#!/usr/bin/env python3
"""Convert images to Adafruit GFX bitmap format for SSD1306 OLED"""

from PIL import Image
import sys
import os

def image_to_bitmap(img_path, output_name):
    """Convert image to 128x64 bitmap for SSD1306"""
    img = Image.open(img_path).convert('1')  # 1-bit black/white
    img = img.resize((128, 64))

    # Bitmap: 128x64 = 1024 bytes
    # Each byte = 8 vertical pixels (bit 0 at top)
    bitmap = bytearray(1024)
    pixels = img.load()

    for y in range(64):
        for x in range(128):
            if pixels[x, y]:  # White pixel
                byte_idx = (y // 8) * 128 + x
                bit_idx = y % 8
                bitmap[byte_idx] |= (1 << bit_idx)

    # Generate C++ header
    header = f"// Auto-generated bitmap for {output_name}\n"
    header += f"// Size: 128x64 pixels\n\n"
    header += f"const uint8_t {output_name}[] PROGMEM = {{\n"

    for i in range(0, len(bitmap), 16):
        header += "    "
        header += ", ".join(f"0x{b:02x}" for b in bitmap[i:i+16])
        if i + 16 < len(bitmap):
            header += ",\n"
        else:
            header += "\n"

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
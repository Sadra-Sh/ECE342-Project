import serial
import time
import numpy as np

SERIAL_PORT = '/dev/cu.usbserial-110'
BAUD_RATE = 115200
TARGET_WIDTH = 144
TARGET_HEIGHT = 174
CHECKER_SIZE = 12

def scale_to_4bit(value):
    return min(15, value // 17)

def encode_rle(image_array):
    flattened = image_array.flatten()
    encoded = bytearray()
    current_val = scale_to_4bit(flattened[0])
    run_length = 1
    
    for pixel in flattened[1:]:
        pixel_val = scale_to_4bit(pixel)
        if pixel_val == current_val and run_length < 15:
            run_length += 1
        else:
            encoded_byte = (current_val << 4) | run_length
            encoded.append(encoded_byte)
            current_val = pixel_val
            run_length = 1
    
    encoded_byte = (current_val << 4) | run_length
    encoded.append(encoded_byte)
    return encoded

def generate_checkerboard(width, height, square_size):
    checkerboard = np.zeros((height, width), dtype=np.uint8)
    for y in range(height):
        for x in range(width):
            checkerboard[y, x] = 0 if (x // square_size + y // square_size) % 2 == 0 else 255
    return checkerboard

def main():
    checkerboard = generate_checkerboard(TARGET_WIDTH, TARGET_HEIGHT, CHECKER_SIZE)
    rle_data = encode_rle(checkerboard)
    print(f"Encoded {len(rle_data)} bytes (Compression: {TARGET_WIDTH*TARGET_HEIGHT/len(rle_data):.1f}x)")

    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            time.sleep(2)  # Extended delay
            ser.write(rle_data)
            print("Image sent successfully!")
    except serial.SerialException as e:
        print(f"ERROR: Could not open {SERIAL_PORT}")
        print("Possible fixes:")
        print("1. Unplug/replug the ESP8266")
        print("2. Run 'lsof /dev/cu.usbserial-110' to check for conflicts")
        print("3. Try a different serial port")

if __name__ == "__main__":
    main()
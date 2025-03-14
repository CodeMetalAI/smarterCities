
### micropython firmware needs to be flashed 
# https://micropython.org/download/ESP32_GENERIC_S3/

# ls /dev/tty.*
# esptool.py --chip esp32s3 --port /dev/tty.usbserial-0001 erase_flash
# esptool.py --chip esp32s3 --port /dev/tty.usbserial-0001 write_flash -z 0 ESP32_GENERIC_S3-20241129-v1.24.1.bin



# make sure you don't have arduino IDE open or anything else that might take control of the device (blocking rshell/minicom)

#### to run micropython in microcontroller, 2 options: 
# 1) Thonny: recommended approach
# just download it, attach board, configure board on bottom panel on the right in Thonny. use "tools" on the toolbar to add 
# 
# 2) rshell: simple CLI tool to get to python REPL 
# pip install rshell esptool
# rshell -p /dev/tty.usbserial-0001  
# cp /Users/username/Desktop/smarterCities/microPyTest.py /pyboard/ 
# repl


# ESP32-S3 has special SIMD instructions that improve performance 

# 7x speed up with dsps_dotprod_f32
# 700x speed up with c++ over micropython

# 17 microseconds in c++ 
# 12461 microseconds in micropython

import time
import array

def benchmark():
    # Create array of 1000 integers
    SIZE = 1000
    numbers = array.array('i', [0] * SIZE)
    
    # Time the operations
    start = time.time_ns()
    
    # Fill and manipulate array
    for i in range(SIZE):
        numbers[i] = i * 3  # multiply
        numbers[i] += 2     # add
        numbers[i] //= 2    # divide
    
    end = time.time_ns()
    
    # Calculate and print sum to ensure compiler doesn't optimize away
    total = sum(numbers)
    print(f"Time taken: {(end-start)/1e3} microseconds")
    print(f"Checksum: {total}")  # Verify results match between implementations

# Run benchmark
benchmark()

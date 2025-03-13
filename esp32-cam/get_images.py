#!/usr/bin/env python3
"""
ESP32 Camera UDP Image Receiver

This script receives image data sent from an ESP32 camera module over UDP
and reconstructs the complete images before saving them to disk.

Usage:
    python esp32_camera_receiver.py

The script will listen on the configured UDP port for incoming image data,
reassemble the image chunks, and save complete images to the 'captured_images' directory.
"""

import socket
import struct
import os
import time
from datetime import datetime
import threading

# Configuration
UDP_IP = "0.0.0.0"  # Listen on all available interfaces
UDP_PORT = 4210     # Must match the port used by the ESP32
BUFFER_SIZE = 1500  # Slightly larger than typical MTU
SAVE_DIRECTORY = "captured_images"

# Create the save directory if it doesn't exist
if not os.path.exists(SAVE_DIRECTORY):
    os.makedirs(SAVE_DIRECTORY)
    print(f"Created directory: {SAVE_DIRECTORY}")

# Global variables for reassembling images
current_image_data = {}
current_image_size = 0
image_counter = 0
last_packet_time = 0
image_timeout = 5  # seconds to wait before considering an image transfer complete/failed

def save_image(image_data, image_number):
    """Save the reassembled image data to a file."""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{SAVE_DIRECTORY}/motion_detected_{timestamp}_{image_number}.jpg"
    
    with open(filename, "wb") as f:
        f.write(image_data)
    
    print(f"Image saved as {filename} ({len(image_data)} bytes)")

def check_for_timeout():
    """Periodically check if we need to save a partial image due to timeout."""
    global current_image_data, current_image_size, image_counter, last_packet_time
    
    while True:
        time.sleep(1)  # Check every second
        
        # If we have image data and it's been more than image_timeout seconds since the last packet
        if current_image_data and (time.time() - last_packet_time) > image_timeout and current_image_size > 0:
            
            # Check if we have at least 80% of the expected packets
            expected_packets = (current_image_size + 1024 - 1) // 1024  # Ceiling division
            received_packets = len(current_image_data)
            
            if received_packets > 0:
                print(f"Image transfer timed out. Received {received_packets}/{expected_packets} packets.")
                
                # Try to reconstruct what we have
                if received_packets >= expected_packets * 0.8:  # At least 80% complete
                    try:
                        # Sort packets by packet number and concatenate
                        packets = sorted(current_image_data.items())
                        image_bytes = b''.join(data for _, data in packets)
                        
                        # Save what we have
                        save_image(image_bytes, image_counter)
                        image_counter += 1
                    except Exception as e:
                        print(f"Error saving partial image: {e}")
                else:
                    print("Not enough packets received to save a useful image.")
            
            # Reset state
            current_image_data = {}
            current_image_size = 0

# float('Inf')

timeout_thread = threading.Thread(target=check_for_timeout, daemon=True)
timeout_thread.start()

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
# sock.settimeout(float('Inf'))

print(f"UDP server started on {UDP_IP}:{UDP_PORT}")
print("Waiting for images from ESP32...")

# data, addr = sock.recvfrom(BUFFER_SIZE)

def main():
    try:
        while True:
            data, addr = sock.recvfrom(BUFFER_SIZE)
            last_packet_time = time.time()
            
            # Check if this is an image size header packet (4 bytes)
            if len(data) == 4:
                # New image is starting
                current_image_size = struct.unpack("<I", data)[0]
                current_image_data = {}
                print(f"New image transfer starting, expected size: {current_image_size} bytes")
                continue
            
            # Check if this is an image data packet
            if len(data) >= 4:  # At least 4 bytes for headers
                try:
                    # Extract packet number and total packets (first 4 bytes: 2 bytes each)
                    packet_number = struct.unpack("<H", data[0:2])[0]
                    total_packets = struct.unpack("<H", data[2:4])[0]
                    
                    # Extract image data (remaining bytes)
                    image_chunk = data[4:]
                    
                    # Store this chunk
                    current_image_data[packet_number] = image_chunk
                    
                    # Progress indicator (print every 10th packet to avoid console spam)
                    if packet_number % 10 == 0 or packet_number == total_packets - 1:
                        print(f"Received packet {packet_number+1}/{total_packets} from {addr[0]}")
                    
                    # Check if we have all packets
                    if len(current_image_data) == total_packets:
                        # Sort packets by packet number and concatenate
                        packets = sorted(current_image_data.items())
                        image_bytes = b''.join(data for _, data in packets)
                        
                        # Verify the size
                        if len(image_bytes) == current_image_size or abs(len(image_bytes) - current_image_size) < 100:
                            save_image(image_bytes, image_counter)
                            image_counter += 1
                            
                            # Reset state
                            current_image_data = {}
                            current_image_size = 0
                        else:
                            print(f"Size mismatch: Expected {current_image_size}, got {len(image_bytes)}")
                except Exception as e:
                    print(f"Error processing packet: {e}")

    except KeyboardInterrupt:
        print("\nShutting down server...")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
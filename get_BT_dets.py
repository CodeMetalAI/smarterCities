
import asyncio
from bleak import BleakClient, BleakScanner
import datetime
import pandas as pd
from collections import defaultdict
import time 

SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

class BLEManager:
    def __init__(self):
        self.active_clients = {}
        self.detections = []
        self.df = pd.DataFrame()
        
    def notification_handler(self, scanner_address, data):
        try:
            message = data.decode()
            device_id, mac, rssi, timestamp, device_name = message.split(',')
            timestamp_ms = int(timestamp) / 1000000
            time_str = datetime.datetime.fromtimestamp(timestamp_ms).strftime('%H:%M:%S.%f')
            
            detection = {
                'timestamp': time_str,
                'scanner_id': device_id,
                'scanner_address': scanner_address,
                'detected_device': device_name,
                'detected_mac': mac,
                'rssi': int(rssi)
            }
            
            self.detections.append(detection)
            self.update_dataframe()
            print(f"Detection from {device_id}: {device_name} ({mac}) at {rssi}dBm")
            
        except Exception as e:
            print(f"Error parsing data: {e}")
    
    def update_dataframe(self):
        self.df = pd.DataFrame(self.detections)
        print("\nUpdated DataFrame:")
        print(f"Total detections: {len(self.df)}")
        print(f"Unique scanners: {self.df['scanner_id'].nunique()}")
        print(f"Unique devices: {self.df['detected_mac'].nunique()}")
        
    async def handle_scanner(self, device):
        if not device.name or 'Scanner' not in device.name:
            return
            
        scanner_address = device.address
        
        try:
            if scanner_address in self.active_clients:
                return
                
            client = BleakClient(scanner_address, timeout=20.0)
            self.active_clients[scanner_address] = client
            
            try:
                await client.connect()
                print(f"\nConnected to {device.name} ({scanner_address})")
                
                services = await client.get_services()
                for service in services:
                    for char in service.characteristics:
                        if "notify" in char.properties:
                            handler = lambda s, d: self.notification_handler(scanner_address, d)
                            await client.start_notify(char.uuid, handler)
                
                while await client.is_connected():
                    await asyncio.sleep(1)
                    
            except Exception as e:
                print(f"Error with {device.name}: {e}")
            finally:
                await client.disconnect()
                del self.active_clients[scanner_address]
                print(f"\nDisconnected from {device.name}")
                
        except Exception as e:
            print(f"Failed to handle {device.name}: {e}")
            if scanner_address in self.active_clients:
                del self.active_clients[scanner_address]

async def main():
    manager = BLEManager()
    tic = time.time()
    
    while True:
        try:
            toc = time.time()-tic
            if toc > 60:
                BLEManager.df.to_csv('/Users/mbenton/Desktop/smarterCities/detections.csv', index=False)
                return
            devices = await BleakScanner.discover()
            print('time since start: '+str(time.time()-tic))
            scanner_tasks = [manager.handle_scanner(device) for device in devices]
            await asyncio.gather(*scanner_tasks)
        except Exception as e:
            print(f"Scan error: {e}")
        await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())

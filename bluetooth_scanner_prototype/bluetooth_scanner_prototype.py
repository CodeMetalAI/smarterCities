from machine import Pin, I2C
import network
import ujson
import ubinascii
import time
from umqtt.simple import MQTTClient
import bluetooth
from micropython import const
from ssd1306 import SSD1306_I2C

# pip install ujson  ssd1306 

# Configuration File
CONFIG_FILE = "config.json"

def load_config():
    try:
        with open(CONFIG_FILE, "r") as f:
            return ujson.load(f)
    except:
        return {"ssid": "", "password": ""}

config = load_config()

# WiFi Setup
wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.connect(config["ssid"], config["password"])

while not wifi.isconnected():
    print("Connecting to WiFi...")
    time.sleep(1)

print("Connected! IP:", wifi.ifconfig()[0])

# OLED Display Setup
WIDTH = 128
HEIGHT = 64
SDA_OLED = 21
SCL_OLED = 22

i2c = I2C(0, scl=Pin(SCL_OLED), sda=Pin(SDA_OLED))
display = SSD1306_I2C(WIDTH, HEIGHT, i2c)

def update_display(text):
    display.fill(0)
    display.text(text, 0, 0)
    display.show()

update_display("BT Scanner Ready")

# BLE Setup
bt = bluetooth.BLE()
scanner = bt.gap_scan(2000, 30000, 30000)

def bt_callback(event, data):
    if event == bluetooth._IRQ_SCAN_RESULT:
        addr_type, addr, adv_type, rssi, adv_data = data
        mac_address = ubinascii.hexlify(addr).decode()
        print(f"Device Found: {mac_address} RSSI: {rssi}")
        update_display(f"MAC: {mac_address} \nRSSI: {rssi}")

bt.irq(bt_callback)

print("Scanning for BLE devices...")

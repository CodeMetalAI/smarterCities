import network
from machine import Pin, I2C
from time import sleep
from ssd1306 import SSD1306_I2C


rst = Pin(21, Pin.OUT)
rst.value(1)

# OLED Display Configuration
i2c = I2C(scl=Pin(18), sda=Pin(17), freq=500000)
from machine import SoftI2C
i2c = SoftI2C(scl=Pin(18), sda=Pin(17), freq=400000)  # Lower frequency

display = SSD1306_I2C(128, 64, i2c, addr=0x3c)

def create_sta():
    sta_if = network.WLAN(network.STA_IF)
    if not sta_if.active():
        print("Activating Wi-Fi STA interface...")
        sta_if.active(True)
        sleep(1)  # Give some time for activation
        print(f"STA Active: {sta_if.active()}")
    else:
        print("STA interface already active.")
    return sta_if

def decode_security(code:int)->str:
    decoder = {0 : 'open',
               1 : 'WEP',
               2 : 'WPA-PSK',
               3 : 'WPA2-PSK',
               4 : 'WPA/WPA2-PSK'}
    try:
        out = decoder[code]
    except KeyError:
        out = "UNKNOWN"
    return out

def display_networks(networks):
    #setting up networks for (Name, Security Protocol, RSSI)
    networks = [[net[0],net[4],net[3]] for net in networks]
    display.fill(0)
    #List of all unique non empty strings
    networks = [(x[0].decode("utf-8"),decode_security(x[1]),abs(x[2])) for x in networks if x[0]]
    unique_networks = []
    for i, net in enumerate(networks):
        if net[0] in unique_networks:
            del networks[i]
        else:
            unique_networks.append(net[0])
    sorted_networks = sorted(networks, key=lambda x:abs(x[-1]))
    #Get top 3 networks
    if len(sorted_networks) > 3:
        top_networks = networks[:3]
    else:
        top_networks = networks
    
    #display.text("Wi-Fi Networks:", 0, 0, 1)
    y = 0
    for i, net in enumerate(top_networks):
        str1 = "{num}.){net_name}".format(num=i+1, net_name=str(net[0]))
        str2 = "S:{security}|P:{ping}".format(security=net[1], ping=net[2])
        print(str1)
        print(str2)
        display.text(str1, 0, y, 1)
        display.text(str2, 0, y+8, 1)
        y += 16
    display.show()

def network_finder(sta_if):
    networks = sta_if.scan() #Capture Networks
    print(f"\nFound {len(networks)} networks.")
    display_networks(networks)

def main():
    display.text("Starting up...",0,0, 0)
    display.show()
    sta_if = create_sta() 
    while True:
        network_finder(sta_if)
        sleep(0.3)  # Scan every 10 seconds

if __name__ == "__main__":
    main()

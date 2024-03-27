import time
from bmp280 import BMP280
from smbus2 import SMBus
import paho.mqtt.client as mqtt
import json

import time
from smbus2 import SMBus, i2c_msg

# Create an I2C bus object
bus = SMBus(0)
address = 0x76
addressbh1750 = 0x23

# Setup BMP280
bmp280 = BMP280(i2c_addr= address, i2c_dev=bus)
interval = 15 # Sample period in seconds

# Setup BH1750
bus.write_byte(addressbh1750, 0x10)
bytes_read = bytearray(2)
interval = 15 # Sample period in seconds

# MQTT settings
MQTT_HOST ="mqtt3.thingspeak.com"
MQTT_PORT = 1883
MQTT_KEEPALIVE_INTERVAL =60
MQTT_TOPIC = "channels/2459557/publish"
MQTT_CLIENT_ID = "ORwyAwIfNCgYIwcGEQUBPDw"
MQTT_USER = "ORwyAwIfNCgYIwcGEQUBPDw"
MQTT_PWD = "sSntnoc3lIXP0R727kfl2YL5"


def on_connect(client, userdata, flags, rc):
    if rc==0:
        print("Connected OK with result code "+str(rc))
    else:
        print("Bad connection with result code "+str(rc))

def on_disconnect(client, userdata, flags, rc=0):
    print("Disconnected result code "+str(rc))

def on_message(client,userdata,msg):
    print("Received a message on topic: " + msg.topic + "; message: " + msg.payload)


# Set up a MQTT Client
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, MQTT_CLIENT_ID)
client.username_pw_set(MQTT_USER, MQTT_PWD)

# Connect callback handlers to client
client.on_connect= on_connect
client.on_disconnect= on_disconnect
client.on_message= on_message

print("Attempting to connect to %s" % MQTT_HOST)
client.connect(MQTT_HOST, MQTT_PORT)
client.loop_start() #start the loop

def get_value(bus, address):
    write = i2c_msg.write(address, [0x10])  # 11x resolution 120ms see datasheet
    read = i2c_msg.read(address, 2)  # Define 'read' here
    bus.i2c_rdwr(write, read)
    bytes_read = list(read)
    return (((bytes_read[0]&3)<<8) + bytes_read[1])/1.2  # conversion see datasheet


PREDIFINED_TEMPS = [20.0, 25.0, 30.0]
PREDIFINED_LUXES = [200.0, 400.0, 600.0]


while True:
    # Measure temperature
    bmp280_temperature = bmp280.get_temperature()

    # Measure lux
    bh1750_lux = get_value(bus, addressbh1750) 

    # Prepare MQTT data as a query string
    MQTT_DATA = "field1={}&field2={}".format(bmp280_temperature, bh1750_lux)
    print(MQTT_DATA)

    try:
        # Check if the client is connected before publishing
        if client.is_connected():
            client.publish(topic=MQTT_TOPIC, payload=MQTT_DATA, qos=0, retain=False)
        else:
            print("Client is not connected to ThingSpeak. Attempting to connect.")
            client.reconnect()
        time.sleep(interval)
    except OSError:
        print("An error occurred. Attempting to reconnect.")
        client.reconnect()






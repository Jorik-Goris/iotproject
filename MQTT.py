import time
from bmp280 import BMP280
import paho.mqtt.client as mqtt
import wiringpi as wiringpi
import datetime
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


# Initialize WiringPi
wiringpi.wiringPiSetup()

# Set the pin mode
temp_button_pin = 5
lux_button_pin = 6
wiringpi.pinMode(temp_button_pin, wiringpi.INPUT)
wiringpi.pinMode(lux_button_pin, wiringpi.INPUT)
                 
# Define the target values
temp_options = [20, 25, 30]
lux_options = [50, 100, 200]

# Initialize the current option indices

target_temperature = 25
target_lux = 100
temp_counter = 0
lux_counter = 0


# LED control

# Initialize the GPIO pin for the LED
led_pin = 2  # Change this to the pin number you are using
wiringpi.pinMode(led_pin, wiringpi.GPIO.PWM_OUTPUT)

# Create a software PWM pin
wiringpi.softPwmCreate(led_pin, 0, 100)  # Range from 0-100


pwm_value = 50  # Change this to your desired initial brightness
wiringpi.softPwmWrite(led_pin, pwm_value)

# Temp control
relay_pin = 3
wiringpi.pinMode(relay_pin, 1)

# Main loop

while True:

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

    print("-----------------")


    # Check if the temperature button is pressed
    if wiringpi.digitalRead(temp_button_pin) == wiringpi.LOW:
        # Increment the counter and get the next target temperature
        temp_counter = (temp_counter + 1) % len(temp_options)
        target_temperature = temp_options[temp_counter]

    # Check if the lux button is pressed
    if wiringpi.digitalRead(lux_button_pin) == wiringpi.LOW:
        # Increment the counter and get the next target lux
        lux_counter = (lux_counter + 1) % len(lux_options)
        target_lux = lux_options[lux_counter]



    # Print date and time of measurement
    print(datetime.datetime.now())

    # Measure temperature
    bmp280_temperature = bmp280.get_temperature()
    print("Current temperature: {:.2f} C".format(bmp280_temperature))
    print("Target temperature: {} C".format(target_temperature))

    # Print the state of the lux button
    temp_button_state = wiringpi.digitalRead(temp_button_pin)
    print("Temp button state: {}".format("LOW" if temp_button_state == wiringpi.LOW else "HIGH"))
    
        # If temperature is below target, turn on relay to heat up
    if bmp280_temperature < target_temperature:
        wiringpi.digitalWrite(relay_pin, 1)  # Turn on relay
    else:
        wiringpi.digitalWrite(relay_pin, 0)  # Turn off relay

    print()

    # Measure lux
    bh1750_lux = get_value(bus, addressbh1750)
    print("Current light: {:.2f} Lux".format(bh1750_lux))
    print("Target light: {} Lux".format(target_lux))

     # Print the state of the lux button
    lux_button_state = wiringpi.digitalRead(lux_button_pin)
    print("Lux button state: {}".format("LOW" if lux_button_state == wiringpi.LOW else "HIGH"))

        # Calculate the difference between the target and current lux
    lux_difference = target_lux - bh1750_lux

    # Adjust the PWM value based on the lux difference
    if lux_difference > 0:
        pwm_value = min(100, pwm_value + lux_difference)  # Increase brightness, but not more than 100
    else:
        pwm_value = max(0, pwm_value + lux_difference)  # Decrease brightness, but not less than 0

    # Set the new PWM value
    wiringpi.softPwmWrite(led_pin, int(pwm_value))

    print("-----------------")

    # Prepare MQTT data as a query string, also send target values to dashboard
    MQTT_DATA = "field1={}&field2={}&field3={}&field4={}".format(bmp280_temperature, target_temperature, bh1750_lux, target_lux)
    


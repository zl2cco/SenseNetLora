"""
Example for using the RFM9x Radio with Raspberry Pi.

Learn Guide: https://learn.adafruit.com/lora-and-lorawan-for-raspberry-pi
Author: Brent Rubell for Adafruit Industries
"""
# Import Python System Libraries
import time
#import array
#import math

# Import Blinka Libraries
import busio
from digitalio import DigitalInOut, Direction, Pull
import board
# Import the SSD1306 module.
import adafruit_ssd1306
# Import RFM9x
import adafruit_rfm9x
import struct 
import paho.mqtt.publish as publish
import json


# Button A
btnA = DigitalInOut(board.D5)
btnA.direction = Direction.INPUT
btnA.pull = Pull.UP

# Button B
btnB = DigitalInOut(board.D6)
btnB.direction = Direction.INPUT
btnB.pull = Pull.UP

# Button C
btnC = DigitalInOut(board.D12)
btnC.direction = Direction.INPUT
btnC.pull = Pull.UP

# Create the I2C interface.
i2c = busio.I2C(board.SCL, board.SDA)

# 128x32 OLED Display
reset_pin = DigitalInOut(board.D4)
display = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c, reset=reset_pin)
# Clear the display.
display.fill(0)
display.show()
width = display.width
height = display.height

# Configure LoRa Radio
CS = DigitalInOut(board.CE1)
RESET = DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, 915.0)
rfm9x.tx_power = 23
prev_packet = None

print("LORA Gateway started")

while True:
    packet = None
    # draw a box to clear the image
    #display.fill(0)
    #display.text('RasPi LoRa', 35, 0, 1)

    # check for packet rx
    packet = rfm9x.receive(with_header=True)
    if packet is None:
        #display.show()
        seconds = int(time.time() % 60)
        display.fill_rect(64, 0, 15, 8, False)
        display.text(str(seconds), 64, 0, 1, font_name="/home/pi/font5x8.bin")
    else:
        packet_len = len(packet)
        packet_time = time.strftime("%H:%M:%S")
        #print("Len: ", packet_len)
        #print("Data: ", [hex(x) for x in packet])
        if packet_len == 40:
            vars = struct.unpack('BxxxxxxxdddIxxxx', packet)
            #print("Data: ", [x for x in vars])

            node_id = vars[0]
            distance_m = vars[1]
            vbat = vars[2]
            temperature = vars[3]

            if distance_m <= 3.0:
                measurement = {"node_id":node_id, "level":3.0 - distance_m, "vbat":vbat, "temp": temperature}
                mjson = json.dumps(measurement)
                # publish.single("maintank", mjson, hostname="rpi4.local")
                # publish.single("maintank/level", 3.0 - distance_m[0], hostname="rpi4.local")
                # publish.single("maintank/vbat", vbat[0], hostname="rpi4.local")

                # Display the packet text and rssi
                #display.fill(0)
                prev_packet = packet
                distance_text = "{:.2f} ".format(3.0 - distance_m) + "m"
                vbat_text = "{:.2f}".format(vbat) + "V"
                nodeid_text = "[{:d}] ".format(node_id)
                display.fill_rect(0, 0,128,8,0)
                display.text(packet_time, 0, 0, 1, font_name="/home/pi/font5x8.bin")
                display.fill_rect(0, node_id*12,128,8,0)
                display.text(nodeid_text, 0, (node_id)*12, 1, font_name="/home/pi/font5x8.bin")
                display.text(distance_text, 20, (node_id)*12, 1, font_name="/home/pi/font5x8.bin")
                display.text(vbat_text, 64, (node_id)*12, 1, font_name="/home/pi/font5x8.bin")
                #time.sleep(10)

    if not btnA.value:
        # Send Button A
        display.fill(0)
        button_a_data = bytes("Button A!\r\n","utf-8")
        rfm9x.send(button_a_data)
        display.text('Sent Button A!', 25, 15, 1)
    elif not btnB.value:
        # Send Button B
        display.fill(0)
        button_b_data = bytes("Button B!\r\n","utf-8")
        rfm9x.send(button_b_data)
        display.text('Sent Button B!', 25, 15, 1)
    elif not btnC.value:
        # Send Button C
        display.fill(0)
        button_c_data = bytes("Button C!\r\n","utf-8")
        rfm9x.send(button_c_data)
        display.text('Sent Button C!', 25, 15, 1)


    display.show()
    #time.sleep(5.0)


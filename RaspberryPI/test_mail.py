import time

from numpy import true_divide
#import smbus
import random
import smtplib
#import RPi.GPIO as GPIO

i2c_ch = 1

i2c_addr = 0x4d

reg_temp = 0x00

old_temp = 0

# GPIO.setmode(GPIO.BCM)
# GPIO.setwarnings(False)
# GPIO.setup(18,GPIO.OUT)
# GPIO.setup(19,GPIO.OUT)

def send_email(mail):
    server = smtplib.SMTP('smtp.gmail.com', 587)
    server.starttls()
    server.login("raspeberryase@gmail.com", "PwL28JgW9ncQ4tq")
    server.sendmail("raspeberryase@gmail.com","joaoresendeoliveira@gmail.com", mail)
    server.quit()


def read_temperature():

    # Read temperature registers
    val = bus.read_i2c_block_data(i2c_addr, reg_temp, 2)
    # NOTE: val[0] = MSB byte 1, val [1] = LSB byte 2

    temp_c = (val[0] << 4) | (val[1] >> 4)

    # Convert to 2s complement (temperatures can be negative)
    if (temp_c & (1 << (12 - 1))) != 0:
        temp_c = temp_c - (1 << 12)

    # Convert registers value to temperature (C)
    temp_c = temp_c * 0.0625

    return temp_c

    #todo analyse what the addresses of our sensors use, and go from there

#bus = smbus.SMBus(i2c_ch)

while True:
    #determine what to read
    #after reading watch the interval
    #if (interval between 20/25 its ok, else turn on)
    #send notification email 
    #temperature=read_temperature()
    # print("current temperature" , temperature)
    
    # if(temperature < 20):
    #     GPIO.output(18,GPIO.HIGH)
    #     mail = "AC was turned on because the temperature dropped below 20º"
    #     send_email(mail)
    
    # if(temperature > 25):
    #     mail = "AC was turned on because the temperature surpassed 25º"
    #     send_email(mail)
    if(True):
        mail="hello"
        send_email(mail)
    
    time.sleep(1)
    

import time
from numpy import true_divide
import smbus
import random
import smtplib
import RPi.GPIO as GPIO

i2c_ch = 1

i2c_addr = 0x4d

reg_temp = 0x00

old_temp = 0

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.cleanup()

#pins TC74
GPIO.setup(16, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)

#pins HC-SR04
ECHO = 20
TRIG = 21
GPIO.setup(26, GPIO.OUT)

#setup pins ultrasound
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.output(16, GPIO.LOW)
GPIO.output(19, GPIO.LOW)
GPIO.output(26, GPIO.LOW)

print("PINS SETUP DONE")


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

def read_distance():
    distance = 0
    GPIO.output(TRIG, 0)
    time.sleep(1)

    GPIO.output(TRIG, 1)
    time.sleep(0.00001)
    GPIO.output(TRIG, 0)

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
    
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)

    return distance


bus = smbus.SMBus(i2c_ch)

old_distance=0


while True:
    temperature=read_temperature()
    print("Temperature: ", temperature, " degrees")
    
    distance = read_distance()
    print("Distance: ", distance, " cm")

    #send notifications
    if(temperature < 20):
        GPIO.output(16, GPIO.HIGH) # --------- COLD ------------
        mail = 'Subject: {}\n\n{}'.format("Temperature", "AC was turned on because the temperature dropped below 20 degrees")
        send_email(mail)
    
    if(temperature > 25):
        GPIO.output(19, GPIO.HIGH) # --------- HOT -------------
        mail = 'Subject: {}\n\n{}'.format("Temperature", "AC was turned on because the temperature surpassed 25 degrees")
        send_email(mail)
    
    if(20>temperature>25):
        GPIO.output(16, GPIO.LOW)
        GPIO.output(19, GPIO.LOW)

    if(distance < 5):
        GPIO.output(26, GPIO.LOW)
    if(distance > 5 and old_distance > 5):
        timeout = time.time() + 10
        while time.time() < timeout:
            GPIO.output(26, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(26, GPIO.LOW)
            time.sleep(0.5)
        
        dist = read_distance()
        if dist > 5:
            mail = 'Subject: {}\n\n{}'.format("Door","Front door was left open, go check")
            send_email(mail)

    old_distance = distance

    time.sleep(5)


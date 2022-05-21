import time
import smbus
import random
import smtplib

i2c_ch = 1

#i2c_addr = 0x4d

#reg_temp = 0x00

old_temp = 0


def send_email(mail):
    server = smtplib.SMTP('smtp.gmail.com', 587)
    server.starttls()
    server.login("raspeberryase@gmail.com", "PwL28JgW9ncQ4tq")
    server.sendmail("raspeberryase@gmail.com","joaoresendeoliveira@gmail.com", mail)
    server.quit()


def read_bus():

    # Read temperature registers
    val = bus.read_i2c_block_data(i2c_addr, reg_temp, 2)
    # NOTE: val[0] = MSB byte 1, val [1] = LSB byte 2

    #todo analyse what the addresses of our sensors use, and go from there

bus = smbus.SMBus(i2c_ch)

while True:
    #determine what to read
    #after reading watch the interval
    #if (interval between 20/25 its ok, else turn on)
    #send notification email 
    mail = "AC was turned on because the temperature dropped below 20º"
    send_email()



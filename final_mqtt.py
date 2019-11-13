#Libraries
import RPi.GPIO as GPIO
import serial, string, time # GPIO Mode(BOARD / BCM)
import paho.mqtt.client as mqtt
import smbus,math

GPIO.setmode(GPIO.BCM) # set GPIO Pins
GPIO.setwarnings(False)
GPIO_TRIGGER = 18
GPIO_ECHO = 24
GPIO_TRIGGER_BRAKE=17
GPIO_ECHO_BRAKE=23
prev = 0 # set GPIO direction(IN / OUT)
prev_brake=0
rev_state=False
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(GPIO_TRIGGER_BRAKE,GPIO.OUT)
GPIO.setup(GPIO_ECHO_BRAKE,GPIO.IN)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP) #Button to GPIO20
GPIO.setup(22,GPIO.OUT)
GPIO.output(22,False)
GPIO.setup(10,GPIO.OUT)
GPIO.output(10,False)

ser = serial.Serial(port = '/dev/ttyUSB0',
    baudrate = 9600,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS, timeout = 1)

def distance(): #set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True) # set Trigger after 0.01 ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    StartTime = time.time()
    StopTime = time.time() # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time() # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time() # time difference between start and arrival
    TimeElapsed = StopTime - StartTime # multiply with the sonic speed(34300 cm / s)# and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
    return distance

def brake(): #set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER_BRAKE, True) # set Trigger after 0.01 ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER_BRAKE, False)
    StartTime = time.time()
    StopTime = time.time() # save StartTime
    while GPIO.input(GPIO_ECHO_BRAKE) == 0:
        StartTime = time.time() # save time of arrival
    while GPIO.input(GPIO_ECHO_BRAKE) == 1:
        StopTime = time.time() # time difference between start and arrival
    TimeElapsed = StopTime - StartTime # multiply with the sonic speed(34300 cm / s) and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
    return distance
################################################################################

# Register
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

def read_byte(reg):
    return bus.read_byte_data(address, reg)

def read_word(reg):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg+1)
    value = (h << 8) + l
    return value

def read_word_2c(reg):
    val = read_word(reg)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def dist_imu(a,b):
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist_imu(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist_imu(x,z))
    return math.degrees(radians)

bus = smbus.SMBus(1) # bus = smbus.SMBus(0) fuer Revision 1
address = 0x68       # via i2cdetect

# Aktivieren, um das Modul ansprechen zu koennen
bus.write_byte_data(address, power_mgmt_1, 0)


################################################################################


if __name__ == '__main__':
    try:
        while True:
            obstacle=ser.read(100)
            ob=obstacle.decode("utf-8")
            print(ob)
            if(ob == '9'):
                GPIO.output(10,True)
            else:
                GPIO.output(10,False)
            time.sleep(0.02)
            dist = distance()
            print("Measured Distance = %.1f cm" % dist)
            if (dist >= 11):
                ser.write(str.encode('0'));
            elif(dist>7) :
                ser.write(str.encode('1'));
            else:
                ser.write(str.encode('2'));
            time.sleep(0.02)
            bk = brake()
            print("Measured Brake Distance = %.1f cm" % bk)
            if (bk >= 11):
                ser.write(str.encode('a'));
            elif(bk>6) :
                ser.write(str.encode('b'));
            else:
                ser.write(str.encode('c'));
            time.sleep(0.02)
            gyro_xout = read_word_2c(0x43)
            gyro_yout = read_word_2c(0x45)
            gyro_zout = read_word_2c(0x47)
            accl_xout = read_word_2c(0x3b)
            accl_yout = read_word_2c(0x3d)
            accl_zout = read_word_2c(0x3f)
            accl_xout_scaled = accl_xout / 16384.0
            accl_yout_scaled = accl_yout / 16384.0
            accl_zout_scaled = accl_zout / 16384.0
            print("X Rotation: " , get_x_rotation(accl_xout_scaled, accl_yout_scaled, accl_zout_scaled))
            print("Y Rotation: " , get_y_rotation(accl_xout_scaled, accl_yout_scaled, accl_zout_scaled))
            x_val =  get_x_rotation(accl_xout_scaled, accl_yout_scaled, accl_zout_scaled)
            y_val =  get_y_rotation(accl_xout_scaled, accl_yout_scaled, accl_zout_scaled)
            if(x_val>=-16 and x_val<=16 and y_val>=-90 and y_val<=-60):
                ser.write(str.encode('p'))             # car is going straight
            elif(x_val>=17 and x_val<=60 and y_val>=-90 and y_val<=-30):
                ser.write(str.encode('q'))             # car is going left
            elif(x_val>=-60 and x_val<=-30 and y_val>=-60 and y_val<=-20):
                ser.write(str.encode('r'))             # car is going right
            else:
                ser.write(str.encode('x'))             # steering in undefined state
            time.sleep(0.02)
            #GPIO.output(22,True)
            button_state = GPIO.input(27)
            if button_state == GPIO.HIGH:
                rev_state=not rev_state
            if rev_state == True:
                ser.write(str.encode('m'))
                GPIO.output(22,True)
                publisher = mqtt.Client()
                publisher.connect("172.16.117.128",1883,60)
                message = "Going reverse"
                publisher.publish("driving simulator",message)
                publisher.disconnect()
            else:
                ser.write(str.encode('n'))
                GPIO.output(22,False)
                publisher=mqtt.Client()
                publisher.connect("172.16.117.128",1883,60)
                message = "Going forward"
                publisher.publish("driving simulator",message)
                publisher.disconnect()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()

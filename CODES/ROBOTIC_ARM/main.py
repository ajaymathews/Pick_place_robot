import RPi.GPIO as GPIO
from Adafruit_PWM_Servo_Driver import PWM
import math
import subprocess
import serial
import time
import os
import glob
import telegram
from telegram.ext import Updater, CommandHandler
from telegram import InlineKeyboardButton, InlineKeyboardMarkup
import numpy as np
from threading import Thread

ser = serial.Serial(
  
   port='/dev/ttyUSB1',
   baudrate = 9600,
   parity=serial.PARITY_NONE,
   stopbits=serial.STOPBITS_ONE,
   bytesize=serial.EIGHTBITS,
   timeout=1
)
counter=0

trigger_pin = 38
echo_pin = 40

GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BOARD)
GPIO.setup(31,GPIO.OUT)  #PINMODE
GPIO.setup(33,GPIO.OUT)
GPIO.setup(35,GPIO.OUT)
GPIO.setup(37,GPIO.OUT)
GPIO.setup(trigger_pin,0)
GPIO.setup(echo_pin,1)


pwm = PWM(0x40)

# Constants defining the range of the servo
SERVO_MIN = 150
SERVO_MAX = 600
HALF_PI = math.pi / 2.0
MIDPOINT = 375.0
SPAN = 225.0


b=0
ud=1
fb=2
p=3
ti=1
nti=-1
dely=.01

def init():
    pwm.setPWMFreq(60)
    pwm.setPWM(b, 300, 1600)#300-1900
    pwm.setPWM(fb, 0, 300) #800-2000
    pwm.setPWM(ud, 0, 500)#1800-400
    pwm.setPWM(p, 0, 300)#400-1800
    

def front1():
    for i in range(1100,1600,ti):#1000-2000
        pwm.setPWM(fb, 0, i)
        print("front 1  ",i)
        time.sleep(dely)
def front2():
    for i in range(1600,1950,ti):#1000-2000
        pwm.setPWM(fb, 0, i)
        print("front 2  ",i)
        time.sleep(dely)
def down2():
    for i in range (600,400,nti):#1000 - 1700
        pwm.setPWM(ud, 0, i)
        print("down 2  ",i)
        time.sleep(dely)
def front3():
    for i in range(1950,2100,ti):#1000-2000
        pwm.setPWM(fb, 0, i)
        print("front 3  ",i)
        time.sleep(dely)
def grab():
    for i in range (1100,1600,ti):#1800-1000
        pwm.setPWM(p, 0, i)
        print("grab ",i)
        time.sleep(dely)
def release():
    for i in range (1200,1100,nti):#1800-1000
        pwm.setPWM(p, 0, i)
        print("relese ",i)
        time.sleep(dely)

def up1():
    for i in range (1000,1300,ti):#1000 - 1700
        pwm.setPWM(ud, 0, i)
        print("up 1  ",i)
        time.sleep(dely)
def up2():
    for i in range (400,1200,ti):#1000 - 1700
        pwm.setPWM(ud, 0, i)
        print("up 2  ",i)
        time.sleep(dely)
def down1():
    for i in range (1300,600,nti):#1000 - 1700
        pwm.setPWM(ud, 0, i)
        print("down 1  ",i)
        time.sleep(dely)
def down3():
    for i in range (1200,300,nti):#1000 - 1700
        pwm.setPWM(ud, 0, i)
        print("down 1  ",i)
        time.sleep(dely)
def back():
    for i in range(1900,1600,nti):#1000-2000
        pwm.setPWM(fb, 0, i)
        print("back   ",i)
        time.sleep(dely)
def back1():
    for i in range(1900,1100,nti):#1000-2000
        pwm.setPWM(fb, 0, i)
        print("back   ",i)
        time.sleep(dely)
def right():
    for i in range(975,1600,ti):#1000-2000
        pwm.setPWM(b, 0, i)
        print("right   ",i)
        time.sleep(dely)
def centre():
    for i in range(1600,975,nti):#1000-2000
        pwm.setPWM(b, 0, i)
        print("right   ",i)
        time.sleep(dely)   
    
def start():
    release()
    pwm.setPWM(b, 0, 950)
    pwm.setPWM(ud, 0,1000)
    pwm.setPWM(fb, 0, 1100)
    pwm.setPWM(p, 0, 1100)
    pwm.setPWM(b, 0, 950)

def pick():
    up1()
    front1()
    down1()
    front2()
    down2()
    front3()
    grab()
    
def place():
    up2()
    back()
    right()
    front2()
    down3()
    release()
    up2()
    centre()
    back1()
    start()

def forward():
    GPIO.output(33,True) 
    GPIO.output(31,False)
    GPIO.output(37,True)
    GPIO.output(35,False)
    print("moving forward")

def backward():
    GPIO.output(31,True) 
    GPIO.output(33,False)
    GPIO.output(35,True)
    GPIO.output(37,False)#backward
    print("moving backward")

def left():
    GPIO.output(33,True) 
    GPIO.output(31,False)
    GPIO.output(35,True)
    GPIO.output(37,False)#left
    print("turning left")

def right():
    GPIO.output(31,True) 
    GPIO.output(33,False)
    GPIO.output(37,True)
    GPIO.output(35,False)#right
    print("turning right")

def halt():
    GPIO.output(31,False) 
    GPIO.output(33,False)
    GPIO.output(37,False)
    GPIO.output(35,False)#stop
    print("engine off")

    
def send_trigger_pulse():
    GPIO.output(trigger_pin, True)
    time.sleep(0.0001)
    GPIO.output(trigger_pin, False)
def wait_for_echo(value, timeout):
    count = timeout
    while GPIO.input(echo_pin) != value and count > 0:
        count = count - 1
def get_distance():
    send_trigger_pulse()
    wait_for_echo(True, 10000)
    start = time.time()
    wait_for_echo(False, 10000)
    finish = time.time()
    pulse_len = finish - start
    distance_cm = (pulse_len *34300)/2
    print(distance_cm)
    return (distance_cm)


bot = telegram.Bot(token='839381492:AAEk289JvWApNpM-KSuYMOv9LrGH0Pl5I0s') #'627333566:AAGanhIPY2BP7UqOJcdAhjYR1mOJZG6OuyM'
updater = Updater('839381492:AAEk289JvWApNpM-KSuYMOv9LrGH0Pl5I0s')
updater.dispatcher.add_handler(CommandHandler('start', start))
updater.dispatcher.add_handler(CommandHandler('front', front))
updater.dispatcher.add_handler(CommandHandler('back', back))
updater.dispatcher.add_handler(CommandHandler('right', right))
updater.dispatcher.add_handler(CommandHandler('left', left))
updater.dispatcher.add_handler(CommandHandler('stop', stop))
updater.dispatcher.add_handler(CommandHandler('check', check))

print("System Start")
try:    
   updater.start_polling()
   updater.idle()
  
except:
   print("Error: unable to start thread")
    

time.sleep(1)
start()

while 1:

if(get_distance<=10):
   stop()
   
x=ser.readline()
y=x.decode('ascii')
print(y)
if(y=='pic'):
   print('picking')
   time.sleep(1)
   pick()
elif(y=='place'):
   print('placing')
   place()
   start()

    

        




    
    

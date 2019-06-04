import RPi.GPIO as GPIO
import time
import sys
from math import sin,cos,pi,sqrt

Radius = 10 # need to change

PWMA = 18
AIN1 = 22
AIN2 = 27

PWMB = 23
BIN1 = 25
BIN2 = 24

PWMC = 16
CIN1 = 20
CIN2 = 21

GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BCM)

GPIO.setup(AIN2,GPIO.OUT)
GPIO.setup(AIN1,GPIO.OUT)
GPIO.setup(PWMA,GPIO.OUT)

GPIO.setup(BIN1,GPIO.OUT)
GPIO.setup(BIN2,GPIO.OUT)
GPIO.setup(PWMB,GPIO.OUT)

GPIO.setup(CIN1,GPIO.OUT)
GPIO.setup(CIN2,GPIO.OUT)
GPIO.setup(PWMC,GPIO.OUT)

A_Motor= GPIO.PWM(PWMA,100)
A_Motor.start(0)

B_Motor = GPIO.PWM(PWMB,100)
B_Motor.start(0)

C_Motor = GPIO.PWM(PWMC,100)
C_Motor.start(0)

def compute(speed,angle,omega):
    radian = angle * pi / 180
    
    Vx = speed * cos(radian) # x velocity
    Vy = speed * sin(radian) # y velocity
    
    a =  sqrt(3)/3 * Vx - 1/3 * Vy + omega * Radius # a velocity
    b = -sqrt(3)/3 * Vx - 1/3 * Vy + omega * Radius # b velocity
    c =                   2/3 * Vy + omega * Radius # c velocity
       
    return a,b,c

def move(va,vb,vc,t_time = 4):
    A_Motor.ChangeDutyCycle(abs(va))
    GPIO.output(AIN1,bool(va>0))
    GPIO.output(AIN2,bool(va<0))

    B_Motor.ChangeDutyCycle(abs(vb))
    GPIO.output(BIN1,bool(vb<0))
    GPIO.output(BIN2,bool(vb>0))
    
    C_Motor.ChangeDutyCycle(abs(vc))
    GPIO.output(CIN1,bool(vc>0))
    GPIO.output(CIN2,bool(vc<0))

    time.sleep(t_time)

if __name__ == "__main__":

    try:
        if len(sys.argv) != 4:
            print("Usage: python3 omni.py Speed(0:100) Angle Omega(0:10)")
            exit(1)

        speed = float(sys.argv[1])
        angle = float(sys.argv[2])
        omega = float(sys.argv[3])

        va,vb,vc = compute(speed,angle,omega)

        print("va: {} \nvb: {} \nvc: {}".format(va,vb,vc))

        move(va,vb,vc)
        
    except KeyboardInterrupt:
        GPIO.cleanup()
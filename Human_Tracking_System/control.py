# -*- coding: utf-8 -*-

"""
Ce fichier gère la patie commande moteur du rover. Une carte moteur type driver L298N permet 
de gerer les moteurs cc. 
"""

import RPi.GPIO as GPIO 
from time import sleep 

class Driver(object) : 
    def __init__(self, ena = 13, in1 = 5, in2 = 6, enb = 12, in3 = 23, in4 = 24) :     
        # setup GPIO pin mode 
        GPIO.setmode(GPIO.BCM)              # choose BCM numbering scheme 
        GPIO.setup(ena, GPIO.OUT)
        GPIO.setup(in1, GPIO.OUT)
        GPIO.setup(in2, GPIO.OUT)
        GPIO.setup(enb, GPIO.OUT)
        GPIO.setup(in3, GPIO.OUT)
        GPIO.setup(in4, GPIO.OUT)
        
        self.w1 = GPIO.PWM(ena, 100)      # angular speed motor A (100 Hertz)
        self.w2 = GPIO.PWM(ena, 100)      # angular speed motor B (100 Hertz)
        self.in1 = in1
        self.in2 = in2
        self.in3 = in3
        self.in4 = in4
        
        self.w1.start(0)
        self.w1.start(1)
    
    def stopMove(self) : 
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.LOW)  
        GPIO.output(self.in3,GPIO.LOW)
        GPIO.output(self.in4,GPIO.LOW) 
        
    def forward(self) : 
        GPIO.output(self.in1,GPIO.HIGH)
        GPIO.output(self.in2,GPIO.LOW)  
        GPIO.output(self.in3,GPIO.HIGH)
        GPIO.output(self.in4,GPIO.LOW) 
   
    def backward(self) : 
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.HIGH)  
        GPIO.output(self.in3,GPIO.LOW)
        GPIO.output(self.in4,GPIO.HIGH) 
        
    def speed(self, v1, v2) : 
        self.w1.ChangeDutyCycle(v1)
        self.w2.ChangeDutyCycle(v2)
        
        
if __name__ == '__main__':
    
    rover = Driver()
    rover.forward()
    sleep(5)
    rover.backward(5)
    sleep(5)
    rover.forward()
    rover.speed(25, 25)
    sleep(5)
    rover.stopMove()
    
    GPIO.cleanup()
    
    
    
            
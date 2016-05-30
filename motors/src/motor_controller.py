#!/usr/bin/python
import RPi.GPIO as GPIO

class MotorController:
  def __init__(self, pin_A, pin_B, pin_Enable):
    self.pin_A = pin_A
    self.pin_B = pin_B
    self.pin_Enable = pin_Enable

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    GPIO.setup(pin_A, GPIO.OUT)
    GPIO.setup(pin_B, GPIO.OUT)
    GPIO.setup(pin_Enable, GPIO.OUT)
    

  def RunForwards(self):
    GPIO.output(self.pin_A, GPIO.HIGH)
    GPIO.output(self.pin_B, GPIO.LOW)
    GPIO.output(self.pin_Enable, GPIO.HIGH)
    
  def RunBackwards(self):
    GPIO.output(self.pin_A, GPIO.LOW)
    GPIO.output(self.pin_B, GPIO.HIGH)
    GPIO.output(self.pin_Enable, GPIO.HIGH)

  def Stop(self):
    GPIO.output(self.pin_A, GPIO.LOW)
    GPIO.output(self.pin_B, GPIO.LOW)
    GPIO.output(self.pin_Enable, GPIO.LOW)
    

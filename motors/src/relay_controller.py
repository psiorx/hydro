#!/usr/bin/python
import RPi.GPIO as GPIO

class RelayController:
  def __init__(self, pin_A, pin_B):
    self.pin_A = pin_A
    self.pin_B = pin_B
    self.is_on = False
    
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    GPIO.setup(pin_A, GPIO.OUT)
    GPIO.setup(pin_B, GPIO.OUT)
  
  def TurnOn(self):
    GPIO.output(self.pin_B, GPIO.HIGH)
    GPIO.output(self.pin_A, GPIO.LOW)
    self.is_on = True
  
  def TurnOff(self):
    GPIO.output(self.pin_B, GPIO.LOW)
    GPIO.output(self.pin_A, GPIO.LOW)
    self.is_on = False

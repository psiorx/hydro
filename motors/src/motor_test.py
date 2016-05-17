#!/usr/bin/python

from motor_controller import MotorController
from time import sleep
if __name__ == '__main__':
  #chip1 left motor (13,15,11)  CONFIRMED
  #chip1 right motor (12, 16, 18) UNCONFIRMED
  #chip2 left motor (35,37,33)  CONFIRMED
  #chip2 right motor (36, 38, 40) CONFIRMED
  motors = [MotorController(13, 15, 11), MotorController(12, 16, 18), MotorController(35, 37, 33), MotorController(36, 38, 40)]
  #motor = motors[3] 
  for motor in motors:
    motor.RunForwards()
  sleep(5)
  for motor in motors:
    motor.RunBackwards()
  sleep(5)
  for motor in motors:
    motor.Stop()

#!/usr/bin/python
from motor_controller import MotorController
from time import sleep   

class FluidDispenser:
  def __init__(self):
    self.motors = dict()

  def AddMotor(self, motor, ml_per_second, motor_name):
    self.motors[motor_name] = (motor, ml_per_second)
    
  def DispenseFluid(self, motor_name, milliliters):
    if motor_name in self.motors:
      motor = self.motors[motor_name]
      controller = motor[0]
      ml_per_second = motor[1]
      dispense_time_seconds = milliliters/ml_per_second
      controller.RunForwards()
      sleep(dispense_time_seconds)
      controller.Stop()
      controller.RunBackwards()
      sleep(10)
      controller.Stop()

      
  

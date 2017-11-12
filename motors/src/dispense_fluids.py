#!/usr/bin/python

from motor_controller import MotorController
from fluid_dispenser import FluidDispenser
if __name__ == '__main__':
  dispenser = FluidDispenser()
  dispenser.AddMotor(MotorController(36, 38, 40), 0.8827, "pump1")
  dispenser.AddMotor(MotorController(35, 37, 33), 0.8156, "pump2")
  dispenser.AddMotor(MotorController(12, 16, 18), 0.7804, "pump3")
  dispenser.AddMotor(MotorController(13, 15, 11), 0.8585, "pump4")

  dispenser.DispenseFluid("pump1", 30)
  dispenser.DispenseFluid("pump2", 30)
  dispenser.DispenseFluid("pump3", 30)
  #dispenser.DispenseFluid("pump4", 20)

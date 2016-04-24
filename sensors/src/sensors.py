#!/usr/bin/python
import time
import serial
import rospy

class Sensors:
  def __init__(self, serial_device):
    self.ser = serial.Serial(serial_device, 9600)
    self.serial_delay = 2 
  
  def ReadSerial(self):
    out = ''
    while self.ser.inWaiting() > 0:
      out += self.ser.read(1)
    return out
  
  def HandleLine(self, line):
    fields = line.split(':')
    if len(fields) == 2:
      if fields[0] == 'PH':
        print "Got PH: " + fields[1]
      elif fields[0] == 'TEMP':
        print "Got Temp: " + fields[1]
      elif fields[0] == 'EC':
        print "Got EC: " + fields[1]
  
  def ReadSerialLines(self):
    self.ser.flushInput()
    self.ser.flushOutput()
    self.ser.flush()
    self.ReadSerial()
    while 1: 
      if self.ser.inWaiting() > 0:
        self.HandleLine(self.ser.readline().strip())
      else:
        time.sleep(0.5)

  def Cleanup(self):
    self.ser.close()
    
if __name__ == '__main__':
  sensors = Sensors('/dev/ttyACM0')
  #print 'TEMPERATURE:\n'
  #print sensors.ReadTemperature()
  #print 'CONDUCTIVITY:\n'
  #print sensors.ReadConductivity()
  #print 'PH:\n'
  #print sensors.ReadPH()
  sensors.ReadSerialLines()
  sensors.Cleanup()

#!/usr/bin/python
import serial
import rospy
from sensors.msg import Alkalinity
from sensors.msg import Temperature
from sensors.msg import Conductivity

class Sensors:
  def __init__(self, serial_device):
    self.ser = serial.Serial(serial_device, 9600)
    self.serial_delay = 0.5
    self.temp_pub = rospy.Publisher('/sensors/temperature', Temperature, queue_size=10)
    self.ph_pub = rospy.Publisher('/sensors/alkalinity', Alkalinity, queue_size=10)
    self.ec_pub = rospy.Publisher('/sensors/conductivity', Conductivity, queue_size=10) 
  
  def ReadSerial(self):
    out = ''
    while self.ser.inWaiting() > 0:
      out += self.ser.read(1)
    return out
  
  def HandlePH(self, value):
    ph_msg = Alkalinity()
    ph_msg.stamp = rospy.Time.now()
    ph_msg.pH = float(value)
    self.ph_pub.publish(ph_msg)
    rospy.loginfo("Published pH: %f", ph_msg.pH)

  def HandleEC(self, value):
    ec_msg = Conductivity()
    ec_msg.stamp = rospy.Time.now()
    fields = value.split(',')
    ec_msg.ec = float(fields[0])
    ec_msg.ppm = int(fields[1])
    self.ec_pub.publish(ec_msg)
    rospy.loginfo("Published ppm: %d", ec_msg.ppm)
 
  def HandleTemp(self, value):
    temp_msg = Temperature()
    temp_msg.stamp = rospy.Time.now()
    temp_msg.celsius = float(value)
    temp_msg.fahrenheit = (temp_msg.celsius * 1.8) + 32
    self.temp_pub.publish(temp_msg)
    rospy.loginfo("Published temp: %f F", temp_msg.fahrenheit)
 
  def HandleLine(self, line):
    fields = line.split(':')
    if len(fields) == 2:
      if fields[0] == 'PH':
        self.HandlePH(fields[1])
      elif fields[0] == 'TEMP':
        self.HandleTemp(fields[1])
      elif fields[0] == 'EC':
        self.HandleEC(fields[1])
      else:
        rospy.logwarn("Got unknown message type: %s", fields[0])
  
  def ReadSerialLines(self):
    self.ser.flushInput()
    self.ser.flushOutput()
    self.ser.flush()
    self.ReadSerial()
    while not rospy.is_shutdown(): 
      if self.ser.inWaiting() > 0:
        self.HandleLine(self.ser.readline().strip())
      else:
        rospy.sleep(self.serial_delay)

  def Cleanup(self):
    self.ser.close()
    
if __name__ == '__main__':
  rospy.init_node('sensors_node')  
  sensors = Sensors(rospy.get_param('serial_device', '/dev/ttyACM0'))
  try:
    sensors.ReadSerialLines()
  except rospy.ROSInterruptException:
    sensors.Cleanup()
    pass  

#!/usr/bin/python
import serial
import rospy
from sensors.msg import Alkalinity
from sensors.msg import Temperature
from sensors.msg import Conductivity
from sensors.msg import Humidity

class Sensors:
  def __init__(self, serial_device):
    self.ser = serial.Serial(serial_device, 9600)
    self.serial_delay = 0.5
    self.temp_pub = rospy.Publisher('/sensors/temperature', Temperature, queue_size=10)
    self.ph_pub = rospy.Publisher('/sensors/alkalinity', Alkalinity, queue_size=10)
    self.ec_pub = rospy.Publisher('/sensors/conductivity', Conductivity, queue_size=10) 
    self.hum_pub = rospy.Publisher('/sensors/humidity', Humidity, queue_size=10)
    self.air_pub = rospy.Publisher('/sensors/air_temperature', Temperature, queue_size=10)
    
  
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
    self.temp_pub.publish(temp_msg)
    rospy.loginfo("Published temp: %f C", temp_msg.celsius)
  
  def HandleAirTemp(self, value):
    temp_msg = Temperature()
    temp_msg.stamp = rospy.Time.now()
    temp_msg.celsius = float(value)
    self.air_pub.publish(temp_msg)
    rospy.loginfo("Published air temp: %f C", temp_msg.celsius)
  
  def HandleHumidity(self, value):
    hum_msg = Humidity()
    hum_msg.stamp = rospy.Time.now()
    hum_msg.relative_humidity = float(value)
    self.hum_pub.publish(hum_msg)
    rospy.loginfo("Published humidity: %f %%", hum_msg.relative_humidity)
 
 
  def HandleLine(self, line):
    fields = line.split(':')
    if len(fields) == 2:
      if fields[0] == 'PH':
        self.HandlePH(fields[1])
      elif fields[0] == 'TEMP':
        self.HandleTemp(fields[1])
      elif fields[0] == 'EC':
        self.HandleEC(fields[1])
      elif fields[0] == 'RELHUM':
        self.HandleHumidity(fields[1])
      elif fields[0] == 'AIRTEMP':
        self.HandleAirTemp(fields[1])
      else:
        rospy.logwarn("Got unknown message type: %s", fields[0])
  
  def ReadSerialLines(self):
    self.ser.flushInput()
    self.ser.flushOutput()
    self.ser.flush()
    self.ReadSerial()
    while not rospy.is_shutdown(): 
      if self.ser.inWaiting() > 0:
        serial_line = self.ser.readline().strip()
        try:
          self.HandleLine(serial_line)
        except:
          print "error while handling data: " + serial_line
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

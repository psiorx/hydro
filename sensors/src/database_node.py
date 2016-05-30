#!/usr/bin/python
import rospy
from influxdb import InfluxDBClient
from sensors.msg import Alkalinity
from sensors.msg import Temperature
from sensors.msg import Conductivity
from sensors.msg import Humidity

class DatabaseInserter:
  def __init__(self, database_ip, database_user, database_password, database_name):
    self.client = InfluxDBClient(database_ip, 8086, database_user, database_password, database_name)
    self.temp_sub = rospy.Subscriber("/sensors/temperature", Temperature, self.OnTemperature)
    self.ec_sub = rospy.Subscriber("/sensors/conductivity", Conductivity, self.OnConductivity)
    self.ph_sub = rospy.Subscriber("/sensors/alkalinity", Alkalinity, self.OnAlkalinity)
    self.airtemp_sub = rospy.Subscriber("/sensors/air_temperature", Temperature, self.OnAirTemperature)
    self.hum_sub = rospy.Subscriber("/sensors/humidity", Humidity, self.OnHumidity)
  
  def OnAirTemperature(self, msg):
    rospy.loginfo("Inserting Air Temperature Point: %f", msg.celsius)
    temperature_point = [{"measurement": "temperature","tags": {"sensor": "air"},"fields": {"value": msg.celsius}}]
    self.client.write_points(temperature_point)
    
  def OnTemperature(self, msg):
    rospy.loginfo("Inserting Temperature Point: %f", msg.celsius)
    temperature_point = [{"measurement": "temperature","tags": {"sensor": "water"},"fields": {"value": msg.celsius}}]
    self.client.write_points(temperature_point)

  def OnConductivity(self, msg):
    rospy.loginfo("Inserting Conductivity Point: %f, %d", msg.ec, msg.ppm)
    conductivity_point = [{"measurement": "conductivity","fields": {"ec": msg.ec, "ppm": msg.ppm}}]
    self.client.write_points(conductivity_point)
  
  def OnAlkalinity(self, msg):
    rospy.loginfo("Inserting Alkalinity Point: %f", msg.pH)
    alkalinity_point = [{"measurement": "alkalinity","fields": {"pH": msg.pH}}]
    self.client.write_points(alkalinity_point)
  
  def OnHumidity(self, msg):
    rospy.loginfo("Inserting Humidity Point: %f", msg.relative_humidity)
    alkalinity_point = [{"measurement": "humidity","fields": {"relative": msg.relative_humidity}}]
    self.client.write_points(alkalinity_point)


if __name__ == '__main__':
  rospy.init_node('database_node')
  database_ip = rospy.get_param('database_ip')
  database_user = rospy.get_param('database_user')
  database_password = rospy.get_param('database_password')
  database_name = rospy.get_param('database_name')
   
  inserter = DatabaseInserter(database_ip, database_user, database_password, database_name)

  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass  

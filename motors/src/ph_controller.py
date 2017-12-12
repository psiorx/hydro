#!/usr/bin/python
import rospy
from motor_controller import MotorController
from fluid_dispenser import FluidDispenser
from influxdb import InfluxDBClient
from sensors.msg import Alkalinity

class PHController:
  
  def __init__(self, database_ip, database_user, database_password, database_name):
    self.ph_measurement = []
    self.buffer_length = rospy.get_param('buffer_length') 
    self.ph_limit_low = rospy.get_param('ph_limit_low')   
    self.ph_limit_high = rospy.get_param('ph_limit_high')   
    self.cooldown_minutes = rospy.get_param('cooldown_minutes')
    self.client = InfluxDBClient(database_ip, 8086, database_user, database_password, database_name)
    self.ph_sub = rospy.Subscriber("/sensors/alkalinity", Alkalinity, self.OnAlkalinity)
    self.dispenser = FluidDispenser()
    self.dispenser.AddMotor(MotorController(36, 38, 40), 0.8827, "pump1") # PH DOWN PUMP
    self.dispenser.AddMotor(MotorController(35, 37, 33), 0.8156, "pump2") # PH UP PUMP
    self.last_action_time = 0
  
  def OnAlkalinity(self, msg):
    rospy.loginfo("Got Ph measurement: %f", msg.pH)
    #alkalinity_point = [{"measurement": "alkalinity","fields": {"pH": msg.pH}}]
    #self.client.write_points(alkalinity_point)
    num_measurements = len(self.ph_measurement)
    if num_measurements == self.buffer_length: #our buffer is full
	self.ph_measurement.pop()
        self.ph_measurement.insert(0,float(msg.pH))
        avePH = sum(self.ph_measurement)/float(len(self.ph_measurement))
        rospy.loginfo("ave: %f",avePH)
        time_now = rospy.Time.now().to_sec()
        if avePH < self.ph_limit_low:
          if (time_now - self.last_action_time) > 60 * self.cooldown_minutes:
            error = abs(avePH - self.ph_limit_low)
            rospy.loginfo("Activating ph UP: %f", error)
            self.last_action_time = time_now
            self.dispenser.DispenseFluid("pump2", 15 + error) 
        if avePH > self.ph_limit_high:
          if (time_now - self.last_action_time) > 60 * self.cooldown_minutes:
            error = abs(avePH - self.ph_limit_high)
            rospy.loginfo("Activating ph DOWN: %f", error)          
            self.last_action_time = time_now
            self.dispenser.DispenseFluid("pump1", 15 + error) 
        
    else:
        self.ph_measurement.insert(0,float(msg.pH))
        rospy.loginfo("gathering measurements: %i remaining", self.buffer_length - num_measurements)
  
if __name__ == '__main__':
  rospy.init_node('ph_controller')
  database_ip = rospy.get_param('database_ip')
  database_user = rospy.get_param('database_user')
  database_password = rospy.get_param('database_password')
  database_name = rospy.get_param('database_name')
  inserter = PHController(database_ip, database_user, database_password, database_name)
  

  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass  

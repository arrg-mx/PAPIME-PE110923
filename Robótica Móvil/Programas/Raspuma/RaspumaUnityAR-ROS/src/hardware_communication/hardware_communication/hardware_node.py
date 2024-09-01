import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist

import RPi.GPIO as GPIO
from gpiozero import AngularServo
import time
from gpiozero.pins.native import NativeFactory
from gpiozero import Device, LED
"""
pip3 install RPi.GPIO
pip3 install gpiozero
cd /dev
sudo chmod og+rwx /dev/gpio*
"""
#Locomoción
a1 = 3
a2 = 5
apwm = 50
b1 = 8
b2 = 10
bpwm = 50
#Servos
s1 = 21 #pitch
s2 = 19 #yaw
#Ultrasónicos
uE = [36, 40, 38] #i, d, f
uT = [31, 33, 29] #i, d, f
def clamp(n, min, max): 
  if n < min: 
    return min
  elif n > max: 
    return max
  else: 
    return n 

class HardwareNode(Node):
  #Definir gpio
  #Locomoción
  GPIO.setwarnings(False)
  GPIO.setmode(GPIO.BOARD)
  GPIO.setup(a1, GPIO.OUT)
  GPIO.setup(a2, GPIO.OUT)
  ma1 = GPIO.PWM(a1, 50)
  ma2 = GPIO.PWM(a2, 50)
  GPIO.setup(b1, GPIO.OUT)
  GPIO.setup(b2, GPIO.OUT)
  mb1 = GPIO.PWM(b1, 50)
  mb2 = GPIO.PWM(b2, 50)
  #Servos
  #yaw
  GPIO.setup(s1, GPIO.OUT)
  GPIO.setup(s2, GPIO.OUT)
  #Ultrasóicos
  GPIO.setup(uE[0], GPIO.IN)
  GPIO.setup(uT[0], GPIO.OUT)
  GPIO.setup(uE[1], GPIO.IN)
  GPIO.setup(uT[1], GPIO.OUT)
  GPIO.setup(uE[2], GPIO.IN)
  GPIO.setup(uT[2], GPIO.OUT)
  #Distancias
  uDis = [0,0,0]
  
  def __init__(self):
    Device.pin_factory = NativeFactory()
    #ROS
    super().__init__('hardware_node')
    self.robotTwistSubscriber = self.create_subscription(Twist, '/robot_twist', self.RobotTwistCallback, 1)
    self.cameraTwistSubscriber = self.create_subscription(Twist, '/robot_camera_twist', self.RobotCameraTwistCallback, 1)
    self.ultrasonicPublisher = self.create_publisher(Float32MultiArray,'/robot_us',1)
    self.robotTwistSubscriber
    self.cameraTwistSubscriber  # prevent unused variable warning
    self.ultrasonicTimer = self.create_timer(0.1, self.UltrasonicCallback)
    
  def UltrasonicCallback(self):
    self.UltrasonicDistance()
    msg = Float32MultiArray()
    msg.data = [float(self.uDis[0]),float(self.uDis[1]),float(self.uDis[2])]
    self.ultrasonicPublisher.publish(msg)

  def RobotTwistCallback(self, msg):
    vala = int(( float(msg._linear.y)+float(msg._linear.x))*apwm)
    valb = int((-float(msg._linear.y)+float(msg._linear.x))*bpwm)
    vala = clamp(vala, -100, 100)
    valb = clamp(valb, -100, 100)
    if vala > 0:
      self.ma1.start(vala)
      self.ma2.stop()
    else:
      self.ma1.stop()
      self.ma2.start(-vala)
    if valb > 0:
      self.mb1.start(valb)
      self.mb2.stop()
    else:
      self.mb1.stop()
      self.mb2.start(-valb)
  
  def RobotCameraTwistCallback(self, msg):
    #Pitch
    pitchTime = ((msg._angular._y * 180.0) * (1.0/180.0) + 1)/1000.0
    yawTime = ((msg._angular._z * 180.0) * (1.0/180.0) + 1)/1000.0
    for i in range (2):
      GPIO.output(s1, GPIO.HIGH)
      time.sleep(pitchTime)
      GPIO.output(s1, GPIO.LOW)
      time.sleep((0.02 - pitchTime))
    for i in range (2):
      GPIO.output(s2, GPIO.HIGH)
      time.sleep(yawTime)
      GPIO.output(s2, GPIO.LOW)
      time.sleep((0.02 - yawTime))

  def UltrasonicDistance(self):
    for i in range (3):
      # set Trigger to HIGH
      GPIO.output(uT[i], True)
      # set Trigger after 0.01ms to LOW
      time.sleep(0.00001)
      GPIO.output(uT[i], False)
      StartTime = time.time()
      StopTime = time.time()
      # save StartTime
      while GPIO.input(uE[i]) == 0:
        StartTime = time.time()
      # save time of arrival
      while GPIO.input(uE[i]) == 1:
        StopTime = time.time()
      # time difference between start and arrival
      TimeElapsed = StopTime - StartTime
      self.uDis[i] = (TimeElapsed * 34300) / 2

def main(args=None):
  rclpy.init(args=args)
  hardware_node = HardwareNode()
  rclpy.spin(hardware_node)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  hardware_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()


import rclpy
import math

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, ReliabilityPolicy

class Fuzzy_system:

  def __init__(self,front1,front2,LMS,RMS):
    # Initialize the Fuzzy_system object with front 1 and front 2 sensors values,
    #  as well as LMS (Left Motor Speed) and RMS (Right Motor Speed) values.
    self.front1 = front1
    self.front2 = front2
    self.LMS = LMS
    self.RMS = RMS
    self.Rules = self.Rules_table_generator()


  def Rule_Values(self,output_1,output_2):
    #Determine the minimum value between the front and back sensors' memberhsip values to determine the rule values.
    rules_values = []
    for front_sensor_state in output_1:
      for back_sensor_state in output_2:
        rule_value=min(output_1[front_sensor_state],output_2[back_sensor_state])
        rules_values.append(rule_value)
    return rules_values


  def Rules_table_generator(self):
    #Generate the rules tables by combining both sensors ranges with corresponding LMS and RMS values
    rules=[]
    LMS_rules = ["slow","slow","slow","slow","medium","medium","slow","medium","fast"]
    RMS_rules =["left","left","left","left","front","front","left","front","front"]
    i=0
    for R1 in ["close","medium","far"]:
      for R2 in ["close","medium","far"]:
        rules.append([R1,R2,LMS_rules[i],RMS_rules[i]])
        i+=1
    return rules


  def fuzzification(self,x,y):
    # fuzzify the input by calculating the membership values of the input values (x and y)
    #also determine the crips output using centroid method 
    output_1=self.calculation_membership(x,self.front1)
    output_2=self.calculation_membership(y,self.front2)
    #self.output = {"front_senor":output_1,"back_sensor":output_2}
    reading_rules = self.Rule_Values(output_1,output_2)
    #complete_rules = self.output_states(reading_rules)
    self.final_output = self.centriod(reading_rules)
    #print(self.final_output)
    return self.final_output

  def interpolation(self,p1,p2,x):
    #calculate the y-value for a given x-value
    (x1,y1) = p1
    (x2,y2) = p2
    return ((y2-y1)*x + x2 * y1 -x1* y2)/(x2-x1)

  def function_line_result(self,function_structure,x):
    #calculate the y-value for line function where start and end points are given
    start_point = function_structure[0]
    end_point = function_structure[1]

    if x <= start_point[0]:
      result = start_point[1]
    elif  start_point[0] < x <= end_point[0]:
      result = self.interpolation(start_point,end_point,x)
    else:
      result = end_point[1]

    return result

  def triangle(self,function_structure,x):
    #calculate the result of a triangular membership where the (start-mid-end) points are given
    start_point = function_structure[0]
    end_point = function_structure[2]
    mid_point = function_structure[1]

    if x <= mid_point[0]:
      result = self.function_line_result([start_point,mid_point],x)
    else:
      result = self.function_line_result([mid_point,end_point],x)

    return result


  def calculation_membership(self,x,sensor):
    #calculate the membership value for a given input
    output = {}
    close_function = sensor[0]
    medimum_function = sensor[1]
    far_function = sensor[2]

    output["close"] = self.function_line_result(close_function,x)
    output["medium"] = self.triangle(medimum_function,x)
    output["far"] = self.function_line_result(far_function,x)


    return output

  def active_rules(self,reading_rules):
    #Removes the rules that are not active(have zero-values)
    Active_rules=[]
    for i in range(len(reading_rules)) :
      value = reading_rules[i]
      if value != 0:
        Active_rules.append([value,self.Rules[i][-2],self.Rules[i][-1]])
    return Active_rules

  def centriod(self,reading_rules):
    # Calculate the centroid of the active rules and return zeros in case no active rules
    if len(reading_rules) == 0:
      return (0,0)

    LMS_summation = 0
    RMS_summation = 0
    denominator = 0

    active_rules = self.active_rules(reading_rules)
    for rule in active_rules:
      value = rule[0]
      denominator+=value
      LMS_centre = self.LMS[rule[1]]
      RMS_centre = self.RMS[rule[2]]

      LMS_rule_value = LMS_centre * value
      RMS_rule_value = RMS_centre * value

      LMS_summation+=LMS_rule_value
      RMS_summation+=RMS_rule_value

    LMS_result = (LMS_summation)/denominator
    RMS_result = RMS_summation/denominator

    return (LMS_result,RMS_result)


#fuzzy system assignment

#first we determine the membership function for the first two sensor
front1 = [[(0.60,1),(0.65,0)],[(0.60,0),(0.65,1),(0.7,0)],[(0.65,0),(0.7,1)]]
front2 = [[(0.60,1),(0.65,0)],[(0.60,0),(0.65,1),(0.7,0)],[(0.65,0),(0.7,1)]]

LMS = {"slow":0.05,"medium":0.1,"fast":0.15}
RMS = {"right":-0.2,"front":0.0,"left":0.2}
Fz=Fuzzy_system(front1,front2,LMS,RMS)


mynode_ = None
pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front1': 0,
    'front2': 0,
    'fleft': 0,
    'left': 0,
}
twstmsg_ = None

# main function attached to timer callback
def timer_callback():
    global pub_, twstmsg_
    if ( twstmsg_ != None ):
        pub_.publish(twstmsg_)


def clbk_laser(msg):
    global regions_, twstmsg_

    regions_ = {
        #LIDAR readings are anti-clockwise
        'front1':  find_nearest (msg.ranges[0:50]),
        'front2':  find_nearest (msg.ranges[310:360]),
        'right':  find_nearest(msg.ranges[265:275]),
        'fright': find_nearest (msg.ranges[310:320]),
        'fleft':  find_nearest (msg.ranges[40:50]),
        'left':   find_nearest (msg.ranges[85:95]),
        'fback': find_nearest (msg.ranges[220:230])

    }
    twstmsg_= movement()


# Find nearest point
def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
    return min(min(f_list, default=10), 10)

#Basic movement method
def movement():
    #print("here")
    global regions_, mynode_
    regions = regions_

    print("Min distance in front region: ", regions_['front1'],regions_['front2'])

    #create an object of twist class, used to express the linear and angular velocity of the turtlebot
    msg = Twist()
    msg.linear.x,msg.angular.z = Fz.fuzzification(regions_['front1'],regions_['front2'])
    return msg

    '''
    #If an obstacle is found to be within 0.25 of the LiDAR sensors front region the linear velocity is set to 0 (turtlebot stops)
    if (regions_['front1'])< 0.25:
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        return msg
    #if there is no obstacle in front of the robot, it continues to move forward
    elif (regions_['front2'])< 0.25:
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        return msg
    else:
        msg.linear.x = 0.1
        msg.angular.z = 0.0
        return msg
'''
#used to stop the rosbot
def stop():
    global pub_
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    pub_.publish(msg)


def main():
    global pub_, mynode_

    rclpy.init()
    mynode_ = rclpy.create_node('reading_laser')

    # define qos profile (the subscriber default 'reliability' is not compatible with robot publisher 'best effort')
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )

    # publisher for twist velocity messages (default qos depth 10)
    pub_ = mynode_.create_publisher(Twist, '/cmd_vel', 10)

    # subscribe to laser topic (with our qos)
    sub = mynode_.create_subscription(LaserScan, '/scan', clbk_laser, qos)

    # Configure timer
    timer_period = 0.2  # seconds
    timer = mynode_.create_timer(timer_period, timer_callback)

    # Run and handle keyboard interrupt (ctrl-c)
    try:
        rclpy.spin(mynode_)
    except KeyboardInterrupt:
        stop()  # stop the robot
    except:
        stop()  # stop the robot
    finally:
        # Clean up
        mynode_.destroy_timer(timer)
        mynode_.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

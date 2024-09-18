
import rclpy
import math

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, ReliabilityPolicy

class Fuzzy_system :

  def __init__(self,front_sensor,back_sensor,LMS,RMS):
    # Initialize the Fuzzy_system object with front and back sensor values,
    #  as well as LMS (Left Motor Speed) and RMS (Right Motor Speed) values.
    self.front_sensor = front_sensor
    self.back_sensor = back_sensor
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
    LMS_rules = ["slow","slow","slow","medium","medium","medium","fast","fast","fast"]
    RMS_rules =["left","left","left","front","front","front","right","right","right"]
    i=0
    for R1 in ["close","medium","far"]:
      for R2 in ["close","medium","far"]:
        rules.append([R1,R2,LMS_rules[i],RMS_rules[i]])
        i+=1
    return rules


  def fuzzification(self,x,y):
    # fuzzify the input by calculating the membership values of the input values (x and y)
    #also determine the crips output using centroid method 
    output_1=self.calculation_membership(x,self.front_sensor)
    output_2=self.calculation_membership(y,self.back_sensor)
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
     # Sum the weighted LMS and RMS values
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

class Fuzzy_system_OV:

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
    LMS_rules = ["slow","slow","slow","slow","medium","medium","slow","medium","slow"]
    RMS_rules =["left","left","left","left","front","front","left","front","right"]
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



class cotext_layer:
  def __init__(self, membership_1,membership_2):
    # Initialize the membership values and LMS and RMS values
    self.D1_membership = membership_1
    self.D2_membership = membership_2
    self.RMS = 0.0
    self.LMS = 0.0

  def fuzzification_combination(self,Front_right,Back_right,front1,front2,RD,OV):
    # Fuzzify the inputs 
    d1 = min(Front_right,Back_right)
    d2 = min(front1,front2)

    #calculate the membership value for both d1 and d2
    membership_d1 = self.calculate_membership(d1,self.D1_membership)
    membership_d2 = self.calculate_membership(d2,self.D2_membership)
    print('membership d1, membership d2:', membership_d1, membership_d2)

    #get the the speed from the RE following system and OV system
    LMS_RD , RMS_RD = RD
    LMS_OV , RMS_OV = OV
    #if both membership are zero assign value so denominator cant be zero 
    if (membership_d1 == 0) and (membership_d2 == 0) :
      self.RMS = -0.1
      self.LMS = 0.1
    else:
      # Calculate the weighted average of LMS and RMS
      self.RMS = ((membership_d1*RMS_RD)+(membership_d2*RMS_OV))/(membership_d1+membership_d2)
      self.LMS =((membership_d1*LMS_RD)+(membership_d2*LMS_OV))/(membership_d1+membership_d2)

    return (self.LMS,self.RMS)

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

  def calculate_membership(self,x,y):
    #calculate the membership value for a given input
    start_point = y[0]
    end_point = y[1]

    if x <= start_point[0]:
      result = start_point[1]
    elif  start_point[0] < x <= end_point[0]:
      result = self.interpolation(start_point,end_point,x)
    else:
      result = end_point[1]

    return result

#fuzzy system assignment
D1_membership = [(0.4,1),(0.5,0)]
D2_membership =  [(1.0,1),(1.3,0)] #[(1.3,1),(1.5,0)]
#first we determine the membership function for the first two sensor
front_sensor = [[(0.70,1),(0.75,0)],[(0.70,0),(0.75,1),(0.80,0)],[(0.75,0),(0.80,1)]]
back_sensor= [[(0.70,1),(0.75,0)],[(0.70,0),(0.75,1),(0.80,0)],[(0.75,0),(0.80,1)]]

front1 = [[(0.85,1),(0.90,0)],[(0.85,0),(0.90,1),(0.95,0)],[(0.90,0),(0.95,1)]]
front2 = [[(0.85,1),(0.90,0)],[(0.85,0),(0.90,1),(0.95,0)],[(0.90,0),(0.95,1)]]

LMS = {"slow":0.05,"medium":0.1,"fast":0.15}
RMS = {"right":-0.10,"front":0.0,"left":0.35}

LMS_O= {"slow":0.05,"medium":0.1,"fast":0.15}
RMS_O= {"right":-0.10,"front":0.0,"left":0.35}

R_Fz=Fuzzy_system(front_sensor,back_sensor,LMS,RMS)
Ov_Fz = Fuzzy_system_OV(front1,front2,LMS,RMS_O)
contex =cotext_layer(D1_membership,D2_membership)

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
        'front1':  find_nearest (msg.ranges[0:15]),
        'front2':  find_nearest (msg.ranges[345:360]),
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

    print(regions_['fright'],regions_['fback'],regions_['front1'],regions_['front2'])

    #create an object of twist class, used to express the linear and angular velocity of the turtlebot
    msg = Twist()

    R_speed = R_Fz.fuzzification(regions_['fright'],regions_['fback'])
    O_speed = Ov_Fz.fuzzification(regions_['front1'],regions_['front2'])
    msg.linear.x,msg.angular.z = contex.fuzzification_combination(regions_['fright'],regions_['fback'],regions_['front1'],regions_['front2'],R_speed,O_speed)
    print('R_speed',R_speed,'O_speed',O_speed)
    #msg.linear.x,msg.angular.z = work.fuzzification_combination(10,10,10,10,R_speed,O_speed)

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

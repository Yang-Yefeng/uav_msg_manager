#! /usr/bin/python3
import rospy
from mavros_msgs.msg import State
# from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32MultiArray, Bool, String, Float32
# import tf
# from tf.transformations import quaternion_matrix
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from uav_msg_manager.msg import uav_msg


class uav_base:
    def __init__(self, uav_name: str=''):
        self.uav_name = uav_name
        # self.pos = [0, 0, 0]
        # self.vel = [0, 0, 0]
        # self.att = [0, 0, 0]
        # self.pqr = [0, 0, 0]
        # self.connected = False
        # self.armed = False
        # self.mode = 'off'
        self.uav_msg = uav_msg()
        self.uav_msg.name.data = uav_name
        self.uav_msg.pos = Float32MultiArray()
        self.uav_msg.vel = Float32MultiArray()
        self.uav_msg.att = Float32MultiArray()
        self.uav_msg.pqr = Float32MultiArray()
        
        # self.current_state = State()  # monitor uav status
        # self.pose = PoseStamped()  # publish offboard [x_d y_d z_d] cmd
        self.uav_odom = Odometry()  # subscribe uav state x y z vx vy vz phi theta psi p q r
        # self.voltage = 11.4
        # self.global_flag = 0  # UAV working mode monitoring
        
        '''topic subscribe'''
        self.state_sub = rospy.Subscriber(self.uav_name + "/mavros/state", State, callback=self.state_cb)
        self.uav_odom_sub = rospy.Subscriber(self.uav_name + "/mavros/local_position/odom", Odometry, callback=self.uav_odom_cb)
        self.uav_battery_sub = rospy.Subscriber(self.uav_name + "/mavros/battery", BatteryState, callback=self.uav_battery_cb)
        '''topic subscribe'''
        
        '''topic publish'''
        # self.pos_pub = rospy.Publisher(self.uav_name + "/uav_msg/pos", Float32MultiArray, queue_size=10)
        # self.vel_pub = rospy.Publisher(self.uav_name + "/uav_msg/vel", Float32MultiArray, queue_size=10)
        # self.att_pub = rospy.Publisher(self.uav_name + "/uav_msg/att", Float32MultiArray, queue_size=10)
        # self.pqr_pub = rospy.Publisher(self.uav_name + "/uav_msg/pqr", Float32MultiArray, queue_size=10)
        # self.connect_pub = rospy.Publisher(self.uav_name + "/uav_msg/connected", Bool, queue_size=10)
        # self.armed_pub = rospy.Publisher(self.uav_name + "/uav_msg/armed", Bool, queue_size=10)
        # self.mode_pub = rospy.Publisher(self.uav_name + "/uav_msg/mode", String, queue_size=10)
        # self.voltage_pub = rospy.Publisher(self.uav_name + "/uav_msg/voltage", Float32, queue_size=10)
        self.uav_msg_pub = rospy.Publisher(self.uav_name + "/uav_msg", uav_msg, queue_size=10)
        '''topic publish'''
        
        # '''arming service'''
        # rospy.wait_for_service(self.uav_name + "/mavros/cmd/arming")  # 等待解锁电机的 service 建立
        # self.arming_client = rospy.ServiceProxy(self.uav_name + "/mavros/cmd/arming", CommandBool)
        #
        # '''working mode service'''
        # rospy.wait_for_service(self.uav_name + "/mavros/set_mode")  # 等待设置 UAV 工作模式的 service 建立
        # self.set_mode_client = rospy.ServiceProxy(self.uav_name + "/mavros/set_mode", SetMode)
        
        self.rate = rospy.Rate(500)
        # self.offb_set_mode = SetModeRequest()  # 先设置工作模式为 offboard
        # self.arm_cmd = CommandBoolRequest()
    
    def state_cb(self, msg: State):
        self.uav_msg.connected.data = msg.connected
        self.uav_msg.armed.data = msg.armed
        self.uav_msg.mode.data = msg.mode
    
    def uav_battery_cb(self, msg: BatteryState):
        self.uav_msg.voltage.data = msg.voltage
    
    def uav_odom_cb(self, msg: Odometry):
        self.uav_odom.pose.pose.position.x = msg.pose.pose.position.x
        self.uav_odom.pose.pose.position.y = msg.pose.pose.position.y
        self.uav_odom.pose.pose.position.z = msg.pose.pose.position.z
        self.uav_odom.pose.pose.orientation.x = msg.pose.pose.orientation.x
        self.uav_odom.pose.pose.orientation.y = msg.pose.pose.orientation.y
        self.uav_odom.pose.pose.orientation.z = msg.pose.pose.orientation.z
        self.uav_odom.pose.pose.orientation.w = msg.pose.pose.orientation.w

        self.uav_odom.twist.twist.linear.x = msg.twist.twist.linear.x
        self.uav_odom.twist.twist.linear.y = msg.twist.twist.linear.y
        self.uav_odom.twist.twist.linear.z = msg.twist.twist.linear.z
        self.uav_odom.twist.twist.angular.x = msg.twist.twist.angular.x
        self.uav_odom.twist.twist.angular.y = msg.twist.twist.angular.y
        self.uav_odom.twist.twist.angular.z = msg.twist.twist.angular.z

        self.uav_msg.pos = Float32MultiArray(data=[msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        self.uav_msg.vel = Float32MultiArray(data=[msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.uav_msg.att = Float32MultiArray(data=[roll, pitch, yaw])
        self.uav_msg.pqr = Float32MultiArray(data=[msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])

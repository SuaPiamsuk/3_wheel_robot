#!/usr/bin/python3

import rclpy
import serial

from rclpy.node import Node

from std_msgs.msg import Float32
from rclpy import qos


from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


# Global Defines --------------------------------------------------------------
DRIVE_INTERFACE_STS_INIT = 0
DRIVE_INTERFACE_STS_RUN = 1

# Class -----------------------------------------------------------------------
class ResqDriveInterface(Node):
    # This class represents the drive interface node.
    # It is a bridge between the ros network and the arduino drive controller

    def __init__(self):
        super().__init__('drive_interface') # Node name: drive_interface
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.publisher_imu = self.create_publisher(Imu, 'imu', 10)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self. L_speed = 0.0
        self. R_speed = 0.0

        self.rx_msg = ""

        # Status see timer callback for description
        self.__state = DRIVE_INTERFACE_STS_INIT
        self.__cmd_speed_l = 0.0
       
        self.__cmd_speed_r = 0.0
       
        # Init class -> read parameters, create subscribers, create timer (for update loop)
        self.__readParams()
        self.__createSubscribers()
        self.__createTimer()

    def __leftWheelCallback(self, msg):
        # Called when a message is received on the left wheel topic
        self.get_logger().info('Left wheel speed: %f' % msg.data)

        self.__cmd_speed_l = msg.data
        

    def __rightWheelCallback(self, msg):
        # Called when a message is received on the right wheel topic
        self.get_logger().info('Right wheel speed: %f' % msg.data)

        self.__cmd_speed_r = msg.data
        
    
    def __timerCallback(self):
        # Called when the timer is triggered with the update rate specified in the parameters
        self.get_logger().info('Timer callback')

        # Statemachine to handle connect and reconnect to arduino uno
        # if it is unplugged while node is running

        # Init state
        # Try to connect to arduino uno, otherwise stay in init state
        if self.__state == DRIVE_INTERFACE_STS_INIT:
            try:
                # Open serial connection
                self.__serial = serial.Serial(self._serial_name.value, self._serial_baudrate.value, timeout=self._serial_timeout.value)
                
                # Flush old data from buffers
                self.__serial.flush()

                # Transition to run state
                self.__state = DRIVE_INTERFACE_STS_RUN

                # Log success
                self.get_logger().info('Connected to arduino uno at %s' % self._serial_name.value)
            except:
                # Stay in init state
                self.__state = DRIVE_INTERFACE_STS_INIT

                # Log error
                self.get_logger().error('Could not connect to arduino uno at %s' % self._serial_name.value)
                

        # Run state
        # Transmit data to arduino uno
        elif self.__state == DRIVE_INTERFACE_STS_RUN:
           
            # cmd_speed_l = self.__cmd_speed_l
            # cmd_speed_r = self.__cmd_speed_r

            cmd_speed_l = self.L_speed
            cmd_speed_r = self.R_speed

            # Create tx message
            tx_msg = format("L{}R{}\n".format(int(cmd_speed_l), int(cmd_speed_r)));

            # Log message / uncomment for debugging
            self.get_logger().info('TX: %s' % tx_msg)

            # Send message
            try:
                self.__serial.write(tx_msg.encode('utf-8'))
                
            except:
                # Transition to init state
                self.__state = DRIVE_INTERFACE_STS_INIT
                self.__serial.close()

                # Log error
                self.get_logger().error('Could not send data to arduino uno -> Transition to init state')

            # Read message
            try:
                self.__serial.reset_input_buffer()
                self.rx_msg = self.__serial.readline().decode('utf-8').rstrip()

                # Log message / uncomment for debugging
                self.get_logger().info('RX: %s' % self.rx_msg)

            except:
                # Transition to init state
                self.__state = DRIVE_INTERFACE_STS_INIT
                self.__serial.close()

                # Log error
                self.get_logger().error('Could not read data from arduino uno -> Transition to init state')

        
        msg = String()
        msg.data = self.rx_msg
        self.publisher_.publish(msg)

        imu = self.rx_msg[1:-1]
        print(imu.split(',')) #ax,ay,az,gx,gy,az
        imu = imu.split(',')
    
        try:
            imu_msg = Imu()
            imu_msg.linear_acceleration.x = float(imu[0])
            imu_msg.linear_acceleration.y = float(imu[1])
            imu_msg.linear_acceleration.z = float(imu[2])
            imu_msg.angular_velocity.x = float(imu[3])
            imu_msg.angular_velocity.y = float(imu[4])
            imu_msg.angular_velocity.z = float(imu[5])
            imu_msg.orientation.x = 0.0 #find roll pitch yaw before convert to quaternion
            imu_msg.orientation.y = 0.0 #find roll pitch yaw before convert to quaternion
            imu_msg.orientation.z = 0.0 #find roll pitch yaw before convert to quaternion
            imu_msg.orientation.w = 0.0 #find roll pitch yaw before convert to quaternion
            self.publisher_imu.publish(imu_msg)
        except:
            pass

        
        

    def __readParams(self):
        # Declare parameters
        self.declare_parameter('update_rate_hz', 80.0)
        self.declare_parameter('serial_timeout_sec', 0.0125)
        self.declare_parameter('serial_name', '/dev/ttyACM0')
        self.declare_parameter('serial_baudrate', 115200)

        # Read parameters

        self._update_rate_hz = rclpy.parameter.Parameter(
            'update_rate_hz',
            rclpy.Parameter.Type.DOUBLE,
            80.0
        )

        self._serial_timeout = rclpy.parameter.Parameter(
            'serial_timeout_sec',
            rclpy.Parameter.Type.DOUBLE,
            0.0125
        )

        self._serial_name = rclpy.parameter.Parameter(
            'serial_name',
            rclpy.Parameter.Type.STRING,
            '/dev/ttyACM0'
        )

        self._serial_baudrate = rclpy.parameter.Parameter(
            'serial_baudrate',
            rclpy.Parameter.Type.INTEGER,
            115200
        )
        
        # Check for valid settings
        if (1.0 / self._update_rate_hz.value) < self._serial_timeout.value:
            self.get_logger().error('Serial timeout cannot be greater than the update rate of the task!')
            raise Exception('Serial timeout cannot be greater than the update rate of the task!')

    def __createSubscribers(self):
        # Create subscribers

        self._sub_left = self.create_subscription(
            Float32,
            'cmd/speed/left',
            self.__leftWheelCallback,
            qos_profile=qos.qos_profile_system_default,
        )

        self._sub_right = self.create_subscription(
            Float32,
            'cmd/speed/right',
            self.__rightWheelCallback,
            qos_profile=qos.qos_profile_system_default,
        )
    
    def listener_callback(self, msg):

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        radius_robot =  0.08 # 8cm

        self.R_speed = linear_x + angular_z*radius_robot
        self.L_speed = linear_x - angular_z*radius_robot

        self.R_speed = self.R_speed * 400
        self.L_speed = self.L_speed * 400

        print(self.R_speed)



    def __createTimer(self):
        # Create timer

        self._timer = self.create_timer(
            1.0 / self._update_rate_hz.value,
            self.__timerCallback
        )

# Main ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)

    drive_interface = ResqDriveInterface()

    rclpy.spin(drive_interface)

    drive_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
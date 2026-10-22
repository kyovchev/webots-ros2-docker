import rclpy
from std_msgs.msg import Float32, Float32MultiArray


class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.joint0_motor = self.__robot.getDevice('joint0')
        self.joint1_motor = self.__robot.getDevice('joint1')
        self.joint2_motor = self.__robot.getDevice('joint2')
        self.joint3_motor = self.__robot.getDevice('joint3')
        self.joint4_motor = self.__robot.getDevice('joint4')
        self.joint5_motor = self.__robot.getDevice('joint5')

        self.joint0_motor.setPosition(0)
        self.joint1_motor.setPosition(0)
        self.joint2_motor.setPosition(0)
        self.joint3_motor.setPosition(0)
        self.joint4_motor.setPosition(0)
        self.joint5_motor.setPosition(0)

        timestep = int(self.__robot.getBasicTimeStep())

        self.joint0_sensor = self.__robot.getDevice("joint0_sensor")
        self.joint0_sensor.enable(timestep)
        self.joint1_sensor = self.__robot.getDevice("joint1_sensor")
        self.joint1_sensor.enable(timestep)
        self.joint2_sensor = self.__robot.getDevice("joint2_sensor")
        self.joint2_sensor.enable(timestep)
        self.joint3_sensor = self.__robot.getDevice("joint3_sensor")
        self.joint3_sensor.enable(timestep)
        self.joint4_sensor = self.__robot.getDevice("joint4_sensor")
        self.joint4_sensor.enable(timestep)
        self.joint5_sensor = self.__robot.getDevice("joint5_sensor")
        self.joint5_sensor.enable(timestep)

        self.__target_pos = [1, 0, 0, 0, 0, 0, 0]

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(
            Float32MultiArray, 'my_robot_cmd', self.__cmd_callback, 1)

        self.joint0_sensor_pub = self.__node.create_publisher(
            Float32, "/joint0_sensor", 1)
        self.joint1_sensor_pub = self.__node.create_publisher(
            Float32, "/joint1_sensor", 1)
        self.joint2_sensor_pub = self.__node.create_publisher(
            Float32, "/joint2_sensor", 1)
        self.joint3_sensor_pub = self.__node.create_publisher(
            Float32, "/joint3_sensor", 1)
        self.joint4_sensor_pub = self.__node.create_publisher(
            Float32, "/joint4_sensor", 1)
        self.joint5_sensor_pub = self.__node.create_publisher(
            Float32, "/joint5_sensor", 1)
        
    def __cmd_callback(self, msg):
        self.__target_pos = msg.data

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        msg = Float32()
        msg.data = self.joint0_sensor.getValue()
        self.joint0_sensor_pub.publish(msg)

        msg = Float32()
        msg.data = self.joint1_sensor.getValue()
        self.joint1_sensor_pub.publish(msg)

        msg = Float32()
        msg.data = self.joint2_sensor.getValue()
        self.joint2_sensor_pub.publish(msg)

        msg = Float32()
        msg.data = self.joint3_sensor.getValue()
        self.joint3_sensor_pub.publish(msg)

        msg = Float32()
        msg.data = self.joint4_sensor.getValue()
        self.joint4_sensor_pub.publish(msg)

        msg = Float32()
        msg.data = self.joint5_sensor.getValue()
        self.joint5_sensor_pub.publish(msg)

        self.joint0_motor.setPosition(self.__target_pos[1])
        self.joint1_motor.setPosition(self.__target_pos[2])
        self.joint2_motor.setPosition(self.__target_pos[3])
        self.joint3_motor.setPosition(self.__target_pos[4])
        self.joint4_motor.setPosition(self.__target_pos[5])
        self.joint5_motor.setPosition(self.__target_pos[6])

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from can_msgs.msg import Frame
from industrial_msgs.msg import RobotStatus, TriState, RobotMode


class SentCanbusMessages(Node):
    def __init__(self):
        super().__init__('sent_msgs')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('device_id', 0),
                ('command_id', 0),
                ('green', 0),
                ('yellow', 0),
                ('red', 0),
                ('blue', 0),
                ('beeper', 0)
        ])
        self.my_data = 0

        self.send_topic = self.declare_parameter('send_topic', '/sent_canbus_messages').value
        self.subscription = self.create_subscription(RobotStatus, '/sirius_status', self.get_robot_status, 10)
        self.subscription
        self.publisher = self.create_publisher(Frame, self.send_topic, 10)
        self.my_device_id = self.get_parameter('device_id').value
        self.my_command_id = self.get_parameter('command_id').value
        self.my_green = str(self.get_parameter('green').value)
        self.my_yellow= str(self.get_parameter('yellow').value)
        self.my_blue = str(self.get_parameter('blue').value)
        self.my_red = str(self.get_parameter('red').value)
        self.my_beeper = str(self.get_parameter('beeper').value)



    def publish_can_frame(self, data_to_msg, frame: Frame = None):
        msg = Frame()
        msg.id = (self.my_device_id <<5) | self.my_command_id
        msg.is_extended = False
        msg.is_error = False
        msg.dlc = len(data_to_msg)
        msg.data = [int(data_to_msg[i]) if i < len(data_to_msg) else 0 for i in range(8)]
        self.publisher.publish(msg)
        self.get_logger().info(f"Published CAN frame {data_to_msg}")

    def get_robot_status(self, msg):
        if msg.e_stopped.val ==TriState.FALSE and msg.mode.val == RobotMode.AUTO:
            self.my_blue= 1

        elif msg.e_stopped.val == TriState.FALSE and msg.drives_powered.val == TriState.TRUE:
            self.my_red= 1

        # elif msg.data == "joy_diff_drive":
        #     self.my_green = 1

        elif msg.mode.val == RobotMode.MANUAL and msg.e_stopped.val == TriState.FALSE and msg.motion_possible.val == TriState.TRUE:
            self.my_yellow = 1

        self.my_data  = [int(self.my_blue), int(self.my_yellow), int(self.my_red), int(self.my_beeper)]
        
        self.publish_can_frame(self.my_data)
        

def main(args=None):
    rclpy.init(args=args)
    sent_canbus_messages = SentCanbusMessages()
    rclpy.spin(sent_canbus_messages)
    sent_canbus_messages.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
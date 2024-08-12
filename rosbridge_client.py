import roslibpy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Imu
from std_msgs.msg import Header
import base64
import argparse
import numpy as np

class RosbridgeSubscriber(Node):
    def __init__(self, rosbridge_ip, rosbridge_port):
        super().__init__('rosbridge_subscriber')

        # Publishers for compressed topics and IMU
        self.publisher_depth_compressed = self.create_publisher(CompressedImage, '/camera/depth/image_raw/compressed', 10)
        self.publisher_color_compressed = self.create_publisher(CompressedImage, '/camera/color/image_raw/compressed', 10)
        self.publisher_imu = self.create_publisher(Imu, '/imu/data_raw', 10)

        # Roslibpy client
        self.client = roslibpy.Ros(host=rosbridge_ip, port=rosbridge_port)
        self.client.run()

        # Subscribers for compressed1 topics and IMU data
        self.listener_depth_compressed = roslibpy.Topic(self.client, '/camera/depth/image_raw/compressed1', 'sensor_msgs/CompressedImage')
        self.listener_depth_compressed.subscribe(self.callback_depth_compressed)

        self.listener_color_compressed = roslibpy.Topic(self.client, '/camera/color/image_raw/compressed1', 'sensor_msgs/CompressedImage')
        self.listener_color_compressed.subscribe(self.callback_color_compressed)

        self.listener_imu = roslibpy.Topic(self.client, '/imu/data_raw', 'sensor_msgs/Imu')
        self.listener_imu.subscribe(self.callback_imu)

    def callback_depth_compressed(self, message):
        # Convert roslibpy message to ROS 2 message for compressed depth image
        ros_msg = CompressedImage()
        ros_msg.header = Header()
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        ros_msg.header.frame_id = message['header']['frame_id']
        ros_msg.format = message['format']

        data = message['data']
        if isinstance(data, str):
            data = base64.b64decode(data)

        ros_msg.data = np.frombuffer(data, dtype=np.uint8).tobytes()
        self.publisher_depth_compressed.publish(ros_msg)

    def callback_color_compressed(self, message):
        # Convert roslibpy message to ROS 2 message for compressed color image
        ros_msg = CompressedImage()
        ros_msg.header = Header()
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        ros_msg.header.frame_id = message['header']['frame_id']
        ros_msg.format = message['format']

        data = message['data']
        if isinstance(data, str):
            data = base64.b64decode(data)

        ros_msg.data = np.frombuffer(data, dtype=np.uint8).tobytes()
        self.publisher_color_compressed.publish(ros_msg)

    def callback_imu(self, message):
        # Convert roslibpy message to ROS 2 message for IMU data
        ros_msg = Imu()
        ros_msg.header = Header()
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        ros_msg.header.frame_id = message['header']['frame_id']

        ros_msg.orientation.x = message['orientation']['x']
        ros_msg.orientation.y = message['orientation']['y']
        ros_msg.orientation.z = message['orientation']['z']
        ros_msg.orientation.w = message['orientation']['w']

        ros_msg.angular_velocity.x = message['angular_velocity']['x']
        ros_msg.angular_velocity.y = message['angular_velocity']['y']
        ros_msg.angular_velocity.z = message['angular_velocity']['z']

        ros_msg.linear_acceleration.x = message['linear_acceleration']['x']
        ros_msg.linear_acceleration.y = message['linear_acceleration']['y']
        ros_msg.linear_acceleration.z = message['linear_acceleration']['z']

        self.publisher_imu.publish(ros_msg)

    def destroy_node(self):
        self.listener_depth_compressed.unsubscribe()
        self.listener_color_compressed.unsubscribe()
        self.listener_imu.unsubscribe()
        self.client.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='ROS 2 to ROS Bridge Subscriber')
    parser.add_argument('--rosbridge_ip', type=str, required=True, help='IP address of the rosbridge server')
    parser.add_argument('--rosbridge_port', type=int, default=9090, help='Port of the rosbridge server')

    args = parser.parse_args()

    rosbridge_ip = args.rosbridge_ip
    rosbridge_port = args.rosbridge_port

    rosbridge_subscriber = RosbridgeSubscriber(rosbridge_ip, rosbridge_port)

    try:
        rclpy.spin(rosbridge_subscriber)
    except KeyboardInterrupt:
        pass

    rosbridge_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

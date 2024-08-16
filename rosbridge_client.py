import roslibpy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Imu
from nav_msgs.msg import Path  # 导入 Path 消息类型
from std_msgs.msg import Header, String
import base64
import argparse
import numpy as np
import time
from geometry_msgs.msg import PoseStamped

class RosbridgeSubscriber(Node):
    def __init__(self, rosbridge_ip, rosbridge_port):
        super().__init__('rosbridge_subscriber')
        self.rosbridge_ip = rosbridge_ip
        self.rosbridge_port = rosbridge_port

        # Publishers for compressed topics, IMU, String, and Path
        self.publisher_depth_compressed = self.create_publisher(CompressedImage, '/camera/depth/image_raw/compressed', 10)
        self.publisher_color_compressed = self.create_publisher(CompressedImage, '/camera/color/image_raw/compressed', 10)
        self.publisher_imu = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.publisher_tag_name = self.create_publisher(String, '/tag_name', 10)
        self.publisher_plan = self.create_publisher(Path, '/plan', 10)  # 新增 Path 格式的话题发布者

        # Initial connection attempt
        self.connect_to_rosbridge()

    def connect_to_rosbridge(self):
        self.get_logger().info(f'Connecting to rosbridge at {self.rosbridge_ip}:{self.rosbridge_port}...')
        self.client = roslibpy.Ros(host=self.rosbridge_ip, port=self.rosbridge_port)

        def on_connected():
            self.get_logger().info('Connect success')
            self.start_subscribers()

        def on_disconnected(proto=None):
            self.get_logger().warn('Disconnected from rosbridge server. Attempting to reconnect...')
            self.retry_connection()

        self.client.on_ready(on_connected, run_in_thread=True)
        self.client.on('close', on_disconnected)  # Use 'close' event for disconnect

        self.client.run()

    def retry_connection(self):
        self.client.terminate()  # Ensure the previous client is terminated
        time.sleep(5)  # Wait before trying to reconnect
        self.get_logger().info('Reconnect')
        self.connect_to_rosbridge()

    def start_subscribers(self):
        # Subscribers for compressed topics, IMU data, String, and Path
        self.listener_depth_compressed = roslibpy.Topic(self.client, '/camera/depth/image_raw/compressed1', 'sensor_msgs/CompressedImage')
        self.listener_depth_compressed.subscribe(self.callback_depth_compressed)

        self.listener_color_compressed = roslibpy.Topic(self.client, '/camera/color/image_raw/compressed1', 'sensor_msgs/CompressedImage')
        self.listener_color_compressed.subscribe(self.callback_color_compressed)

        self.listener_imu = roslibpy.Topic(self.client, '/imu/data_raw', 'sensor_msgs/Imu')
        self.listener_imu.subscribe(self.callback_imu)

        self.listener_tag_name = roslibpy.Topic(self.client, '/tag_name', 'std_msgs/String')
        self.listener_tag_name.subscribe(self.callback_tag_name)

        self.listener_plan = roslibpy.Topic(self.client, '/plan', 'nav_msgs/Path')  # 新增 Path 格式的订阅者
        self.listener_plan.subscribe(self.callback_plan)

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

    def callback_tag_name(self, message):
        # Convert roslibpy message to ROS 2 message for String
        ros_msg = String()
        ros_msg.data = message['data']
        self.publisher_tag_name.publish(ros_msg)

    def callback_plan(self, message):
        # Convert roslibpy message to ROS 2 message for Path
        ros_msg = Path()
        ros_msg.header = Header()
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        ros_msg.header.frame_id = message['header']['frame_id']

        ros_msg.poses = []
        for pose in message['poses']:
            ros_pose = PoseStamped()
            ros_pose.header = Header()
            ros_pose.header.stamp = self.get_clock().now().to_msg()
            ros_pose.header.frame_id = pose['header']['frame_id']

            ros_pose.pose.position.x = pose['pose']['position']['x']
            ros_pose.pose.position.y = pose['pose']['position']['y']
            ros_pose.pose.position.z = pose['pose']['position']['z']

            ros_pose.pose.orientation.x = pose['pose']['orientation']['x']
            ros_pose.pose.orientation.y = pose['pose']['orientation']['y']
            ros_pose.pose.orientation.z = pose['pose']['orientation']['z']
            ros_pose.pose.orientation.w = pose['pose']['orientation']['w']

            ros_msg.poses.append(ros_pose)

        self.publisher_plan.publish(ros_msg)

    def destroy_node(self):
        if self.client.is_connected:
            self.listener_depth_compressed.unsubscribe()
            self.listener_color_compressed.unsubscribe()
            self.listener_imu.unsubscribe()
            self.listener_tag_name.unsubscribe()
            self.listener_plan.unsubscribe()  # 取消 Path 格式订阅
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

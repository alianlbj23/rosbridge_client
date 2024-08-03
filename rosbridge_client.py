import roslibpy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
import base64
import numpy as np
import argparse

class RosbridgeSubscriber(Node):
    def __init__(self, rosbridge_ip, rosbridge_port):
        super().__init__('rosbridge_subscriber')
        self.publisher_ = self.create_publisher(CompressedImage, '/out/compressed', 10)

        self.client = roslibpy.Ros(host=rosbridge_ip, port=rosbridge_port)
        self.client.run()

        self.listener = roslibpy.Topic(self.client, '/out/compressed', 'sensor_msgs/CompressedImage')
        self.listener.subscribe(self.callback)

    def callback(self, message):
        # Convert roslibpy message to ROS 2 message
        ros_msg = CompressedImage()
        ros_msg.header = Header()
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        ros_msg.format = message['format']

        # Decode base64-encoded string if necessary
        data = message['data']
        if isinstance(data, str):
            data = base64.b64decode(data)

        ros_msg.data = np.frombuffer(data, dtype=np.uint8).tobytes()
        self.publisher_.publish(ros_msg)

    def destroy_node(self):
        self.listener.unsubscribe()
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

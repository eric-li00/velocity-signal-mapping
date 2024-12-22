 
import socket  
import rclpy  
from rclpy.node import Node 
from std_msgs.msg import Int32  # ROS2 standard message type, used for publishing integers 
  
class UDPToROS2Publisher(Node):  
  
    def __init__(self):  
        super().__init__('udp_to_ros2_publisher')  
        self.publisher_ = self.create_publisher(Int32, 'udp_data', 10)  # Create a publisher and publish to the 'udp_data' topic  
        self.socket_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create UDP socket 
        self.socket_.bind(('192.168.1.106', 17000))  # Bind to local address and port  
        self.timer_ = self.create_timer(1.0, self.check_queue)  # Create a timer to periodically check the queue 
        self.get_logger().info(f'The node has been started, and the topic has been published. Waiting for data from the socket')
    def check_queue(self):  
        try:  
            while True:
                # Attempt to receive data from a UDP socket
                data, addr = self.socket_.recvfrom(1024)  #1024 4096
            # Assuming that the data is an ASCII-encoded integer, convert the received string data to an int
                integer_data1 = int(data.decode('ascii')) 
                #unit conversion
                integer_data=round(integer_data1*3.6)
            #  Create a ROS2 message and publish it.
                msg = Int32()  
                msg.data = integer_data  
                self.publisher_.publish(msg)  
                self.get_logger().info(f'Received {integer_data} from {addr} and published to ROS2')  
                
        except KeyboardInterrupt:
            pass 
        finally:
            pass
  
def main(args=None):  
    rclpy.init(args=args)  
    udp_publisher = UDPToROS2Publisher()  
    rclpy.spin(udp_publisher)  
  
    #Close the socket and ROS2 node
    udp_publisher.socket_.close()  
    udp_publisher.destroy_node()  
    rclpy.shutdown()  
  

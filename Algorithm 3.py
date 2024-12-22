# udp_ros2_subscriber.py  
import rclpy  
from rclpy.node import Node  
from std_msgs.msg import Int32  # ROS2 standard message type, used for subscribing to integers
import pigpio
import time
class UDPROS2Subscriber(Node):  
  
    def __init__(self):  
        super().__init__('udp_ros2_subscriber') 
        self.pi = pigpio.pi()
        if not self.pi.connected:
            exit()
        self.gpio_pin = 13                              #(bcm code), corresponding to on-board code 33
        self.pi.set_mode(self.gpio_pin, pigpio.OUTPUT)  # Set GPIO 13 to output mode (bcm encoding)
        self.pi.set_PWM_range(self.gpio_pin, 10000)     # Set the PWM range to 25-40000
        self.pi.set_PWM_frequency(self.gpio_pin, 50)    # Set the PWM frequency to 50 Hz
        # Open the node, and the buzzer will start This vehicle uses a 12065 passive buzzer with a frequency of 4000hz, an operating voltage of 1-3V, and a rated voltage of 1.5V
        self.buzzer_gpio_pin = 5                             #(bcm code), corresponding to on-board code 29
        self.pi.set_mode(self.buzzer_gpio_pin, pigpio.OUTPUT)# Set GPIO 5 to output mode (bcm encoding)
        self.pi.set_PWM_range(self.buzzer_gpio_pin, 100)     # Set the PWM range to 25-40000, the resolution of the PWM signal
        self.pi.set_PWM_frequency(self.buzzer_gpio_pin, 3500)# Set the PWM frequency to 4000 Hz Cycle 0.25ms
        
        self.pi.set_PWM_dutycycle(self.buzzer_gpio_pin, 66)    # Setting the PWM duty cycle to 66% within the range of 30.3% to 66.6% to 90.9% of the rated voltage can enable the buzzer to operate normally, and the larger the duty cycle, the louder the sound.
        
        time.sleep(2)# Let the buzzer sound for a period of time, for example, 2 seconds, which increases the delay
        self.pi.set_PWM_dutycycle(self.buzzer_gpio_pin, 0)    # Set the PWM duty cycle to 0, and turn off the buzzer duty cycle range of 30.3% to 66.6% to 90.9% of the rated voltage duty cycle to enable the buzzer to work normally. The larger the duty cycle, the louder the sound.


        self.subscription_ = self.create_subscription(Int32, 'udp_data', self.listener_callback, 10)  #  Create a subscriber to subscribe to the 'udp_data' topic. 
        self.get_logger().info(f'The rear-wheel drive node has been started, and the topic has been subscribed. Waiting for the data from the publisher node')

    def listener_callback(self, msg): 
                # Called when a message is received
        self.get_logger().info(f'Received {msg.data} from ROS2') 
        try:
        
            self.pi.set_PWM_dutycycle(self.gpio_pin, round(800 + (msg.data - 0) * (1000 - 800) / (135 - 0)) )   # Set the PWM duty cycle to 5%-7.5%-10% corresponding to values of 500, 750, and 1000.
          
            print(msg.data)
  

        except KeyboardInterrupt:
     
            pass
        
      
 
  
def main(args=None):  
    rclpy.init(args=args)  
    udp_subscriber = UDPROS2Subscriber()  
    try:

        rclpy.spin(udp_subscriber) 
    except KeyboardInterrupt:  
        pass
    finally:
        
        udp_subscriber.pi.set_PWM_dutycycle(udp_subscriber.gpio_pin, 0)    # Stop the PWM signal output.
        udp_subscriber.pi.set_PWM_dutycycle(udp_subscriber.buzzer_gpio_pin, 66)    # Set the PWM duty cycle to 66% (a duty cycle range of 30.3% to 66.6% to 90.9% of the rated voltage can make the buzzer operate normally, and the louder the sound, the higher the duty cycle).
        
        time.sleep(1)# Make the buzzer sound for a duration (e.g., 1 second), with added delay.
        udp_subscriber.pi.set_PWM_dutycycle(udp_subscriber.buzzer_gpio_pin, 0)    # Set the PWM duty cycle to 0, and turn off the buzzer duty cycle range of 30.3% to 66.6% to 90.9% of the rated voltage duty cycle to enable the buzzer to work normally. The larger the duty cycle, the louder the sound.
        udp_subscriber.pi.stop()
        udp_subscriber.get_logger().info(f'The node is closing')
    
        # Shutdown the ROS2 node  
        udp_subscriber.destroy_node() 
        
        rclpy.shutdown() 
        
        udp_subscriber.get_logger().info(f'The node has been completely shut down finish it')
       






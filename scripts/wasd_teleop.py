import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Save terminal settings
settings = termios.tcgetattr(sys.stdin)

def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rclpy.init()
    node = rclpy.create_node('wasd_teleop')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    speed = 2.0  # Linear speed (m/s)
    turn = 1.0   # Angular speed (rad/s)
    
    print("🎮 WASD to drive, SPACE to stop, Ctrl+C to quit")
    
    try:
        while True:
            key = get_key()
            msg = Twist()
            
            if key == 'w':
                msg.linear.x = speed
                print("↑ Moving Forward")
            elif key == 's':
                msg.linear.x = -speed
                print("↓ Moving Backward")
            elif key == 'a':
                msg.angular.z = turn
                print("← Turning Left")
            elif key == 'd':
                msg.angular.z = -turn
                print("→ Turning Right")
            elif key == ' ':
                print("🛑 Stopping")
                # Sends all zeros
            elif key == '\x03': # Ctrl+C
                break
                
            pub.publish(msg)
            
    except Exception as e:
        print(e)
    finally:
        # Stop the robot when exiting
        pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

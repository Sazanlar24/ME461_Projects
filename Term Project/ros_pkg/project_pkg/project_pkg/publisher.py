import rclpy, random
from rclpy.node import Node
from std_msgs.msg import String

# change these if row and columns are different than the example
R = 5
C = 8

# move2target topic variables
number_of_seconds = 20
current_target_index = 10

# game_control topic list
game_control_list = ["start", "pause", "resume", "stop"]

class Publisher(Node):
    def __init__(self):
        super().__init__('publisher')  # Name of the node

        # Create two publishers for two different topics
        self.publisher_cell_values = self.create_publisher(String, 'cell_values', 10)  
        self.publisher_move2target = self.create_publisher(String, 'move2target', 10)  
        self.publisher_game_control = self.create_publisher(String, 'game_control', 10)  
        self.publisher_my_target = self.create_publisher(String, 'my_target', 10)  
        self.publisher_my_position = self.create_publisher(String, 'my_position', 10)  

        # Create timers for each topic's publishing rate
        self.timer_cell_values  = self.create_timer(1.0, self.timer_cell_values_callback)  # Publish to topic every 1 second
        self.timer_move2target  = self.create_timer(1.0, self.timer_move2target_callback)
        self.timer_game_control = self.create_timer(1.0, self.timer_game_control_callback)
        self.timer_my_target    = self.create_timer(1.0, self.timer_my_target_callback)
        self.timer_my_position  = self.create_timer(1.0, self.timer_my_position_callback)

        self.counter_cell_values  = 0  # Counters for topics
        self.counter_move2target  = 0  
        self.counter_game_control = 0  
        self.counter_my_target    = 0  
        self.counter_my_position  = 0  

        self.cell_value_string = ""
        self.create_cell_values(R*C)

        self.game_control_string = random.choice(game_control_list)
    
    def create_cell_values(self, cell_number):
        list_numbers = [i for i in range(1, cell_number+1)]

        while len(list_numbers) > 1:
            number = random.choice(list_numbers)
            self.cell_value_string += str(number)
            self.cell_value_string += ", "
            list_numbers.remove(number)
        self.cell_value_string += str(list_numbers[0])

    def timer_cell_values_callback(self):
        msg = String()
        msg.data = self.cell_value_string
        self.publisher_cell_values.publish(msg)
        self.get_logger().info(f'Cell values are published: "{msg.data}"')
        self.counter_cell_values += 1

    def timer_move2target_callback(self):
        msg = String()
        msg.data = f"{number_of_seconds},{current_target_index}"
        self.publisher_move2target.publish(msg)
        self.get_logger().info(f'Move2target numbers are published: "{msg.data}"')
        self.counter_move2target += 1

    def timer_game_control_callback(self):
        msg = String()
        msg.data = self.game_control_string
        self.publisher_game_control.publish(msg)
        self.get_logger().info(f'Game control string is published: "{msg.data}"')
        self.counter_game_control += 1

    def timer_my_target_callback(self):
        msg = String()
        msg.data = f'Topic2: Hello from publisher 2, message #{self.counter_my_target}'
        self.publisher_my_target.publish(msg)
        self.get_logger().info(f'Publishing to topic2: "{msg.data}"')
        self.counter_my_target += 1

    def timer_my_position_callback(self):
        msg = String()
        msg.data = f'Topic2: Hello from publisher 2, message #{self.counter_my_position}'
        self.publisher_my_position.publish(msg)
        self.get_logger().info(f'Publishing to topic2: "{msg.data}"')
        self.counter_my_position += 1        


def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    node = Publisher()  # Create the node
    rclpy.spin(node)  # Keep the node alive and responsive to callbacks
    # Clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

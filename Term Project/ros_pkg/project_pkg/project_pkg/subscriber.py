import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Subscriber(Node):
    def __init__(self):
        super().__init__('subscriber')  # Name of the node

        # Create two subscriptions for two different topics
        self.subscription1 = self.create_subscription(
            String,
            'cell_values',
            self.listener_callback_cell_values,
            10
        )
        self.subscription2 = self.create_subscription(
            String,
            'move2target',
            self.listener_callback_move2target,
            10
        )
        self.subscription3 = self.create_subscription(
            String,
            'game_control',
            self.listener_callback_game_control,
            10
        )
        self.subscription4 = self.create_subscription(
            String,
            'my_target',
            self.listener_callback_my_target,
            10
        )
        self.subscription5 = self.create_subscription(
            String,
            'my_position',
            self.listener_callback_my_position,
            10
        )

        # Prevent unused variable warning
        self.subscription1
        self.subscription2
        self.subscription3
        self.subscription4
        self.subscription5

        # list that holds the bonus point in each cell
        # number in 0th index gives the point in first cell and so on
        self.cell_values = []

        # move2target variables
        self.number_of_seconds    = 0
        self.current_target_index = 0

        # game_control topic string
        self.robot_state = ""

        self.group_name = "SAZAN"
        self.current_cell = 0

        self.next_cell_to_move = 0

    def listener_callback_cell_values(self, msg):
        #self.get_logger().info(f'Received on cell_values topic: "{msg.data}"')
        
        str = msg.data.replace(" ", "")  # erases empty characters
        list = str.split(",")

        new_values = [int(x) for x in list]
        if new_values != self.cell_values:
            self.cell_values = new_values
            print("New points list is arrived!")    # TODO: it should blink LEDs1 GREEN 

    def listener_callback_move2target(self, msg):
        #self.get_logger().info(f'Received on topic2: "{msg.data}"')

        str = msg.data.replace(" ", "")  # erases empty characters
        list = str.split("")
        int1 = list[0]
        int2 = list[1]

        # if any of the two variables is changed, make a new plan after equating new integers
        if self.number_of_seconds != int1 or self.current_target_index != int2:
            self.number_of_seconds    = int1
            self.current_target_index = int2
            # TODO: make a plan about the path planning
            # during planning, LEDs1 will be solid BLUE
            # after planning is over, LEDs1 will fast blink GREEN, indicating it is ready to move
            # show new plan in GUI
    
    def listener_callback_game_control(self, msg):
        self.game_control_string = msg.data

        if self.game_control_string == "start":
            self.robot_state = "start"
            # TODO: LEDs1 turn into solid WHITE
            # timer_service should be initiated
            # robot should start moving.
            # start a countdown from --self.number_of_seconds--
            # show remaining time on GUI
            pass
        elif self.game_control_string == "pause":
            self.robot_state = "pause"
            # TODO: robot should pause
            # wait for resume or stop
            # take note of the remaining time
            # LEDs1 should blink RED
            pass
        elif self.game_control_string == "resume":
            if self.robot_state == "pause": # if robot is paused before
                # TODO: continue executing the existing plan
                # set LEDs1 solid WHITE
                # initiate new timer_service with the remaining time
                pass
        elif self.game_control_string == "stop":
            # TODO: terminate tasks
            # set LEDs1 solid RED
            pass
        else:
            pass



    def listener_callback_my_target(self, msg):
        self.get_logger().info(f'Received on topic2: "{msg.data}"')
    
    def listener_callback_my_position(self, msg):
        self.get_logger().info(f'Received on topic1: "{msg.data}"')

    
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    node = Subscriber()  # Create the subscriber node
    rclpy.spin(node)  # Keep the node alive and responsive to callbacks

    # Clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

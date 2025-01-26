"""
TODO: robotun konumunu bulup my_position topic'ine gönderecek
TODO: konum değişirse --> blink LEDs2 WHITE
TODO: konum sabitse   --> LEDs2 solid WHITE

cell_values topic'inden string alacak, kaydedecek. TODO: Yeni cell_value gelince --> LEDs1 Green
TODO: robot buralardan geçtikçe puanı toplayacak, anlık olarak total_points topic'ine gönderecek

move2target topic'inden string alacak, kaydedecek.
TODO: plan yapacak, ve bu planı PİCO'ya anlatabilecek koda (90 derece dön, şu kadar ileri vs.) çevirecek, gönderecek
"""

import time
import rclpy, random
from rclpy.node import Node
from std_msgs.msg import String, Int16

from project_pkg.planner import plan_path

class ROS_Image_Processing(Node):
    def __init__(self):
        super().__init__('image_processing')  # Name of the node 
        self.publisher_my_position  = self.create_publisher(String, 'my_position', 10)
        self.publisher_my_target    = self.create_publisher(String, 'my_target', 10)  
        self.publisher_total_points = self.create_publisher(Int16, 'total_points', 10) 
  
        self.robot_location  = 0  #şu anki konum            # TODO: ANLIK DEĞİŞMELİ
        self.target_location = 0  #bir sonraki target cell  # TODO: ANLIK DEĞİŞMELİ

        self.timer_my_position  = self.create_timer(1.0, self.publish_my_position)
        self.timer_my_target    = self.create_timer(1.0, self.publish_my_target)
        self.timer_total_points = self.create_timer(1.0, self.publish_total_points)  # Add timer for total_points
        
        self.counter_my_position  = 0 
        self.counter_total_points = 0 
        self.counter_my_target = 0 

        self.group_name = "sazanlar"
        self.cell_values = [] 
        self.total_points = 0

        self.total_seconds = 0
        self.selected_target = 0

        self.remaining_time = 0

        self.new_path_is_found = False

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
            'move2target',
            self.listener_callback_game_control,
            10
        )

    def publish_my_position(self):
        msg = String()
        msg.data = self.group_name + ", " + str(self.robot_location)
        self.publisher_my_position.publish(msg)
        self.get_logger().info(f'My position is published: "{msg.data}"')
        self.counter_my_position += 1

    def publish_my_target(self):
        msg = String()
        msg.data = self.group_name + ", " + str(self.target_location)
        self.publisher_my_target.publish(msg)
        self.get_logger().info(f'My target is published: "{msg.data}"')
        self.counter_my_target += 1

    def publish_total_points(self):
        msg = Int16()
        msg.data = self.total_points
        self.publisher_total_points.publish(msg)
        self.get_logger().info(f'Total_points is published: "{msg.data}"')
        self.counter_total_points += 1

    def listener_callback_cell_values(self, msg):
        #self.get_logger().info(f'Received on cell_values topic: "{msg.data}"')
        
        str = msg.data.replace(" ", "")  # erases empty characters
        list = str.split(",")        

        new_values = [int(x) for x in list]
        self.cell_values = new_values
        print(self.cell_values)
        print(type(self.cell_values))
        
        print("New points list is arrived!")    
        # TODO: it should blink LEDs1 GREEN

    def listener_callback_move2target(self, msg):
        self.get_logger().info(f'Received on cell_values topic: "{msg.data}"')
        str = msg.data.replace(" ", "")  # erases empty characters

        seconds, target = str.split(",")
        
        # if new target is arrived
        if target != self.selected_target:
            self.total_seconds, self.selected_target = seconds, target
            self.new_path_is_found = False 

        # path is not known (either no target is arrived yet, or target is changed)
        if self.new_path_is_found is False:
            """
            TODO: burdaki planner düzeltilmeli
            TODO: During planning LEDs1 will be solid BLUE
            result = plan_path(self.cell_values, self.robot_location, self.selected_target, 15) # TODO: süreye göre adım dönüşler daha uzun sürer aslında
            print("Maximum Points:", result[0])
            print("Best Path:", result[1])
            self.new_path_is_found = True
            TODO: After planning, LEDs1 will blink GREEN but faster
            TODO: yapılan planı ve alınması planlanan puanı ayrı bir topic'e gönder
            """
            print("plan is done")

    def listener_callback_game_control(self, msg):
        self.game_control_string = msg.data.upper()

        if self.game_control_string == "START":
            self.robot_state = "START"
            # TODO: LEDs1 turn into solid WHITE
            # timer_service should be initiated
            # robot should start moving.
            # start a countdown from --self.number_of_seconds--
            # TODO: bir topic'e kalan zamanı gönder
            pass
        elif self.game_control_string == "PAUSE":
            self.robot_state = "PAUSE"
            # TODO: robot should pause
            # wait for resume or stop
            # take note of the remaining time
            # LEDs1 should blink RED
            pass
        elif self.game_control_string == "RESUME":
            if self.robot_state == "PAUSE": # if robot is paused before
                # TODO: continue executing the existing plan
                # set LEDs1 solid WHITE
                # initiate new timer_service with the remaining time
                pass
            else:
                pass # discard
        elif self.game_control_string == "stop":
            # TODO: terminate tasks
            # set LEDs1 solid RED
            pass
        else:
            pass

def main(args=None):
    global publisher_node
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)
    # Create the ROS2 publisher node
    publisher_node = ROS_Image_Processing()

    rclpy.spin(publisher_node)    
    rclpy.shutdown()

from project_pkg.design import Ui_MainWindow
import time
from PyQt6.QtGui import QPixmap
from PyQt6.QtWidgets import QApplication, QMainWindow
import socket
import rclpy, random
from rclpy.node import Node
from std_msgs.msg import String, Int16
import threading

from project_pkg.planner import plan_path

HOST = '192.168.4.1'
PORT = 8080

buttons = []

class Publisher(Node):
    def __init__(self):
        super().__init__('GUI_node')  # Name of the node
        self.publisher_cell_values  = self.create_publisher(String, 'cell_values', 10)  
        self.publisher_move2target  = self.create_publisher(String, 'move2target', 10)  
        self.publisher_game_control = self.create_publisher(String, 'game_control', 10)   
        
        self.group_name = "sazanlar"
        self.robot_location  = 0  #su anki konum       # TODO: ANLIK DEĞİŞMELİ
        self.prev_location = 0
        self.target_location = 1  #bir sonraki konum       # TODO: ANLIK DEĞİŞMELİ
        self.selected_target = 0
        
        self.cell_value_list = []
        
        self.cell_value_string = ""
        self.game_control_string = ""
        self.expected_points = 0
        
        self.total_seconds   = 0

        self.subscription = self.create_subscription(
            Int16,
            'total_points',
            self.timer_total_points_callback,
            10
        )
    
        self.subscription2 = self.create_subscription(
            String,
            'expected_path_and_score',
            self.expected_path_and_score_callback,
            10
        )

        self.subscription2 = self.create_subscription(
            String,
            'my_position',
            self.my_position_callback,
            10
        )

        """
        try:
            # Connect to the Pico W server
            self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client.connect((HOST, PORT))
            print(f"Connected to {HOST}:{PORT}")
        except socket.error as e:
            print(f"Failed to connect to {HOST}:{PORT}. Error: {e}")
            exit(1)  # Exit the program if the connection fails"""
        
    def publish_cell_values(self):
        msg = String()
        msg.data = self.cell_value_string
        self.publisher_cell_values.publish(msg)
        self.get_logger().info(f'Cell values are published: "{msg.data}"')

    def publish_game_control(self):
        msg = String()
        msg.data = self.game_control_string
        self.publisher_game_control.publish(msg)
        self.get_logger().info(f'Game control string is published: "{msg.data}"')

    def timer_total_points_callback(self, msg):
        #self.get_logger().info(f'Total points received by GUI: "{msg.data}"')
        self.total_points = int(msg.data)
        total_point_label.setText(_translate("MainWindow", "Total Points: " + str(self.total_points)))

    def expected_path_and_score_callback(self, msg):
        my_list = msg.data.split(":")

        path_string = my_list[0].strip()
        expected_points = my_list[1].strip()

        # if new expected point is arrived
        if self.expected_points != int(expected_points): 
            expected_point_label.setText(_translate("MainWindow", "Expected Points: " + str(int(expected_points))))
            self.expected_set = True

        self.expected_points = int(expected_points)

        points_to_go = path_string.split(",")
        points_to_go = list(map(int, points_to_go)) # convert string integers into integers

        for i in range(1, len(points_to_go)-1):
            button = buttons[points_to_go[i]]
            self.buttonColor(button, "yellow")

        print("New path is received by GUI", path_string)
        #print(expected_points)    
        #print("Expected points is received: ", self.expected_points)

    def my_position_callback(self, msg):
        group_name, robot_location_string = msg.data.split(",")
        self.group_name = group_name
        robot_location_string = robot_location_string.strip()
        
        if robot_location_string == "None":
            self.robot_location = self.prev_location
        else:
            self.prev_location = self.robot_location
            self.robot_location = int(robot_location_string)
        
        #print("Robot location is received by GUI: ", self.robot_location)

        if (0 < self.robot_location < 41):
            try:
                button_robot = buttons[self.robot_location]
                button_prev  = buttons[self.prev_location]
                self.buttonColor(button_prev, "white")
                self.buttonColor(button_robot, "red")
            except:
                pass

    def publish_move2target(self):
        msg = String()
        msg.data = f"{self.total_seconds},{self.selected_target}"
        self.publisher_move2target.publish(msg)
        self.get_logger().info(f'Move2target numbers are published: "{msg.data}"')
        self.cell_value_list = list(map(int, self.cell_value_list))

    def sendPico(self, message:str):
        # Validate the message before sending (optional, implement as needed)
        if not message.strip():
            print("Empty message. Please enter a valid command.")

        # Send data to the server
        self.client.sendall(message.encode())
        print(f'Message sent to pico: {message}')

        # Receive and process the response
        """
        while True:
            try:
                response = publisher_node.client.recv(1024)
                if not response:
                    print("No response received. Server might have disconnected.")
                print("Received from Pico:", response.decode())
            except socket.error as e:
                print(f"Error receiving data: {e}")

            except Exception as e:
                print(f"An error occurred: {e}")

            return response.decode()"""
        
    def buttonColor(self, buttonObject: object, color: str):
        buttonObject.setStyleSheet(f"background-color: {color}")

class Window(QMainWindow, Ui_MainWindow):
    def __init__(self):
        global buttons, expected_point_label, total_point_label, _translate
        super().__init__()
        self.setupUi(self)
        self.init_vars()

        buttons = self.buttons
        expected_point_label = self.label_expected_points
        total_point_label = self.label_total_points
        _translate = self.translate
        self.button_target = None

        self.pushButton.clicked.connect(self.textHandler)
        for i in range(1,len(self.buttons)+1): self.buttons[i].clicked.connect(self.clickHandler)
        for button in self.radioButtons: button.clicked.connect(self.radioHandler)

    def setDisplay(self, textOrImage:str, textOrPath:str):
        self.scene.clear()
        if textOrImage == 'text': 
            self.scene.addText(textOrPath)
        elif textOrImage == 'image': 
            pixmap = QPixmap(textOrPath)
            self.scene.addPixmap(pixmap)
        else: print('Just provide str(text) or str(image) as first parameter.')

        self.view = self.graphicsView.setScene(self.scene)

    def radioHandler(self): 
        button = self.sender().objectName()
        #print(f'{button} clicked!')
        publisher_node.game_control_string = str(button)
        publisher_node.publish_game_control()

    def clickHandler(self):

        if self.button_target != None:
            self.buttonColor(self.button_target,'white')  
            #print("Old target: ", self.button_target.objectName()[11:])  

        self.button_target = self.sender()

        self.buttonColor(self.sender(),'green')
        buttonName = self.sender().objectName()
        #print(f'{buttonName} clicked!')

        publisher_node.selected_target = buttonName[11:]
        #print("New target: ", publisher_node.selected_target)

    def textHandler(self):
        text = self.textEdit.toPlainText()
        #print(f'Text sent: {text}')
        # Assign cell values to map for visual purposes
        if text[:4] == 'path': 
            path = text[5:]
            try:
                cellValues = open(path, "r").read().rstrip().lstrip().split(',')
                cellValues = [x.rstrip() for x in cellValues]
                publisher_node.cell_value_string = cellValues

                for value in cellValues:
                    value = value.replace(" ", "")  # erases empty characters

                for i in range(1,len(self.buttons)+1): self.buttons[i].setText(cellValues[i-1])
                
                string = ""
                while len(cellValues) > 1:
                    number = cellValues[0]
                    string += str(number)
                    string += ","
                    cellValues.remove(number)
                string += str(cellValues[0])
                
                publisher_node.cell_value_string = string                
                publisher_node.publish_cell_values()
            except:
                print("Could not open this path")            
        
        elif text[:4] == 'pico': 
            message = text[5:]
            publisher_node.sendPico(message)  

        elif text[:7] == 'seconds':
            publisher_node.total_seconds = int(text[8:])
            publisher_node.publish_move2target()

    def buttonColor(self, buttonObject:object, color:str):
        buttonObject.setStyleSheet(f"background-color : {color}") 

    def init_vars(self): 
        self.progressBar.setValue(0)
        for button in self.radioButtons: button.setChecked(False)
        self.setDisplay('image', '/home/me461/mnt/labs_ws/src/project_pkg/project_pkg/home.jpg')

"""
def main(args=None):
    app = QApplication([])
    window = Window()

    window.show()
    app.exec()
    client.close()
main()"""


def main(args=None):
    global publisher_node
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create the ROS2 publisher node
    publisher_node = Publisher()

    # Start a thread to spin the ROS2 node
    def ros_spin():
        rclpy.spin(publisher_node)
    
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    # Start the GUI application
    app = QApplication([])
    window = Window()

    window.show()
    app.exec()

    # Shut down the ROS2 node and clean up
    publisher_node.destroy_node()
    rclpy.shutdown()
    publisher_node.client.close()
    print("pico connection is stopped")
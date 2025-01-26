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

#client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#client.connect((HOST, PORT))

class Publisher(Node):
    def __init__(self):
        super().__init__('publisher')  # Name of the node
        self.publisher_cell_values  = self.create_publisher(String, 'cell_values', 10)  
        self.publisher_move2target  = self.create_publisher(String, 'move2target', 10)  
        self.publisher_game_control = self.create_publisher(String, 'game_control', 10)   
        
        self.group_name = "sazanlar"
        self.robot_location  = 0  #su anki konum       # TODO: ANLIK DEĞİŞMELİ
        self.target_location = 1  #bir sonraki konum       # TODO: ANLIK DEĞİŞMELİ
        self.selected_target = 0
        
        self.cell_value_list = []
        
        self.cell_value_string = ""
        self.game_control_string = ""
        
        self.total_seconds   = 0

        self.subscription = self.create_subscription(
            Int16,
            'total_points',
            self.timer_total_points_callback,
            10
        )
        
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
        self.get_logger().info(f'Total points received by GUI: "{msg.data}"')

    def publish_move2target(self):
        msg = String()
        msg.data = f"{self.total_seconds},{self.selected_target}"
        self.publisher_move2target.publish(msg)
        self.get_logger().info(f'Move2target numbers are published: "{msg.data}"')
        #TODO: gitmek için plan yap
        self.cell_value_list = list(map(int, self.cell_value_list))

        result = plan_path(self.cell_value_list, self.robot_location, self.selected_target, 15) # TODO: süreye göre adım dönüşler daha uzun sürer aslında
        print("Maximum Points:", result[0])
        print("Best Path:", result[1])


class Window(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.init_vars()

        self.pushButton.clicked.connect(self.textHandler)
        for i in range(1,len(self.buttons)+1): self.buttons[i].clicked.connect(self.clickHandler)
        for button in self.radioButtons: button.clicked.connect(self.radioHandler)
            
    def sendPico(self, message:str):
        client.sendall(message.encode()) 
        response = client.recv(1024)
        print("Received from Pico:", response.decode())
        return response.decode()

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
        print(f'{button} clicked!')
        publisher_node.game_control_string = str(button)
        publisher_node.publish_game_control()

    def clickHandler(self):
        self.buttonColor(self.sender(),'green')
        buttonName = self.sender().objectName()
        #print(f'{buttonName} clicked!')

        if publisher_node.selected_target == 0: # if target location is not set yet:
            publisher_node.selected_target = buttonName[11:]

    def textHandler(self):
        text = self.textEdit.toPlainText()
        print(f'Text sent: {text}')
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
                    number = random.choice(cellValues)
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
            self.sendPico(message)  

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
    client.close()

# TODO: total_point topic'inden gelen bilgiyi ekranda göstermek için yeni label eklenmeli
# TODO: plan ve alınması planlanan puan topic'e yayınlandıktan sonra onu al göster, 
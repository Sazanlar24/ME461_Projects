import cv2
import numpy as np
import math
import threading

import time, socket
import rclpy, random
from rclpy.node import Node
from std_msgs.msg import String, Int16

from project_pkg.planner import plan_path
from project_pkg.new_planner import find_max_points

HOST = '192.168.4.1'
PORT = 8080

R = 5
C = 8

class ROS_Image_Processing(Node):
    def __init__(self):
        super().__init__('image_processing')  # Name of the node 
        self.publisher_my_position  = self.create_publisher(String, 'my_position', 10)
        self.publisher_my_target    = self.create_publisher(String, 'my_target', 10)  
        self.publisher_total_points = self.create_publisher(Int16, 'total_points', 10) 
        self.publisher_expected_path_and_score = self.create_publisher(String, 'expected_path_and_score', 10) 
  
        self.robot_location  = 40  #şu anki konum            # TODO: ANLIK DEĞİŞMELİ
        self.target_location = 0  #bir sonraki target cell  # TODO: ANLIK DEĞİŞMELİ

        self.timer_my_position  = self.create_timer(1.0, self.publish_my_position)
        self.timer_my_target    = self.create_timer(1.0, self.publish_my_target)
        self.timer_total_points = self.create_timer(1.0, self.publish_total_points)  # Add timer for total_points
        self.timer_game_controller = self.create_timer(1.0, self.game_controller)
        
        self.counter_my_position  = 0 
        self.counter_my_target = 0 
        self.counter_total_points = 0 
        self.counter_game_controller = 0

        self.group_name = "sazanlar"
        self.cell_values = [] 
        self.total_points = 0

        self.total_seconds = 0
        self.selected_target = 0

        self.plan_string = ""
        self.new_path = []
        self.expected_score = 0
        self.new_path_is_found = False

        self.target_received = False

        self.remaining_time = 0
        
        self.robot_state = None
        self.robot_thread = None  # Thread for handling robot movement

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

        """
        try:
            # Connect to the Pico W server
            self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client.connect((HOST, PORT))
            print(f"Connected to {HOST}:{PORT}")
        except socket.error as e:
            print(f"Failed to connect to {HOST}:{PORT}. Error: {e}")
            exit(1)  # Exit the program if the connection fails"""    

    def sendPico(self, message:str):
        print(f'Message sent: {message}')
        # Validate the message before sending (optional, implement as needed)
        if not message.strip():
            print("Empty message. Please enter a valid command.")

        # Send data to the server
        while True:
            try:
                self.client.sendall(message.encode())
                print("Message sent to pico.")
                break
            except:
                print("Couldn't send message to pico")

    def publish_path_and_expected_points(self):
        msg = String()
        msg.data = self.plan_string + ": " + str(self.expected_score)
        self.publisher_expected_path_and_score.publish(msg)
        #self.get_logger().info(f'Found plan and score is published: "{msg.data}"')

    def publish_my_position(self):
        msg = String()
        msg.data = self.group_name + ", " + str(self.robot_location)
        self.publisher_my_position.publish(msg)
        #self.get_logger().info(f'My position is published: "{msg.data}"')
        self.counter_my_position += 1

    def publish_my_target(self):
        msg = String()
        msg.data = self.group_name + ", " + str(self.target_location)
        self.publisher_my_target.publish(msg)
        #self.get_logger().info(f'My target is published: "{msg.data}"')
        self.counter_my_target += 1

    def publish_total_points(self):
        msg = Int16()
        msg.data = self.total_points
        self.publisher_total_points.publish(msg)
        #self.get_logger().info(f'Total_points is published: "{msg.data}"')
        self.counter_total_points += 1

    def listener_callback_game_control(self, msg):
        new_state = msg.data.upper()
        print("New control string received:", new_state)

        if new_state in ["START", "PAUSE", "RESUME", "STOP"]:
            self.robot_state = new_state

            # Start movement thread only if it's a new START command
            if new_state == "START" and (self.robot_thread is None or not self.robot_thread.is_alive()):
                self.robot_thread = threading.Thread(target=self.run_robot_logic)
                self.robot_thread.start()

    def game_controller(self):
        if self.robot_state is None:
            print("Waiting for a game control command...")

    def run_robot_logic(self):
        """Runs robot logic in a separate thread, continuously checking robot_state."""
        while True:
            if self.robot_state == "START":
                print("Hedefe gidiyorum...")
                time.sleep(1)
                self.start_string()
            
            elif self.robot_state == "PAUSE":
                print("Durduruldum...")
                # TODO: STOP motors
                # TODO: take note of the remaining time
                # TODO: LEDs1 should blink RED
                time.sleep(1)
            
            elif self.robot_state == "RESUME":
                print("Devam ediyorum...")
                # initiate new timer_service with the remaining time
                time.sleep(1)
                self.start_string()
            
            elif self.robot_state == "STOP":
                print("Durdum bekliyorum....")
                # TODO: Motorları durdur
                # TODO: terminate tasks: remaining_time sıfırla, new_path sıfırla vs.
                # self.sendPico("PIX:1,0,red") # set LEDs1 solid RED
                break  # Exit the thread when STOP is received

            time.sleep(0.1)  # Small sleep to avoid excessive CPU usage

    def start_string(self):
        # TODO: LEDs1 turn into solid WHITE
        # self.sendPico("PIX:1,0,white")
        # timer_service should be initiated
        # robot should start moving.

        # start a countdown from self.total_seconds if start
        # if pause, continue from remaining time
        # send remaining time into a topic, GUI will receive it and show
        # if time is up, stop the robot

        i = 0
        while self.robot_state in ["START", "RESUME"]:
            
            print("while'in içindeyim")            

            while True:
                try:
                    next_cell = self.new_path[0]
                except:
                    print("yeni path gelmemiş...")

                print(f"aradaki açı farkına bakıyorum: {i}...")
                angle_threshold = 5
                aci_fark = 5 - 3
                if aci_fark > angle_threshold:
                    print("motorlara ccw mesajı gönderdim..")
                elif aci_fark < -angle_threshold:
                    print("motorlara cw mesajı gönderdim..")
                else:
                    print("bir sonraki cell için dönmeye gerek yok...")
                    print("motorlara durma mesajı gönderdim..")
                    break
        
            while True:

                distance_threshold = 10
                uzaklik_fark = 7 - 2

                if uzaklik_fark > distance_threshold:
                    print("motorlara düz gitme mesajı verdim..")
                else:
                    print("robot hedefe vardı..")
                    try:
                        self.new_path.pop(0)
                    except:
                        print("olmadi..")

                    print("bir sonraki hedefe geçiliyor...")
                    i+=1
                    break
            
            if len(self.new_path) == 0:
                print("target'a ulaşıldı")
                self.robot_state = "STOP"

    def listener_callback_cell_values(self, msg):
        #self.get_logger().info(f'Received on cell_values topic: "{msg.data}"')
        
        string = msg.data.replace(" ", "")  # erases empty characters
        list = string.split(",")        

        new_values = [int(x) for x in list]
        self.cell_values = new_values
        print(self.cell_values)

        self.grid = [self.cell_values[i * C:(i + 1) * C] for i in range(R)]
        
        print("New points list is arrived!")    

        # TODO: it should blink LEDs1 GREEN
        # check if it works
        """
        while not self.target_received:
            self.sendPico("PIX:1,1,green") # set LEDs1 green for 1 seconds
            time.sleep(2)                  # sleep 2 seconds
        """ 

    def listener_callback_move2target(self, msg):
        self.get_logger().info(f'Received on move2target topic: "{msg.data}"')
        self.target_received = True
        
        string = msg.data.replace(" ", "")  # erases empty characters
        seconds, target = string.split(",")
        
        # if new target is arrived
        if target != self.selected_target:
            self.total_seconds, self.selected_target = seconds, target
            self.selected_target = int(self.selected_target)
            self.new_path_is_found = False 

        # path is not known (either no target is arrived yet, or target is changed)
        if self.new_path_is_found is False:

            #TODO: During planning LEDs1 will be solid BLUE
            #self.sendPico("PIX:1,1,blue")

            try:
                result = find_max_points(self.grid, self.robot_location, self.selected_target)
                expected_score = result['score']
                found_path = result['path']

                #print(f"Maximum Points Collected: {expected_score}")
                #print("Path Taken:", found_path)
                
                if expected_score > 0:
                    self.expected_score = expected_score
                    self.new_path_is_found = True
                    plan_string = ""

                for point in found_path:
                    cell_number=point[0]*C+point[1]+1
                    plan_string += str(cell_number)
                    plan_string += ","

                # Remove the trailing comma
                self.plan_string = plan_string.rstrip(',')
                
                # list that contains tuple representation of the path
                self.new_path = self.plan_string.split(",")
                self.new_path = list(map(int, self.new_path))
                
                print(f"Plan string: {self.plan_string}")
                #print("plan is done")

                #print(self.plan_string)
                #print(self.expected_score)

                # TODO: check if it works
                """
                while self.robot_state != "START":
                    self.sendPico("PIX:1,0.5,green") # set LEDs1 green for 0.5 seconds
                    time.sleep(1)                    # sleep 1 seconds
                """

            except:
                print("Some variables are missing or plan cannot be found: (cell_values, robot_location, selected_target)")

            publisher_node.publish_path_and_expected_points()

            """"
            TODO: After planning, LEDs1 will blink GREEN but faster
            """

class GridProcessor:
    def __init__(self, video_path):
        self.video_path = video_path

    def process_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply binary thresholding with threshold value 100
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)

        # Divide the image into connected components and eliminate the largest one
        image = binary.astype('uint8')
        nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(image, connectivity=4)
        sizes = stats[:, -1]

        # Ensure there's more than one component before accessing sizes[1]
        if nb_components > 1:
            max_label = 1
            max_size = sizes[1]
            for i in range(2, nb_components):
                if sizes[i] > max_size:
                    max_label = i
                    max_size = sizes[i]
            img2 = np.zeros(output.shape)
            img2[output == max_label] = 255
            image = img2
        else:
            image = np.zeros_like(output)

        # Negative binary
        image = image.astype("uint8")
        image = cv2.bitwise_not(image)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(image, connectivity=8)

        # Define area threshold
        min_area = 500  # Replace with your desired value
        max_area = 5000

        # Create an output image initialized to zero (black)
        output = np.zeros_like(binary)

        # Loop through each connected component
        cell_count = 0
        areas = []
        for i in range(1, num_labels):  # Start from 1 to skip the background
            area = stats[i, cv2.CC_STAT_AREA]
            if max_area >= area >= min_area:
                # Retain components that meet the area requirement
                output[labels == i] = 255
                cell_count += 1
                areas.append(area)

        image = output

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(image, connectivity=8)
        # Loop through each connected component (excluding the background)
        for i in range(1, num_labels):  # Start from 1 to exclude the background
            cx, cy = centroids[i]
            # print(f"Component {i}: Centroid at ({cx:.2f}, {cy:.2f})")
        
        #if cell_count == 40: print(centroids[1:])
            
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)  # Convert to color for visualization

        for i in range(1, num_labels):  # Skip the background
            cx, cy = centroids[i]
            cv2.circle(image, (int(cx), int(cy)), 5, (0, 0, 255), -1)
            cv2.putText(image, str(i), (int(cx), int(cy)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

        return (image, centroids[1:])

    def gridSort(self, centroids:np.array):
        sortedGrid = []
        sorted_by_y = sorted(centroids, key=lambda x: x[1])
        for i in range(5):
            row = sorted_by_y[i*8:i*8+8]
            row_sorted = sorted(row, key=lambda x: x[0])
            for item in row_sorted:
                sortedGrid.append(item)
        return sortedGrid
    
    def process_video(self):
        cap = cv2.VideoCapture(self.video_path)

        if not cap.isOpened():
            print("Error: Cannot open video.")
            return

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Process each frame
            processed_frame = self.process_frame(frame)[0]

            # Display the processed frame and the detected objects frame
            cv2.imshow('Processed Frame', processed_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            if cv2.waitKey(1) & 0xFF == ord('o'):
                print("hadii")
                centroids = self.process_frame(frame)[1]
                sortedCentroids = self.gridSort(centroids)
                print(f'Centroid of grid cells detected: {centroids}')
                print(f'Centroids are sorted in grid order: {sortedCentroids}')
                return sortedCentroids
                
        cap.release()
        cv2.destroyAllWindows()

class ArucoMarkerDetector:
    def __init__(self, video_path, centroids, marker_size=0.15, camera_matrix=None, dist_coeffs=None):
        # Load the dictionary used to generate markers
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.marker_size = marker_size
        self.video_path = video_path
        self.pos_x =  None
        self.pos_y = None
        self.orient = None
        self.centroids = centroids

        # Default camera calibration parameters if none are provided
        self.camera_matrix = camera_matrix if camera_matrix is not None else np.array([
            [800.0, 0.0, 320.0],
            [0.0, 800.0, 240.0],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = dist_coeffs if dist_coeffs is not None else np.array([0.0, 0.0, 0.0, 0.0])

        # Initialize video capture
        self.cap = cv2.VideoCapture(self.video_path)

    def measureAngle(self, robotLoc:tuple, cellNumer:int, robotOrient:float):
        targetx, targety = self.centroids[cellNumer][0], self.centroids[cellNumer][1]
        robotx, roboty = robotLoc[0], robotLoc[1]

        targetRespecttoRobot = (targetx- robotx, targety-roboty)
        angle_degrees = math.degrees(math.atan2(targetRespecttoRobot[1], targetRespecttoRobot[0]))

        turnAngle = angle_degrees - robotOrient
        
        print("location ile gidilmesi arasındaki açı: ", angle_degrees)
        print("dönülmesi gereken açı: ", turnAngle)

    def detect_markers(self, frame):
        # Convert frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Detect markers in the image
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        if len(corners) != 0: 
            (tl, tr, br, bl) = corners[0][0][0],corners[0][0][1],corners[0][0][2],corners[0][0][3]
            self.pos_x = (tl[0] + br[0]) / 2
            self.pos_y = (tl[1] + br[1]) / 2
        return corners, ids

    def estimate_pose(self, corners):
        # Estimate pose of each marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.camera_matrix, self.dist_coeffs
        )
        return rvecs, tvecs

    def draw_markers_and_axes(self, frame, corners, ids, rvecs, tvecs):
        if ids is not None:
            # Draw detected markers and pose axes
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for i in range(len(ids)):
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.03)
                # Display translation and rotation vectors       
                #self.orient = rvecs[i].flatten()

                # Convert rotation vector to rotation matrix
                rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
                # Calculate yaw (rotation around z-axis)
                yaw_angle = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
                # Convert to degrees for better interpretation
                yaw_angle_deg = np.degrees(yaw_angle)
                # Display the result
                #print(f"Marker ID {ids[i][0]} -> Yaw (Z Angle): {yaw_angle_deg:.2f} degrees")
                # Store the yaw angle
                self.orient = yaw_angle_deg

                position_str = f"Position: x={self.pos_x:.2f}, y={self.pos_y:.2f}, z={tvecs[i][0][2]:.2f}"
                rotation_str = f"Orientation: rvecs={self.orient}"

                cv2.putText(frame, position_str, (10, 30 + i * 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, rotation_str, (10, 60 + i * 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    def closest_point(self, threshold):
        # Filter points within the threshold distance
        filtered_points = []
        for point in self.centroids:
            dist = math.dist((self.pos_x, self.pos_y), point)
            if dist <= threshold: filtered_points.append(point)
        
        closestCellCoordinates = min(filtered_points, key=lambda point: math.dist((self.pos_x, self.pos_y), point), default=None)
        closestCell = None
        for index, point in enumerate(self.centroids):
            if closestCellCoordinates is point:
                closestCell = index + 1
            
        return closestCellCoordinates, closestCell


    def run(self):
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                break

            corners, ids = self.detect_markers(frame)
            if ids is not None:
                rvecs, tvecs = self.estimate_pose(corners)
                self.draw_markers_and_axes(frame, corners, ids, rvecs, tvecs)
                threshold = 30
                nearestCellCoordinates, nearestCell = self.closest_point(threshold)
                print(f"Closest point within threshold is 'Cell {nearestCell}' with coordinates of {nearestCellCoordinates}")
                publisher_node.robot_location = nearestCell
                self.measureAngle((self.pos_x, self.pos_y), 2, self.orient)
                cv2.circle(frame, (int(self.centroids[5][0]), int(self.centroids[5][1])), 2, (0, 0, 255), -1)

            # Display the frame
            cv2.imshow('Aruco Marker Detection', frame)

            # Exit on pressing 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Release resources
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    global publisher_node

    video_path = 0 
    #processor = GridProcessor(video_path)
    #centroids = processor.process_video()
    centroids = [np.array([ 59.80806916, 202.8389049 ]), np.array([132.2450614 , 202.18606514]), np.array([204.14507   , 201.88501638]), np.array([269.38349984, 201.26941669]), np.array([336.68097811, 201.0864373 ]), np.array([410.01501743, 200.53177796]), np.array([482.49071537, 200.02465753]), np.array([538.53244722, 210.82799062]), np.array([ 60.23868097, 263.06385763]), np.array([132.74440433, 263.00697954]), np.array([204.77792805, 262.29997295]), np.array([269.97419929, 261.97864769]), np.array([336.94712526, 261.60600616]), np.array([410.91293774, 261.52577821]), np.array([482.58253227, 260.84152705]), np.array([547.62198221, 260.58132147]), np.array([ 60.99275992, 323.86562409]), np.array([133.17318289, 323.45814747]), np.array([204.97468354, 323.476597  ]), np.array([270.48180637, 323.01721897]), np.array([337.54642656, 322.57512662]), np.array([410.88862683, 322.40277778]), np.array([483.40492102, 321.97387606]), np.array([547.869761  , 321.70904053]), np.array([ 61.34394193, 383.08961474]), np.array([133.51786626, 383.49974477]), np.array([204.9108826 , 383.25478435]), np.array([270.51664612, 382.53390875]), np.array([337.51625135, 382.52979415]), np.array([411.05522884, 382.1920225 ]), np.array([483.05790838, 381.91961971]), np.array([547.3708166 , 381.06392236]), np.array([ 62.3846382 , 441.58155397]), np.array([134.6241573, 442.0005618]), np.array([205.81843318, 442.54807988]), np.array([270.96521445, 442.35123269]), np.array([337.73476703, 441.90979689]), np.array([411.41468802, 441.4445058 ]), np.array([482.73684211, 440.77725564]), np.array([547.07637655, 439.90763766])]
    detector = ArucoMarkerDetector(video_path, centroids)
    
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create the ROS2 publisher node
    publisher_node = ROS_Image_Processing()

    # Start a thread to spin the ROS2 node
    def ros_spin():
        executor.spin()
    
    # Use a MultiThreadedExecutor to allow concurrent message handling
    from rclpy.executors import MultiThreadedExecutor

    executor = MultiThreadedExecutor()
    executor.add_node(publisher_node)

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    detector.run()

    rclpy.shutdown()

    # Shut down the ROS2 node and clean up
    publisher_node.destroy_node()
    rclpy.shutdown()
    publisher_node.client.close()
    print("pico connection is stopped")
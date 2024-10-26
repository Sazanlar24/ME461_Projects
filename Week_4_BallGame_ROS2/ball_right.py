import cv2
import mediapipe as mp
import numpy as np
import random
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class ScoreRight(Node):
    def __init__(self):
        super().__init__('score_node_at_right')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'points', 10)
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'points',
            self.listener_callback,
            10
        )
        self.left_score  = 0
        self.right_score = 0
    
    def listener_callback(self, msg):
        left, right = msg.data
        if left != self.left_score:
            self.left_score = left
        if right != self.right_score:
            self.right_score = right
        
    def publish_point(self, score_left, score_right):
        msg = Int32MultiArray()
        msg.data = [score_left, score_right]
        self.publisher_.publish(msg)

# ROS2 node for publishing ball position in the right screen (integer values)
class BallPositionPublisher(Node):
    def __init__(self):
        super().__init__('ball_data_publisher_right')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'ball_data_right', 10)

    def publish_position(self, x, y, vx, vy):
        msg = Int32MultiArray()
        msg.data = [int(x), int(y), int(vx), int(vy)]
        self.publisher_.publish(msg)
        #self.get_logger().info(f'Right published: ball is at pos: x={int(x)}, y={int(y)} in the right screen')

# Node for receiving ball position from the left screen
class BallPositionSubscriber(Node):
    def __init__(self):
        super().__init__('ball_data_subscriber_right')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'ball_data_left',
            self.listener_callback,
            10
        )
        self.ball_position_at_left = (0, 0)
        self.ball_velocity_at_left = (0, 0)

    def listener_callback(self, msg):
        x, y, vx, vy = msg.data
        self.ball_position_at_left = (int(x),  int(y))
        self.ball_velocity_at_left = (int(vx), int(vy))
        #self.get_logger().info(f'Right received: ball is at pos: x={int(x)}, y={int(y)} in the left screen')

# Mediapipe hands model
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1)

# Initial ball coordinates and speed
ball_x, ball_y = 200, 200
ball_radius = 20
ball_speed_x, ball_speed_y = 5, 0

# Initial score
score_left = 0
score_right = 0
max_line_length = 100

# Function to reflect ball direction
def reflect_ball(ball_speed_x, ball_speed_y, normal_x, normal_y):
    normal_length = math.sqrt(normal_x ** 2 + normal_y ** 2)
    if normal_length == 0:
        return ball_speed_x, ball_speed_y
    normal_x /= normal_length
    normal_y /= normal_length
    dot_product = ball_speed_x * normal_x + ball_speed_y * normal_y
    reflect_x = ball_speed_x - 2 * dot_product * normal_x
    reflect_y = ball_speed_y - 2 * dot_product * normal_y
    return reflect_x, reflect_y

# ROS2 initialization
rclpy.init(args=None)
score_node = ScoreRight()
publisher_node = BallPositionPublisher()
receiver_node = BallPositionSubscriber()

# Open video capture from the laptop camera
cap = cv2.VideoCapture(0)  # 0 is usually the default camera index
if not cap.isOpened():
    print("Error: Could not open video capture.")
    rclpy.shutdown()
    exit()

ball_here = True
frame_count = 0

while rclpy.ok():

    rclpy.spin_once(score_node, timeout_sec=0.01)
    rclpy.spin_once(receiver_node, timeout_sec=0.01)
    
    # Capture frame from the laptop camera
    ret, img = cap.read()
    if not ret:
        print("Error: Could not read frame from camera.")
        break

    img = cv2.flip(img, 1)  # Flip the image horizontally
    h, w, c = img.shape

    score_left, score_right = score_node.left_score, score_node.right_score
    cv2.putText(img, f'Score Right: {score_right}', (w - 400, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    # Publish ball position to ROS2 topic
    publisher_node.publish_position(ball_x, ball_y, ball_speed_x, ball_speed_y)

    if ball_here:
        # Move the ball
        ball_x += ball_speed_x
        ball_y += ball_speed_y

        # Ball bounces on top and bottom walls
        if ball_y - ball_radius <= 0 or ball_y + ball_radius >= h:
            ball_speed_y = -ball_speed_y

        if (ball_x - ball_radius <= 0) and ball_speed_x < 0 and ball_here:
            print("ball is sent to left part")
            ball_x = -50
            ball_y = -50
            ball_here = False

        if ball_x + ball_radius >= w:
            score_left += 1
            ball_here = True
            ball_x, ball_y = w/2, h/2 # ekranın ortası
            ball_speed_x = 5
            ball_speed_y = 0
            score_node.publish_point(score_left, score_right)

        cv2.circle(img, (int(ball_x), int(ball_y)), ball_radius, (0, 255, 0), -1)

    else:
        # Receive ball position from the left screen
        read_x, read_y   = receiver_node.ball_position_at_left
        read_vx, read_vy = receiver_node.ball_velocity_at_left
        print(read_x)
        if read_x > w - ball_radius - 5 and read_vx > 0 and not ball_here:
            print("Ball coming from the left side")
            ball_x = 0
            ball_y = read_y
        
            ball_speed_x = read_vx
            ball_speed_y = read_vy

            ball_here = True

    # Hand tracking
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img_rgb = cv2.resize(img_rgb, (int(w * 0.5), int(h * 0.5)))  # Scale down for faster processing # TODO
    results = hands.process(img_rgb)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            index_finger_tip = hand_landmarks.landmark[8]
            thumb_tip = hand_landmarks.landmark[4]

            index_x, index_y = int(index_finger_tip.x * w), int(index_finger_tip.y * h)
            thumb_x, thumb_y = int(thumb_tip.x * w), int(thumb_tip.y * h)

            line_length = math.sqrt((thumb_x - index_x) ** 2 + (thumb_y - index_y) ** 2)

            if line_length > max_line_length:
                mid_x = (index_x + thumb_x) // 2
                mid_y = (index_y + thumb_y) // 2
                delta_x = thumb_x - index_x
                delta_y = thumb_y - index_y
                angle = math.atan2(delta_y, delta_x)

                index_x = int(mid_x - (max_line_length / 2) * math.cos(angle))
                index_y = int(mid_y - (max_line_length / 2) * math.sin(angle))
                thumb_x = int(mid_x + (max_line_length / 2) * math.cos(angle))
                thumb_y = int(mid_y + (max_line_length / 2) * math.sin(angle))

            cv2.line(img, (index_x, index_y), (thumb_x, thumb_y), (255, 0, 0), 3)
            cv2.circle(img, (index_x, index_y), 10, (255, 0, 0), -1)
            cv2.circle(img, (thumb_x, thumb_y), 10, (0, 0, 255), -1)

            if line_length > 0:
                distance = abs((thumb_y - index_y) * ball_x - (thumb_x - index_x) * ball_y + thumb_x * index_y - thumb_y * index_x) / line_length
                if distance <= ball_radius and ball_here:
                    normal_x = thumb_y - index_y
                    normal_y = index_x - thumb_x
                    ball_speed_x, ball_speed_y = reflect_ball(ball_speed_x, ball_speed_y, normal_x, normal_y)

    cv2.imshow("Right Game", img)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Limit buffer size to 1 frame
    cap.set(cv2.CAP_PROP_FPS, 30)  # Set FPS to 30, or higher if your camera supports it

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and destroy all OpenCV windows
cap.release()
cv2.destroyAllWindows()

# Shutdown ROS2
score_node.destroy_node()
publisher_node.destroy_node()
receiver_node.destroy_node()
rclpy.shutdown()

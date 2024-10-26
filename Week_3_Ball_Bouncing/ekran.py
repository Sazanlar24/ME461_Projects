import mediapipe.python.solutions.hands as mp_hands
import mediapipe.python.solutions.drawing_utils as drawing
import mediapipe.python.solutions.drawing_styles as drawing_styles

import cv2 as cv
import pyautogui
import time
import numpy as np

from ball import Ball
from detection import Hand
from target import Target

HEIGHT_MAX = 1060
WIDTH_MAX = 1900

class Ekran:

    def __init__(self, is_fullscreen):
        walls_points = []
        basket_points = []
        # Get the screen size (width, height)
        screen_width, screen_height = pyautogui.size()

        # Open the camera
        cam = cv.VideoCapture(0)

        self.myBall = Ball()
        handObject = Hand()
        self.target = Target()

        if is_fullscreen:
            # Create a window first
            cv.namedWindow("Show Video", cv.WND_PROP_FULLSCREEN)

            # Set the window to fullscreen mode
            cv.setWindowProperty("Show Video", cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)

        previous_time = time.time()  # Initialize previous time for frame timing
        bounce_time = 1

        while cam.isOpened():
            # Read a frame from the camera
            success, self.frame = cam.read()

            # If the frame is not available, skip this iteration
            if not success:
                print("Camera Frame not available")
                continue

            # Convert frame from BGR to RGB for MediaPipe processing
            rgb_frame = cv.cvtColor(self.frame, cv.COLOR_BGR2RGB)

            # Process the frame for hand detection and tracking
            hands_detected = handObject.hands.process(rgb_frame)

            # Calculate time elapsed (in seconds) since the last frame
            current_time = time.time()
            time_step = current_time - previous_time
            previous_time = current_time  # Update previous time

            bounce_time += time_step
            
            # Flip the frame horizontally (mirror effect)
            mirrored_frame = cv.flip(self.frame, 1)

            # Resize the mirrored frame to fit the screen dimensions
            resized_frame = cv.resize(mirrored_frame, (screen_width, screen_height))
            
            # If hands are detected, draw landmarks and connections on the frame
            if hands_detected.multi_hand_landmarks:
                hand_position = []
                for hand_landmarks in hands_detected.multi_hand_landmarks:
                    cxsum = 0 
                    cysum = 0
                    c = 0
                    
                    for landmark in hand_landmarks.landmark:
                        # Mirror the x-coordinate
                        landmark.x = 1 - landmark.x

                        cx, cy = landmark.x * screen_width, landmark.y*screen_height
                        cxsum += cx
                        cysum += cy
                        c += 1 

                    #drawing.draw_landmarks(
                    #    resized_frame,  # Draw on the resized frame
                    #    hand_landmarks,
                    #    mp_hands.HAND_CONNECTIONS,
                    #    drawing_styles.get_default_hand_landmarks_style(),
                    #    drawing_styles.get_default_hand_connections_style(),
                    #)
                    
                    #print('average points', cxsum/c, cysum/c)
                    self.draw_ball(resized_frame, int(cxsum/c), int(cysum/c), int(self.myBall.radius/4))
                    hand_position.append((int(cxsum/c), int(cysum/c)))
                
                # Draw line btw hands and get points inside line
                try: 
                    dist = np.linalg.norm(np.array(hand_position[0]) - np.array(hand_position[1]))
                    if dist < screen_width/4 : 
                        self.draw_line(resized_frame, hand_position[0], hand_position[1])
                        line_points_x = np.linspace(hand_position[0][0], hand_position[1][0], 100)
                        line_points_y = np.linspace(hand_position[0][1], hand_position[1][1], 100)
                        line_points = np.stack((line_points_x, line_points_y), axis=1)
                        line_points = line_points.astype(int)

                        if bounce_time >= 1:
                            # Check for collision between ball and line segments
                            for i in range(len(line_points) - 1):
                                distance = np.linalg.norm(np.array((self.myBall.pos_x, self.myBall.pos_y)) - np.array(line_points[i]))
                                if distance <= self.myBall.radius:
                                    #print(f"Collision detected at point {line_points[i]}")
                                    self.bounce_ball_from_line(line_points[99], line_points[0])
                                    bounce_time = 0
                                    break

                    
                except: None

            # Display score
            cv.putText(resized_frame, f"Score: {self.target.score}", (int(WIDTH_MAX/2)-50, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)   

            # Draw walls and basket 
            self.draw_wall(resized_frame, (0, 0), (0,screen_height))
            self.draw_wall(resized_frame, (0,screen_height), (screen_width, screen_height))
            self.draw_wall(resized_frame, (screen_width,screen_height), (screen_width, 0))
            self.draw_wall(resized_frame, (screen_width, 0), (0, 0))
            
            # check collision
            self.target.checkCollision(self.myBall.pos_x, self.myBall.pos_y, self.myBall.radius)

            # Draw target
            cv.circle(resized_frame, (self.target.pos_x, self.target.pos_y), self.target.radius, color=(0, 255, 255), thickness=-1)  # Draw ball

            # Get walls and basket points for further collision
            walls_points = list(walls_points)
            if walls_points == []:
                walls_points += tuple(((0, item)) for item in np.linspace(0, screen_height, int(screen_height/100)))
                walls_points += tuple(((item, screen_height)) for item in np.linspace(0,screen_width, int(screen_width/100)))
                walls_points += tuple(((screen_width, item)) for item in np.linspace(screen_height, 0, int(screen_height/100)))
                walls_points += tuple(((item, 0)) for item in np.linspace(screen_width, 0, int(screen_width/100)))
                
                basket_points += tuple(((10, item)) for item in np.linspace(0.1*screen_height, 0.2*screen_height, int(0.01*screen_height)))
                
            walls_points = np.array(walls_points)
            basket_points = np.array(basket_points)

            bottom_dist = self.myBall.pos_y
            if bottom_dist <= self.myBall.radius:
                self.myBall.velocity_y = -self.myBall.velocity_y
            left_dist = self.myBall.pos_x
            if left_dist <= self.myBall.radius:
                self.myBall.velocity_x = -self.myBall.velocity_x
            right_dist = screen_width - self.myBall.pos_x
            if right_dist <= self.myBall.radius:
                self.myBall.velocity_x = -self.myBall.velocity_x
            up_dist = screen_height - self.myBall.pos_y
            if up_dist <= self.myBall.radius:
                self.myBall.velocity_y = -self.myBall.velocity_y

            # Update the ball's position with the calculated time step
            self.myBall.calculateNewPos(time_step)
            
            # Get points around ball for further collision
            ball_points = np.array(self.myBall.points)

            # Draw ball on the resized frame
            self.draw_ball(resized_frame, int(self.myBall.pos_x), int(self.myBall.pos_y), int(self.myBall.radius))

            # Display the frame with the ball and hand landmarks
            cv.imshow("Show Video", resized_frame)

            # Exit the loop if 'q' key is pressed
            if cv.waitKey(20) & 0xff == ord('q'):
                break

        # Release the camera
        cam.release()
        cv.destroyAllWindows()  # Ensure the window is closed

    def bounce_ball_from_line(self, line_start, line_end):        
        # Calculate the normal vector of the line (perpendicular direction)
        line_vec = np.array(line_end) - np.array(line_start)
        normal_vec = np.array([-line_vec[1], line_vec[0]])  # Rotate line vector 90 degrees to get the normal
        
        # Normalize the normal vector
        normal_vec = normal_vec / np.linalg.norm(normal_vec)
        
        # Reflect the ball's velocity along the normal vector
        velocity_vec = np.array([self.myBall.velocity_x, self.myBall.velocity_y])
        dot_product = np.dot(velocity_vec, normal_vec)
        
        # Update the velocity of the ball based on the reflection
        self.myBall.velocity_x = velocity_vec[0] - 2 * dot_product * normal_vec[0]
        self.myBall.velocity_y = velocity_vec[1] - 2 * dot_product * normal_vec[1]

        # Move the ball slightly away from the line to prevent sticking
        #reflection_distance = 25  # You can adjust this value if needed
        #self.myBall.pos_x += reflection_distance * normal_vec[0]
        #self.myBall.pos_y += reflection_distance * normal_vec[1]

        # Debugging print to check the velocity and position after reflection
        #print(f"New velocity after bounce: ({self.myBall.vel_x}, {self.myBall.vel_y})")
        #print(f"New position after bounce: ({self.myBall.pos_x}, {self.myBall.pos_y})")

    def draw_ball(self, frame, ball_x, ball_y, radius):
        cv.circle(frame, (ball_x, ball_y), radius, color=(0, 0, 255), thickness=-1)  # Draw ball

    def draw_wall(self, frame, start_point, end_point): 
        cv.line(frame, start_point, end_point, (255, 0, 0), 10) 

    def draw_line(self, frame, start_point, end_point): 
        cv.line(frame, start_point, end_point, (0, 255, 0), 10) 

if __name__ == "__main__":
    # Initialize with fullscreen mode set to True or False
    is_fullscreen = True

    # Create an instance of Ekran to start the program
    ekran = Ekran(is_fullscreen)   
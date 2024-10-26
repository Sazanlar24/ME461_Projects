import cv2 as cv
import mediapipe.python.solutions.hands as mp_hands
import mediapipe.python.solutions.drawing_utils as drawing
import mediapipe.python.solutions.drawing_styles as drawing_styles
import random
import time

# Initialize the Hands model
hands = mp_hands.Hands(
    static_image_mode=False, 
    max_num_hands=2,           
    min_detection_confidence=0.5
)

# Open the camera
cam = cv.VideoCapture(0)
cam.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

# Screen size
screen_width = 1280
screen_height = 720

# Initializers 
start_time_global = 0
apples = []
score = 0
lives = 10
speed_constant = 10

# Function to generate random apple at the top
def create_apple(start):
    color = (random.choice([(0, 0, 255), (0, 255, 0)]), 0)
    return {"x": random.randint(0, screen_width), "y": 0, 't':start, 'color':color}

# Function to move apples down
def move_apples():
    global lives  # Access the global lives variable
    global speed_constant
    
    for apple in apples:
        # Calculate elapsed time
        elapsed_time = time.time() - apple['t']

        # Increase apple speed over time
        apple_speed = int(speed_constant*elapsed_time)  # Start speed + time elapsed, capped at max_speed
        
        apple["y"] += apple_speed
        # Check if the apple has reached the bottom
        if apple["y"] > screen_height:
            apples.remove(apple)  # Remove the apple from the list
            if apple['color'][0] == (0,255,0):
                lives -= 1  # Reduce lives
            
            if lives <= 0:     
                over()    

# Function to check if apple is caught by hand and color is red
def check_catch(apple, hand_landmarks):
    hand_size = 100  # Size of the hand bounding box for simplicity
    for landmark in hand_landmarks.landmark:
        hand_x = int(landmark.x * screen_width)
        hand_y = int(landmark.y * screen_height)
        if (hand_x - hand_size // 2 <= apple["x"] <= hand_x + hand_size // 2) and \
           (hand_y - hand_size // 2 <= apple["y"] <= hand_y + hand_size // 2) :
            return True
    return False

def over():
    global score
    global lives
    print("Game Over!", 'Score:', score) 
    # Release the camera
    cam.release()
    cv.destroyAllWindows()

while cam.isOpened():
    success, fl = cam.read()
    if not success:
        print("Camera Frame not available")
        continue
    
    # Change color of the apple
    if random.randint(0, 20) == 4:
        apple = random.choice(apples)
        if apple['color'][1] == 0 and apple['y'] <= screen_height/2:
            if apple['color'][0] == (0, 0, 255) : 
                apple['color'] = ((0, 255, 0), 1)
            else: 
                apple['color'] = ((0, 0, 255), 1)
            print('Color Changed!')


    frame = cv.flip(fl,1)
    frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
    hands_detected = hands.process(frame)
    frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)

    # Capture total time 
    if start_time_global == 0: 
        start_time_global = time.time()
    time_seconds = time.time() - start_time_global

    # Create a new apple every n seconds
    if int(time_seconds*20)%7 == 0 :
        start = time.time()
        apples.append(create_apple(start))

    # Detect hand positions
    if hands_detected.multi_hand_landmarks:
        for hand_landmarks in hands_detected.multi_hand_landmarks:
            # Draw hand landmarks
            drawing.draw_landmarks(
                frame,
                hand_landmarks,
                mp_hands.HAND_CONNECTIONS,
                drawing_styles.get_default_hand_landmarks_style(),
                drawing_styles.get_default_hand_connections_style(),
            )

            # Check for catching apples 
            apples_to_remove = []
            for apple in apples:
                if check_catch(apple, hand_landmarks):
                    apples_to_remove.append(apple)
                    # Check if the apple is red or green
                    if apple['color'][0] == (0, 255, 0):
                        score += 1
                    else: lives -= 1

            # Remove caught apples
            apples = [apple for apple in apples if apple not in apples_to_remove]


    # Move apples down
    move_apples()

    # Display lives in the top left corner
    cv.putText(frame, f"Lives: {lives}", (10, 60), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    # Draw apples
    for apple in apples:
        cv.circle(frame, (apple["x"], apple["y"]), 20, apple['color'][0], -1)  # Draw apple

    # Display score
    cv.putText(frame, f"Score: {score}", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    # Display the frame with annotations
    cv.imshow("Apple Catching Game", frame)

    if lives <= 0 or cv.waitKey(20) & 0xff == ord('q'):
        break

over() 
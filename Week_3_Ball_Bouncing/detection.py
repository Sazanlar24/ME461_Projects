import cv2 as cv
import mediapipe.python.solutions.hands as mp_hands
import mediapipe.python.solutions.drawing_utils as drawing
import mediapipe.python.solutions.drawing_styles as drawing_styles

class Hand:
    def __init__(self): 
        
        # Initialize the Hands model
        self.hands = mp_hands.Hands(
            static_image_mode=False,  # Set to False for processing video frames
            max_num_hands=2,           # Maximum number of hands to detect
            min_detection_confidence=0.5  # Minimum confidence threshold for hand detection
        )
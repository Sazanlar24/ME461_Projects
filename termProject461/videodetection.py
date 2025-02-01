import cv2
import numpy as np
import math
from time import sleep


class GridProcessor:
    def __init__(self, video_path):
        self.video_path = video_path

    def process_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply binary thresholding with threshold value 100
        _, binary = cv2.threshold(gray, 90, 255, cv2.THRESH_BINARY_INV)

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
        self.centroidDict = {}
        for i in range(len(centroids)): self.centroidsDict[i+1] = centroids[i]

        # Default camera calibration parameters if none are provided
        self.camera_matrix = camera_matrix if camera_matrix is not None else np.array([
            [800.0, 0.0, 320.0],
            [0.0, 800.0, 240.0],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = dist_coeffs if dist_coeffs is not None else np.array([0.0, 0.0, 0.0, 0.0])

        # Initialize video capture
        self.cap = cv2.VideoCapture(self.video_path)

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
                self.orient = rvecs[i].flatten()
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

    def measureAngle(robotLoc:tuple, target:tuple, robotOrient:float):
        targetx, targety = target[0], target[1]
        robotx, roboty = robotLoc[0], robotLoc[1]

        targetRespecttoRobot = (targetx-robotx, targety-roboty)
        angle_degrees = math.degrees(math.atan2(-targetRespecttoRobot[1], targetRespecttoRobot[0]))
        angle_degrees = angle_degrees % 360 if angle_degrees >= 0 else (angle_degrees % 360 + 360) % 360

        turnAngle = angle_degrees - robotOrient
        
        return targetRespecttoRobot, angle_degrees, turnAngle
    
    def run(self):
        while self.cap.isOpened():
            sleep(0.1)
            ret, frame = self.cap.read()
            if not ret:
                break

            corners, ids = self.detect_markers(frame)
            if ids is not None:
                rvecs, tvecs = self.estimate_pose(corners)
                self.draw_markers_and_axes(frame, corners, ids, rvecs, tvecs)
                threshold = 20
                nearestCellCoordinates, nearestCell = self.closest_point(threshold)
                print(f"Closest point within threshold is 'Cell {nearestCell}' with coordinates of {nearestCellCoordinates}")

                m = 10
                diffAngle = self.measureAngle((self.pos_x, self.pos_y),self.centroidsDict[m], self.orient)[2]
                print(diffAngle)
                
            # Display the frame
            cv2.imshow('Aruco Marker Detection', frame)

            # Exit on pressing 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Release resources
        self.cap.release()
        cv2.destroyAllWindows()

video_path = 0
processor = GridProcessor(video_path)
centroids = processor.process_video()
#centroids = [np.array([ 59.80806916, 202.8389049 ]), np.array([132.2450614 , 202.18606514]), np.array([204.14507   , 201.88501638]), np.array([269.38349984, 201.26941669]), np.array([336.68097811, 201.0864373 ]), np.array([410.01501743, 200.53177796]), np.array([482.49071537, 200.02465753]), np.array([538.53244722, 210.82799062]), np.array([ 60.23868097, 263.06385763]), np.array([132.74440433, 263.00697954]), np.array([204.77792805, 262.29997295]), np.array([269.97419929, 261.97864769]), np.array([336.94712526, 261.60600616]), np.array([410.91293774, 261.52577821]), np.array([482.58253227, 260.84152705]), np.array([547.62198221, 260.58132147]), np.array([ 60.99275992, 323.86562409]), np.array([133.17318289, 323.45814747]), np.array([204.97468354, 323.476597  ]), np.array([270.48180637, 323.01721897]), np.array([337.54642656, 322.57512662]), np.array([410.88862683, 322.40277778]), np.array([483.40492102, 321.97387606]), np.array([547.869761  , 321.70904053]), np.array([ 61.34394193, 383.08961474]), np.array([133.51786626, 383.49974477]), np.array([204.9108826 , 383.25478435]), np.array([270.51664612, 382.53390875]), np.array([337.51625135, 382.52979415]), np.array([411.05522884, 382.1920225 ]), np.array([483.05790838, 381.91961971]), np.array([547.3708166 , 381.06392236]), np.array([ 62.3846382 , 441.58155397]), np.array([134.6241573, 442.0005618]), np.array([205.81843318, 442.54807988]), np.array([270.96521445, 442.35123269]), np.array([337.73476703, 441.90979689]), np.array([411.41468802, 441.4445058 ]), np.array([482.73684211, 440.77725564]), np.array([547.07637655, 439.90763766])]
detector = ArucoMarkerDetector(video_path, centroids)
detector.run()
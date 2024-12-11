import cv2
import numpy as np

def detect_aruco_coordinates(image_path):
    """
    Detect ArUco markers in an image and return their pixel coordinates.
    
    Parameters:
    image_path (str): The path to the image file.
    
    Returns:
    dict: A dictionary where each key is a detected marker ID (int),
          and each value is a list of corner coordinates in the form:
          [(x1, y1), (x2, y2), (x3, y3), (x4, y4)].
          If no markers are detected, returns an empty dictionary.
    """
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"Could not read image at {image_path}")
    
    # Don't know what the argument would be but you can change here if it's not the file path to the image

    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Initialize the ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    
    # Initialize detector parameters with default values
    aruco_params = cv2.aruco.DetectorParameters()
    
    # Create detector
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    
    # Detect markers
    corners, ids, _ = detector.detectMarkers(gray)
    
    # If no markers are detected, return an empty dictionary
    if ids is None or len(ids) == 0:
        return {}
    
    # Build a dictionary of marker IDs to their corner coordinates
    marker_coordinates = {}
    for i, marker_id in enumerate(ids):
        # corners[i] is an array of shape (1,4,2).
        # Extract and convert to a list of tuples.
        corner_list = [(float(coord[0]), float(coord[1])) for coord in corners[i][0]]
        marker_coordinates[int(marker_id[0])] = corner_list
    
    return marker_coordinates

# Testing
if __name__ == "__main__":
    image_path = "IMG005.jpg"
    coordinates = detect_aruco_coordinates(image_path)
    print("Detected Marker Coordinates:", coordinates)
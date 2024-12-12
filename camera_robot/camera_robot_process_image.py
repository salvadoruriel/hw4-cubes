import cv2
import math
import numpy as np
import os


def read_aruco_marker(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    aruco_params = cv2.aruco.DetectorParameters()
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    markers = []
    if ids is not None:
        for i, marker_id in enumerate(ids):
            # Extract the corner points and compute the centroid
            corner = corners[i][0]
            centroid = np.mean(corner, axis=0).astype(int)
            markers.append((marker_id[0], centroid))
    return markers


def process_image(image, contour_area_threshold=2000):
    height, width = image.shape[:2]

    # Crop 5% from each side
    crop_x = int(width * 0.05)
    crop_y = int(height * 0.05)
    cropped_image = image[crop_y : height - crop_y, crop_x : width - crop_x]

    # Detect ArUco markers
    aruco_markers = read_aruco_marker(cropped_image)
    print(f"Number of ArUco markers detected: {len(aruco_markers)}")

    # Convert to grayscale
    gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply adaptive thresholding to handle different lighting conditions
    binary = cv2.adaptiveThreshold(
        blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2
    )

    # Apply morphological operations to remove small noise
    kernel = np.ones((3, 3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

    # Find contours
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    marker_to_contour = {}
    contour_assignments = {}

    # Numbers of contours detected
    print(f"Number of contours detected: {len(contours)}")

    # Numbers of contours detected above threshold
    contours_above_threshold = len(
        [c for c in contours if cv2.contourArea(c) > contour_area_threshold]
    )
    print(
        f"Number of contours above threshold of {contour_area_threshold}: {contours_above_threshold}"
    )

    for contour in contours:
        area = cv2.contourArea(contour)

        # Ignore small contours that may be noise
        if area < contour_area_threshold:
            continue

        adjusted_contour = contour + np.array([[crop_x, crop_y]])

        # Calculate the centroid of the contour
        M = cv2.moments(adjusted_contour)
        if M["m00"] != 0:
            contour_centroid = (
                int(M["m10"] / M["m00"]),
                int(M["m01"] / M["m00"]),
            )
        else:
            contour_centroid = (0, 0)

        # Convert contour points to float32 for PCA
        data_points = adjusted_contour.reshape(-1, 2).astype(np.float32)

        # Calculate angle using PCA
        mean, eigenvectors = cv2.PCACompute(data_points, mean=None)
        principal_axis = eigenvectors[0]  # Primary eigenvector
        angle = -np.degrees(np.arctan2(principal_axis[1], principal_axis[0]))

        # Get the minimum area rectangle
        rect = cv2.minAreaRect(adjusted_contour)
        box = cv2.boxPoints(rect).astype(int)

        # Find closest marker for each contour
        closest_marker = None
        min_distance = float("inf")
        for marker_id, marker_centroid in aruco_markers:
            dist = np.linalg.norm(
                np.array(marker_centroid) - np.array(contour_centroid)
            )
            if dist < min_distance:
                closest_marker = (marker_id, marker_centroid)
                min_distance = dist

        if closest_marker and closest_marker[0] not in contour_assignments:
            marker_to_contour[closest_marker[0]] = contour_centroid
            contour_assignments[closest_marker[0]] = (
                adjusted_contour,
                contour_centroid,
                rect,
                box,
                angle,
            )

    output = []
    for marker_id, (
        contour,
        new_centroid,
        rect,
        box,
        angle,
    ) in contour_assignments.items():
        # Draw contours
        cv2.drawContours(image, [contour], 0, (255, 0, 0), 2)

        # Draw bounding box
        cv2.drawContours(image, [box], 0, (0, 255, 0), 2)

        # Draw centroid
        cv2.circle(image, new_centroid, 5, (0, 0, 255), -1)

        # Calculate full-length line endpoints
        angle_radians = np.radians(angle)
        x1 = int(new_centroid[0] - width * np.cos(angle_radians))
        y1 = int(new_centroid[1] + width * np.sin(angle_radians))
        x2 = int(new_centroid[0] + width * np.cos(angle_radians))
        y2 = int(new_centroid[1] - width * np.sin(angle_radians))

        # Draw full-length line
        cv2.line(image, (x1, y1), (x2, y2), (0, 165, 255), 2)  # Orange color in BGR

        # Draw marker ID near the centroid
        text_position = (new_centroid[0] + 10, new_centroid[1] - 10)  # Slightly offset
        cv2.putText(
            image,
            f"ID: {marker_id}",
            text_position,
            cv2.FONT_HERSHEY_SIMPLEX,
            1,  # Font size
            (255, 51, 204),
            3,  # Thickness
            cv2.LINE_AA,
        )

        # Append the marker id, centroid, and angle
        output.append((marker_id, new_centroid, angle))

    # Display the image with all annotations
    cv2.imshow("Result", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return output


# Example usage

image_path = "example_images//IMG006.jpg"
contour_area_threshold = 2000

# id to object name dict
id_to_object = {0: "bowl", 1: "spoon", 2: "toy"}

image = cv2.imread(image_path)
output = process_image(image, contour_area_threshold)

print("Detected objects:")
for o in output:
    print(
        f"Object ID: {o[0]}, Object: {id_to_object[o[0]]}, Centroid: {o[1]}, Angle: {o[2]:.2f} degrees"
    )


# Output:
# marker_id, centroid, angle for each object detected in the image

# TODO for use in project:
# 1. copy process_image function and its helper functions
# 2. Figure out how to use the calculated angle to determine the angle for the gripper
# 3. Find a better gripping location for objects (e.g. using tape markers)

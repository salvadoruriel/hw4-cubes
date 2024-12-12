import cv2
import numpy as np


def read_aruco_markers(image):
    """Detects ArUco markers in the image and returns their IDs and locations."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    aruco_params = cv2.aruco.DetectorParameters()
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    markers = []
    if ids is not None:
        for i, marker_id in enumerate(ids):
            corner = corners[i][0]
            centroid = tuple(np.mean(corner, axis=0).astype(int))
            markers.append((marker_id[0], centroid))
    return markers


def is_point_in_rect(point, rect):
    """Checks if a point (x, y) is inside a rectangle (start_x, start_y, end_x, end_y)."""
    x, y = point
    start_x, start_y, end_x, end_y = rect
    return start_x <= x <= end_x and start_y <= y <= end_y


def draw_sections(image, sections, markers):
    """Draws sections, highlights the section containing markers, adds marker IDs and section numbers."""
    output_image = image.copy()

    marker_with_sections = []

    # Highlight sections and add marker IDs
    for marker_id, (x, y) in markers:
        for i, section in enumerate(sections):
            if is_point_in_rect((x, y), section["rect"]):
                # Highlight section
                overlay = output_image.copy()
                cv2.rectangle(
                    overlay, section["start"], section["end"], (0, 255, 255), -1
                )
                alpha = 0.3
                output_image = cv2.addWeighted(
                    overlay, alpha, output_image, 1 - alpha, 0
                )
                # Add marker ID
                text_position = (x + 10, y - 10)
                cv2.putText(
                    output_image,
                    f"ID: {marker_id}",
                    text_position,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
                marker_with_sections.append((i, marker_id, (x, y)))
                break

    # Draw section borders and section numbers
    for i, section in enumerate(sections):
        start, end = section["start"], section["end"]
        # Draw border lines
        cv2.rectangle(output_image, start, end, (0, 0, 255), 2)
        # Add section number in the top-left corner
        section_text_position = (start[0] + 10, start[1] + 20)
        cv2.putText(
            output_image,
            f"Section {i}",
            section_text_position,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    return output_image, marker_with_sections


def get_sections_horizontally(image, n=5):
    """Divides the image into n horizontal sections and returns the annotated image."""
    height, width = image.shape[:2]
    section_height = height // n
    sections = [
        {
            "start": (0, i * section_height),
            "end": (width, (i + 1) * section_height if i < n - 1 else height),
            "rect": (
                0,
                i * section_height,
                width,
                (i + 1) * section_height if i < n - 1 else height,
            ),
        }
        for i in range(n)
    ]

    markers = read_aruco_markers(image)
    output_image, marker_with_sections = draw_sections(image, sections, markers)
    return output_image, marker_with_sections


def get_sections_vertically(image, n=3):
    """Divides the image into n vertical sections and returns the annotated image."""
    height, width = image.shape[:2]
    section_width = width // n
    sections = [
        {
            "start": (i * section_width, 0),
            "end": ((i + 1) * section_width if i < n - 1 else width, height),
            "rect": (
                i * section_width,
                0,
                (i + 1) * section_width if i < n - 1 else width,
                height,
            ),
        }
        for i in range(n)
    ]

    markers = read_aruco_markers(image)
    output_image, marker_with_sections = draw_sections(image, sections, markers)
    return output_image, marker_with_sections


def get_sections_quad(image):
    """Divides the image into 4 quadrants and returns the annotated image."""
    height, width = image.shape[:2]
    mid_x, mid_y = width // 2, height // 2
    sections = [
        {
            "start": (0, 0),
            "end": (mid_x, mid_y),
            "rect": (0, 0, mid_x, mid_y),
        },  # Top-left
        {
            "start": (mid_x, 0),
            "end": (width, mid_y),
            "rect": (mid_x, 0, width, mid_y),
        },  # Top-right
        {
            "start": (0, mid_y),
            "end": (mid_x, height),
            "rect": (0, mid_y, mid_x, height),
        },  # Bottom-left
        {
            "start": (mid_x, mid_y),
            "end": (width, height),
            "rect": (mid_x, mid_y, width, height),
        },  # Bottom-right
    ]

    markers = read_aruco_markers(image)
    output_image, marker_with_sections = draw_sections(image, sections, markers)
    return output_image, marker_with_sections


# Example usage

# Load an image
image = cv2.imread("example_images/2024-12-11-131754.jpg")

# Horizontal sections
output_horizontal, markers_horizontal = get_sections_horizontally(image, n=5)
print("Markers in horizontal sections:", markers_horizontal)
cv2.imshow("Horizontal Sections", output_horizontal)
cv2.waitKey(0)

# Vertical sections
output_vertical, markers_vertical = get_sections_vertically(image, n=3)
print("Markers in vertical sections:", markers_vertical)
cv2.imshow("Vertical Sections", output_vertical)
cv2.waitKey(0)

# Quadrants
output_quad, markers_quad = get_sections_quad(image)
print("Markers in quadrants:", markers_quad)
cv2.imshow("Quadrants", output_quad)
cv2.waitKey(0)

# Clean up
cv2.destroyAllWindows()


# OUTPUT
# each section:
# (marker_id, (centroid_x, centroid_y), section_number)

# TODO in final project:
# - pick the a method
# - use section to determine the robot's rotation / facing the cat

import glob
import os
import cv2
import numpy as np 
from hw3utils import ( resizeWithAspectRatio, printNBL )

def main():
    CHECKERBOARD = (8, 6) #CORNERS, not squares
    #CHECKERBOARD = (9, 7) #outer
    #CHECKERBOARD = (7, 5) #inner #funky looking..
    #square_size = 1  # Set to square real-world units
    square_size = 24 #24.5mm #2.45cm

    #termination criteria: when to stop,, epsilon size and maximum iterations reached
    #   it will iterate 30 times and stop when the change is lesss than 0.001
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Make object to store values  X,Y,Z from the real world checkerboard' corners positions
    # # (0,0,0), (1,0,0), (2,0,0), ....,(6,6,0)
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= square_size  # Scale by the actual square size in real-world units

    objpoints = []  # 3D points in real world
    imgpoints = []  # 2D points in image plane

    inputFolder = "input"
    images = glob.glob(f"{inputFolder}/*.jpg")

    for idx,image in enumerate(images):
        printNBL(f"{idx}")
        img = cv2.imread(image)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        printNBL(".")

        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
        printNBL("-")

        # Add object and image points
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            #cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            #cv2.imshow("Image with corners", img)
            #resImg = resizeWithAspectRatio(img, height=1020)
            #cv2.imshow("Image with corners", resImg)
            #waits in ms
            #cv2.waitKey(1500)
            printNBL("|")

    cv2.destroyAllWindows()

    ret, cameraMatrix, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print("Camera matrix (Intrinsic Parameters):\n", cameraMatrix)
    print("\nDistortion coefficients:\n", dist)
    print("\nRotation Vectors:\n", rvecs)
    print("\nTranslation Vectors:\n", tvecs)

    if not os.path.exists("./output"):
            os.makedirs("./output")
    with open(f'./output/camera_calibration_data.txt', 'a') as file:
        file.write("Camera matrix (Intrinsic Parameters):\n", cameraMatrix)
        file.write("\nDistortion coefficients:\n", dist)
        file.write("\nRotation Vectors:\n", rvecs)
        file.write("\nTranslation Vectors:\n", tvecs)
    np.savez("./output/camera_calibration_data.npz", mtx=cameraMatrix, dist=dist, rvecs=rvecs, tvecs=tvecs)

    # Undistort all images in input+1 directory
    all_images = glob.glob("hw3-images/*.jpg")
    print("Undistorting...")
    for idx, image_path in enumerate(all_images):
        printNBL(f"{idx}")
        img = cv2.imread(image_path)
        h, w = img.shape[:2]

        newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w, h), 1, (w, h))

        printNBL(".")
        dst = cv2.undistort(img, cameraMatrix, dist, None, newCameraMatrix)

        # Crop the image to the region of interest (roi)
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]

        printNBL(".")
        filename = os.path.basename(image_path)
        output_path = f"./output/undistorted_{filename}"

        cv2.imwrite(output_path, dst)
        print(f"Saved {output_path}")


def showData():
    data = np.load("./output/camera_calibration_data.npz")

    camera_matrix = data['mtx']
    print("Camera Matrix:\n", camera_matrix)

if __name__ == "__main__":
    main()
    #showData()
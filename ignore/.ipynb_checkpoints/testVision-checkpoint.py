import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# Initialize the camera
cap = cv2.VideoCapture(0)  # 0 for the default camera

# Define the ArUco dictionary and parameters
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

# Generate an ArUco marker
marker_id = 1  # Change this value to generate a different marker
marker_size = 200  # Size of the marker in pixels
marker_image = np.zeros((marker_size, marker_size), dtype=np.uint8)
cv2.aruco.drawMarker(dictionary, marker_id, marker_size, marker_image, 1)

# Convert the marker image to a PIL Image
marker_pil = Image.fromarray(marker_image)

# Define the font for text overlay
font = ImageFont.truetype("arial.ttf", 16)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Detect ArUco markers in the frame
    markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(frame)

    # If an ArUco marker is detected
    if len(markerCorners) > 0:
        # Flatten the ArUco IDs list
        markerIds = markerIds.flatten()

        # Loop over the detected ArUco markers
        for i, corner in enumerate(markerCorners):
            # Extract the marker corners
            corners = corner.reshape((4, 2))
            (top_left, top_right, bottom_right, bottom_left) = corners

            # Convert the coordinates to integers
            top_right = (int(top_right[0]), int(top_right[1]))
            bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
            bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
            top_left = (int(top_left[0]), int(top_left[1]))

            # Draw the bounding box of the ArUco detection
            cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
            cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
            cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
            cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)

            # Calculate and draw the center of the ArUco marker
            center_x = int((top_left[0] + bottom_right[0]) / 2.0)
            center_y = int((top_left[1] + bottom_right[1]) / 2.0)
            cv2.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)

            # Draw the ArUco marker ID on the video feed
            cv2.putText(frame, str(markerIds[i]), (top_left[0], top_left[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Convert the frame to a PIL Image
            frame_pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

            # Create a drawing object
            draw = ImageDraw.Draw(frame_pil)

            # Draw the ArUco marker on the video feed
            frame_pil.paste(marker_pil, (top_left[0], top_left[1]))

            # Draw the position and orientation text
            position_text = f"Position: ({center_x}, {center_y})"
            orientation_text = "Orientation: TODO"  # You can calculate the orientation here
            draw.text((10, 10), position_text, font=font, fill=(255, 255, 255))
            draw.text((10, 30), orientation_text, font=font, fill=(255, 255, 255))

            # Convert the PIL Image back to OpenCV format
            frame = cv2.cvtColor(np.array(frame_pil), cv2.COLOR_RGB2BGR)

    # Display the resulting frame
    cv2.imshow('ArUco Marker Detection', frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close the windows
cap.release()
cv2.destroyAllWindows()
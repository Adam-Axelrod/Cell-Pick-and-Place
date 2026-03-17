import cv2

# 1. Initialize the camera (0 is usually the default built-in webcam)
cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

print("Camera feed started. Press 'q' to quit.")

while True:
    # 2. Capture frame-by-frame
    ret, frame = cap.read()

    # If the frame was not grabbed successfully, break the loop
    if not ret:
        print("Error: Can't receive frame. Exiting...")
        break

    # 3. Display the resulting frame in a window
    cv2.imshow('Camera Feed', frame)

    # 4. Wait for the 'q' key to be pressed to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 5. When everything is done, release the capture and close windows
cap.release()
cv2.destroyAllWindows()
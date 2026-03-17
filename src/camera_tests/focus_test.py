import cv2

# 1. Initialize the camera
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# 2. Disable Autofocus
# 0 = Manual Focus, 1 = Auto Focus
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) 

# 3. Set a specific focus value (Optional)
# The range is hardware dependent (often 0 to 255 or 0 to 1024)
# You may need to experiment with this value to get a clear image
cap.set(cv2.CAP_PROP_FOCUS, 100) 

print("Autofocus disabled. Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow('Manual Focus Feed', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
import cv2

# 1. Initialize the camera
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# 2. Disable Auto Exposure
# Most drivers use 0 for manual and 1 for auto, 
# but some use CAP_PROP_AUTO_EXPOSURE values like 1 (manual) and 3 (auto).
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) 

# 3. Set a specific Exposure value
# Negative values often represent powers of 2 (e.g., -6 = 1/64s)
# Positive values might be direct milliseconds depending on the driver.
# Start with -6 or -5 for a standard indoor light level.
cap.set(cv2.CAP_PROP_EXPOSURE, -6)

print("Auto exposure disabled. Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Display the current exposure value on the frame for debugging
    current_exp = cap.get(cv2.CAP_PROP_EXPOSURE)
    cv2.putText(frame, f"Exp: {current_exp}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow('Manual Exposure Feed', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
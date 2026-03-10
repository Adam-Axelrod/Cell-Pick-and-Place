import cv2

def find_all_camera_indices():
    """
    Checks for and returns a list of all available camera indices.
    """
    # Define a maximum index to check to prevent an excessively long scan
    max_index_to_check = 10 
    available_indices = []
    
    print("🚀 Starting scan for multiple camera indices (checking 0 to 9)...")

    for index in range(max_index_to_check):
        # Attempt to create a VideoCapture object
        cap = cv2.VideoCapture(index)
        
        # Check if the camera was opened successfully
        if cap.isOpened():
            print(f"✅ Camera found and opened successfully at index: **{index}**")
            available_indices.append(index)
            
            # Optional: Briefly show a frame to confirm it's a unique device
            try:
                ret, frame = cap.read()
                if ret:
                    cv2.imshow(f"Camera Index {index}", frame)
                    # Wait briefly (e.g., 500ms) to allow visual confirmation
                    cv2.waitKey(500) 
            except Exception as e:
                # Handle cases where read() fails even if isOpened() is True
                print(f"   Warning: Could not read frame from index {index}. Error: {e}")
                
            # Important: Release the camera object so the next iteration can try
            cap.release()
            
        else:
            # If the camera couldn't be opened, release the attempt and continue
            cap.release()
            print(f"   Index {index}: No camera found or inaccessible.")

    # Clean up any open windows
    cv2.destroyAllWindows()
    
    return available_indices

# Run the function
indices = find_all_camera_indices()

## ✨ Results Summary

if indices:
    print(f"\n🎉 **Found {len(indices)} available camera(s):**")
    print(f"The working indices are: **{indices}**")
    print("\n💡 You can use any of these indices (e.g., `cv2.VideoCapture(indices[0])`) to access your cameras.")
else:
    print("\n❌ No cameras found within the checked range.")
    print("💡 Tip: Ensure your USB cameras are connected and drivers are installed.")
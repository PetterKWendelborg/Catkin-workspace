import cv2
import cv2.aruco as aruco

def generate_bulk_markers(aruco_dict):
    # Prompt the user for input
    marker_size = 300
    num_markers = 1
    
    # Initialize list to store generated markers
    marker_imgs = []
    
    # Loop to generate markers
    for marker_id in range(num_markers):
        # Generate ArUco marker (correct function name for OpenCV 4.2)
        marker_img = aruco.drawMarker(aruco_dict, marker_id, marker_size)
        
        # Save the marker image
        cv2.imwrite(f"marker_{marker_id}.png", marker_img)
        marker_imgs.append(marker_img)
    
    # Display each generated marker
    for marker_img in marker_imgs:
        cv2.imshow("Marker", marker_img)
        print("Dimensions (height x width):", marker_img.shape)
        cv2.waitKey(0)  # Wait for any key press
        cv2.destroyAllWindows()

def main():
    # Use correct dictionary name (DICT_4X4_50, DICT_5X5_50, etc.)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)
    
    # Call the generation function
    generate_bulk_markers(aruco_dict)

if __name__ == "__main__":
    main()
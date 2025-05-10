import cv2
import numpy as np

def generate_checkerboard(rows, cols, square_size):
    # Calculate the overall size of the checkerboard image
    img_height = rows * square_size
    img_width = cols * square_size

    # Create a blank white image
    checkerboard_img = np.ones((img_height, img_width), dtype=np.uint8) * 255

    # Loop through each row and column to create the checkerboard pattern
    for i in range(rows):
        for j in range(cols):
            if (i + j) % 2 == 0:  # Changed from 'x z' to modulus operator
                # Calculate coordinates for the square
                top_left = (j * square_size, i * square_size)  # Fixed variable name from 'l' to 'i'
                bottom_right = ((j + 1) * square_size, (i + 1) * square_size)

                # Draw a black square
                cv2.rectangle(checkerboard_img, top_left, bottom_right, 0, -1)

    # Save the checkerboard image with proper extension
    filename = "checkerboard_{}x{}.png".format(rows, cols)  # Changed .ppg to .png
    cv2.imwrite(filename, checkerboard_img)

    # Display the checkerboard
    cv2.imshow("Checkerboard", checkerboard_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Call the function with valid parameters (minimum 1 row and 1 column)
generate_checkerboard(rows=7, cols=10, square_size=50)  # Changed from 0,0 to reasonable values
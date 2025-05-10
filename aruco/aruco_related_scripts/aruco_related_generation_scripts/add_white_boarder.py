from PIL import Image, ImageOps
import os

# Settings
input_png = "marker_3.png"    # Input file (no border)
output_png = "marker_4x4_3.png"  # Output file (with border)
border_ratio = 0.2  # 20% border

# Load the image
img = Image.open(input_png)

# Calculate border size (20% of the largest dimension)
border_size = int(max(img.size) * border_ratio)

# Add a white border
bordered_img = ImageOps.expand(img, border=border_size, fill="white")

# Save the new image
bordered_img.save(output_png)
print(f"Saved bordered image to: {os.path.abspath(output_png)}")
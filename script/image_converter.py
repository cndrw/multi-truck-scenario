from PIL import Image, ImageOps
import numpy as np

# Enum-like dictionary for colors
COLOR_MAP = {
    range(-128, -69): -128,  # Red
    range(-70, -1): -70,     # Yellow
    range(0, 81): 0,         # Grey
    range(80, 126): 100,     # Black
    range(127, 256): 127     # Green (255 included)
}

def map_color(value):
    """Map a color value to its corresponding enum."""
    if value >= -128 and value <= -70:
        return -128  # Red
    elif value >= -69 and value <= -1:
        return -70   # Yellow
    elif value >= 0 and value <= 80:
        return 0     # Grey
    elif value >= 81 and value <= 100:
        return 100   # Black
    elif value >= 101 and value <= 255:
        return 127   # Green
    else:
        raise ValueError(f"Value {value} does not fit in any color range.")

def convert_image_to_grid(image_path):
    """
    Convert an image to a grid, mirror it, and return the grid, height, and width.
    
    Args:
        image_path (str): Path to the input image.
    
    Returns:
        tuple: A tuple containing a 1D list representing the mirrored grid, the height, and the width of the image.
    """
    # Open the image
    image = Image.open(image_path).convert("L")  # Convert to grayscale

    # Mirror the image
    mirrored_image = ImageOps.mirror(image)

    # Convert to numpy array
    image_data = np.array(mirrored_image)
    
    # Get dimensions
    height, width = image_data.shape
    
    # Flatten and map colors
    mapped_grid = [map_color(pixel) for pixel in image_data.flatten()]
    return mapped_grid, height, width

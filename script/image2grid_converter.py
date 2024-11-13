from PIL import Image

class ImageToCppGridConverter:

    def __init__(self, image_path):
        # Load the image
        self.image = Image.open(image_path)
        self.width, self.height = self.image.size

    def get_color_of_pixel(self, rgb_value):
        """
        Maps an RGB color to the Colors enum based on the specified ranges.
        Returns a string representing the Colors enum name.

        To use get a good result, pay attention to the RGB values of the colors in the image, when creating in the image editor.
        PNG file should bes used for best results.
        Recommenden values:
        - Red: (255, 0, 0)
        - Green: (0, 255, 0)
        - Yellow: (255, 255, 0)
        - Black: (0, 0, 0)
        - Gray: (255, 255, 255) - Will be white in RVIZ, is called gray in Occupancy Grid
        """
        gray_value = int((rgb_value[0] + rgb_value[1] + rgb_value[2]) / 3)

        if rgb_value[0] > rgb_value[1] and rgb_value[0] > rgb_value[2]:  # Red dominant
            return "Colors::Red"
        elif rgb_value[1] > rgb_value[0] and rgb_value[1] > rgb_value[2]:  # Green dominant
            return "Colors::Green"
        elif rgb_value[0] == rgb_value[1] and rgb_value[1] > rgb_value[2]:  # Yellow dominant
            return "Colors::Yellow"
        elif gray_value <= 20:  # Black
            return "Colors::Black"
        elif gray_value <= 200:  # Grey -but actually white in grid
            return "Colors::Gray"
        else:
            return "Colors::Gray"
        # return rgb_value # only for testing
        # return gray_value # only for testing

    def convert_to_cpp_grid_data(self):
        """
        Convert Pixel data to C++ grid data.
        """
        # init grid data as string
        cpp_grid_data = "grid.data = {\n"
        
        for y in range(self.height):
            row_data = "    "
            for x in range(self.width):
                color_enum = self.get_color_of_pixel(self.image.getpixel((x, y)))
                row_data += f"{color_enum}, "
            cpp_grid_data += row_data.rstrip(", ") + "\n"  # Strip trailing comma and newline for each row
        
        cpp_grid_data = cpp_grid_data.rstrip(", \n") + "\n};"  # Finalize formatting
        
        return cpp_grid_data

    def get_dimensions(self):
        # return dimensions as parameters of class
        return self.width, self.height

# Final Steps:
# Path to file:
image_path = "painting_10x10_6_colors.png"
converter = ImageToCppGridConverter(image_path)
cpp_grid_data = converter.convert_to_cpp_grid_data()

# Print the results - should be transferred to cpp
print("C++ grid data:\n", cpp_grid_data)
print("Height =", converter.height)
print("Width =", converter.width)


'''ToDo:
- Implement to Launchfile
- Add Method so it is executed in the Launchfile
- Method in Launchfile shall execute this script together with specified image
- This script + image files must be transferred to share directory
- Paint scenario_S2 and implemet it in the launchfile
'''


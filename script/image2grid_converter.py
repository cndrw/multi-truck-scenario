from PIL import Image

class ImageToCppGridConverter:

    def __init__(self, image_path):
        # Load the image
        self.image = Image.open(image_path)
        self.width, self.height = self.image.size

    def get_color_of_pixel(self, rgb_value):
        # convert RGB to grayscale
        gray_value = int((rgb_value[0] + rgb_value[1] + rgb_value[2]) / 3)

        if rgb_value[0] > rgb_value[1] and rgb_value[0] > rgb_value[2]:  # Red dominant
            return -128  # "Colors::Red"
        elif rgb_value[1] > rgb_value[0] and rgb_value[1] > rgb_value[2]:  # Green dominant
            return 127  # "Colors::Green"
        elif rgb_value[0] == rgb_value[1] and rgb_value[1] > rgb_value[2]:  # Yellow dominant
            return -70  # "Colors::Yellow"
        elif gray_value <= 20:  # Black
            return 100  # "Colors::Black"
        elif gray_value <= 200:  # Grey (actually white in RViz)
            return 0  # "Colors::Gray"
        else:
            return 0  # "Colors::Gray"

    def convert_to_cpp_grid_data(self):
        # Convert the image to a flattened grid
        static_map = []
        for y in range(self.height):
            for x in range(self.width):
                color_enum = self.get_color_of_pixel(self.image.getpixel((x, y)))
                static_map.append(color_enum)
        return static_map

    def get_dimensions(self):
        
        return self.width, self.height


def generate_rviz_static_map(image_path):
    # Convert the image to a flattened grid
    converter = ImageToCppGridConverter(image_path)
    cpp_grid_data = converter.convert_to_cpp_grid_data()
    width, height = converter.get_dimensions()

    # Flip the rows for RViz (Y-axis correction)
    rviz_matrix = []
    for i in range(height):
        start_index = i * width
        end_index = start_index + width
        rviz_matrix.insert(0, cpp_grid_data[start_index:end_index])

    # Flatten the matrix for the RViz static map
    flattened_map = [value for row in rviz_matrix for value in row]

    # Return the final flattened grid as a dictionary
    return {
        "static_map": flattened_map,
        "width": width,
        "height": height,
    }


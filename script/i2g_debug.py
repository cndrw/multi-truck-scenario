#!/usr/bin/env python3

import argparse
from PIL import Image

class ImageToCppGridConverter:

    def __init__(self, image_path):
        self.image = Image.open(image_path)
        self.width, self.height = self.image.size

    def get_color_of_pixel(self, rgb_value):
        gray_value = int((rgb_value[0] + rgb_value[1] + rgb_value[2]) / 3)

        if rgb_value[0] > rgb_value[1] and rgb_value[0] > rgb_value[2]:
            return -128  # Colors::Red
        elif rgb_value[1] > rgb_value[0] and rgb_value[1] > rgb_value[2]:
            return 127  # Colors::Green
        elif rgb_value[0] == rgb_value[1] and rgb_value[1] > rgb_value[2]:
            return -70  # Colors::Yellow
        elif gray_value <= 20:
            return 100  # Colors::Black
        elif gray_value <= 200:
            return "000"  # Colors::Gray
        else:
            return "000"  # Colors::Gray

    def convert_to_cpp_grid_data(self):
        static_map = []
        for y in range(self.height):
            for x in range(self.width):
                color_enum = self.get_color_of_pixel(self.image.getpixel((x, y)))
                static_map.append(color_enum)
        return static_map

    def get_dimensions(self):
        return self.width, self.height


def generate_rviz_static_map(image_path):
    converter = ImageToCppGridConverter(image_path)
    cpp_grid_data = converter.convert_to_cpp_grid_data()
    width, height = converter.get_dimensions()

    rviz_matrix = []
    for i in range(height):
        start_index = i * width
        end_index = start_index + width
        rviz_matrix.insert(0, cpp_grid_data[start_index:end_index])

    flattened_map = [value for row in rviz_matrix for value in row]

    return {
        "static_map": flattened_map,
        "width": width,
        "height": height,
    }

def main():
    parser = argparse.ArgumentParser(description="Convert image to RViz occupancy grid.")
    parser.add_argument("image", help="Path to the image file (e.g., big_test.png)")
    args = parser.parse_args()

    result = generate_rviz_static_map(args.image)
    print(result)

if __name__ == "__main__":
    main()

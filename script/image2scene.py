import argparse
from PIL import Image
# from math import dist
from itertools import combinations

# checks every pixel if yellow
def find_yellow_pixels(img):
    pixels = img.load()
    width, height = img.size
    yellow = (255, 255, 0)
    yellow_pixels = []

    for y in range(height):
        for x in range(width):
            if pixels[x, y][:3] == yellow:
                coord = (x, height - 1 - y)  # Flip y-axis for bottom-left origin
                yellow_pixels.append(coord)
    return yellow_pixels

def is_rectangle(cluster):
    if len(cluster) != 4:
        return False

    x_vals = {p[0] for p in cluster}
    y_vals = {p[1] for p in cluster}

    if len(x_vals) != 2 or len(y_vals) != 2:
        return False

    # All 4 corner combinations must exist
    expected_corners = {(x, y) for x in x_vals for y in y_vals}
    return expected_corners == set(cluster)

# Method sorts valid crossings
# Checks if the 4 points form a rectangle
def find_rectangle_crossings(yellow_pixels):
    valid_rectangles = []

    for combo in combinations(yellow_pixels, 4):
        if is_rectangle(combo):
            # Avoid duplicates (sort coordinates)
            sorted_rect = tuple(sorted(combo))
            if sorted_rect not in valid_rectangles:
                valid_rectangles.append(sorted_rect)

    return valid_rectangles

"""
# TODO: Validate crossings by checking edges
# 1. Method: Use 2 Points, check if the edge between them is valid
# 2. Method: check rectangles if edges are valid
"""
def edge_valid(combo_of_two_pixels, img):
    # method must use img to recheck pixels
    # method that calls this uses output of find_rectangle_crossings() -> input is list of tuples
    # Check if edge between two pixels is valid
    # green & white is valid
    # black & yellow invalid
    # don't check corner point, because yellow, obviously
    # input is two tuples of (x, y) coordinates
    # return bool
    p1, p2 = combo_of_two_pixels

    pixels = img.load()
    width, height = img.size
    valid_colors = {(255, 255, 255), (0, 255, 0)}  # white and green

    x1, y1 = p1
    x2, y2 = p2

    # Convert to image coordinates (flip Y-axis)
    y1_img = height - 1 - y1
    y2_img = height - 1 - y2

    if x1 == x2:
        # Vertical edge
        for y in range(min(y1_img, y2_img) + 1, max(y1_img, y2_img)):
            if pixels[x1, y][:3] not in valid_colors:
                return False
    elif y1 == y2:
        # Horizontal edge
        y_img = height - 1 - y1
        for x in range(min(x1, x2) + 1, max(x1, x2)):
            if pixels[x, y_img][:3] not in valid_colors:
                return False
    else:
        # Not axis-aligned (diagonal)
        return False

    return True

def is_rectangle_valid_crossing(rect, img):
    # Check if all edges of the rectangle are valid
    edges = [
        (rect[1], rect[3]),  # Top edge - TL to TR
        (rect[3], rect[2]),  # Right edge - TR to BR
        (rect[2], rect[0]),  # Bottom edge - BR to BL
        (rect[0], rect[1]),  # Left edge - BL to TL
    ]

    for edge in edges:
        if not edge_valid(edge, img):
            return False

    return True

def distance(p1, p2):
    # p1 and p2 are tuples of (x, y) coordinates
    # return dist between x values and y values
    # return tuple of (x, y) distances
    x1, y1 = p1
    x2, y2 = p2

    return abs(x1 - x2), abs(y1 - y2)


def output_values(valid_crossing):
    # output asked values
    # 1. height of crossing
    # 2. width of crossing
    # 3. bot left of crossing
    # only 1 crossing per execution of this method
    # iterate over all crossings in the final implementation
    # return list/tuple - not set yet

    p1, p2, p3, p4 = valid_crossing
    # p1 = bot left, p2 = top left, p3 = bot right, p4 = top right
    x_dist, y_dist = distance(p1, p4)  # calculate height and width from diagonal points


    return (x_dist, y_dist, p1)
    # width, height, pos bot left

def exec_script(image_path):
    # create method that can be run in another script that only forwards output values
    # shall be iterared over all crossings
    # shall return list of tuples with all crossings
    # use code that works in main() method
    
    img = Image.open(image_path)
    # img = image_path

    yellow_pixels = find_yellow_pixels(img)
    rectangles = find_rectangle_crossings(yellow_pixels)
    valid_crossings = [rect for rect in rectangles if is_rectangle_valid_crossing(rect, img)]

    return [output_values(crossing) for crossing in valid_crossings]



def main():
    # Argument parser for command line input
    parser = argparse.ArgumentParser(description="Detect crossings formed by yellow corner pixels.")
    parser.add_argument("image_path", help="Path to the image file")
    args = parser.parse_args()

    # Select image path
    img = Image.open(args.image_path)

    # find all yellow pixels in the image
    # print coordinates of all yellow pixels
    yellow_pixels = find_yellow_pixels(img)
    print(f"Found {len(yellow_pixels)} yellow pixels.")

    # print potential crossings - not validated crossings
    rectangles = find_rectangle_crossings(yellow_pixels)
    print(f"\nDetected {len(rectangles)} potential crossings:")
    for i, rect in enumerate(rectangles):
        print(f"Crossing {i+1}: {rect}")

    # Validate crossings
    valid_crossings = [rect for rect in rectangles if is_rectangle_valid_crossing(rect, img)]
    print(f"\nValidated crossings: {len(valid_crossings)}")
    for i, rect in enumerate(valid_crossings):
        print(f"Crossing {i+1}: {rect}")

    # print(output_values(valid_crossings[]))  # Example output for the first valid crossing
    print(exec_script(img))  # Example output for the first valid crossing

if __name__ == "__main__":
    main()

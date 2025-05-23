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


def output_values_event(valid_crossing):
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

def output_values_streets(valid_crossing, image):
    # output lists of all 4 sides of the crossing
    # one list for each street
    # street = (width, direction)	
    # width = no. of white pixel on edge
    # direction = (x, y) direction of the street
    # direction = (1, 0) = right, (0, 1) = up, (-1, 0) = left, (0, -1) = down
    # return list of lists with all 4 sides of the crossing
    
    p1, p2, p3, p4 = valid_crossing
    # p1 = bot left, p2 = top left, p3 = bot right, p4 = top right

    img = image.load()
    width, height = image.size
    # count white pixels on each edge
    # left edge (p1 to p2)
    # left = []
    # right = []
    # top = []
    # bottom = []

    left_edge = 0
    for y in range(p1[1], p2[1] + 1):
        if img[p1[0], y][:3] == (255, 255, 255):
            left_edge += 1
    # right edge (p3 to p4)
    right_edge = 0 
    for y in range(p3[1], p4[1] + 1):
        if img[p3[0], y][:3] == (255, 255, 255):
            right_edge += 1
    # top edge (p2 to p4)  
    bottom_edge = 0
    for x in range(p2[0], p4[0] + 1):
        if img[x, p2[1]][:3] == (255, 255, 255):
            bottom_edge += 1
    # bottom edge (p1 to p3)
    top_edge = 0
    for x in range(p1[0], p3[0] + 1):
        if img[x, p1[1]][:3] == (255, 255, 255):
            top_edge += 1

    # left.append(left_edge)
    # right.append(right_edge)
    # top.append(top_edge)
    # bottom.append(bottom_edge)
    # witdhts of the streets
    left = left_edge
    right = right_edge
    top = top_edge
    bottom = bottom_edge

    # now append direction of the streets

    # left direction = (1, 0) = right, (0, 1) = up, (-1, 0) = left, (0, -1) = down
    # left.append((-1, 0))  # left direction
    # right.append((1, 0))  # right direction
    # top.append((0, 1))  # bottom direction
    # bottom.append((0, -1))  # top direction
    # return list of lists with all 4 sides of the crossing
    # since top and bottom are switched due to inverted y axis, we need to switch them back


    return [left, right, top, bottom]  # left, right, top, bottom


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



    # extend output, so streets are declared
    # [[event1[str1][str2][str3][str4]], [event2[str1][str2][str3][str4]], ...]
    # event(width, height, bot left corner)
    # str(width, (direction)) 

    return [output_values_event(crossing) for crossing in valid_crossings]
    # return [output_values_streets(crossing, img) for crossing in valid_crossings]

def output_lists(output_events):

    width_values = []
    height_values = []
    bot_left_x_values = []
    bot_left_y_values = []
    # iterate over all events and append values to the lists

    for event in output_events:
        width_values.append(event[0])
        height_values.append(event[1])
        bot_left_x_values.append(event[2][0])
        bot_left_y_values.append(event[2][1])

    # return list of lists with all values
    return [width_values, height_values, bot_left_x_values, bot_left_y_values]


def output_event(image_path):
    return output_lists(exec_script(image_path))

def output_event_streets(image_path):
    img = Image.open(image_path)
    # img = image_path

    yellow_pixels = find_yellow_pixels(img)
    rectangles = find_rectangle_crossings(yellow_pixels)
    valid_crossings = [rect for rect in rectangles if is_rectangle_valid_crossing(rect, img)]

    # make lists like for events
    width_values_left_streets = []
    width_values_right_streets = []
    width_values_top_streets = []
    width_values_bottom_streets = []
    # iterate over all events and append values to the lists
    for crossing in valid_crossings:
        left, right, top, bottom = output_values_streets(crossing, img)
        width_values_left_streets.append(left)
        width_values_right_streets.append(right)
        width_values_top_streets.append(top)
        width_values_bottom_streets.append(bottom)

    # return [output_values_streets(crossing, img) for crossing in valid_crossings]
    return [width_values_left_streets, width_values_right_streets, width_values_top_streets, width_values_bottom_streets]

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
    print(output_values_streets(valid_crossings[0], img))  # Example output for the first valid crossing
    print(output_values_streets(valid_crossings[1], img))  # Example output for the first valid crossing
    print(output_values_streets(valid_crossings[2], img))  # Example output for the first valid crossing
    print(exec_script(img))  # Example output for the first valid crossing
    print(output_event(img))  # Example output for the first valid crossing
    print(output_event_streets(img))  # Example output for the first valid crossing


if __name__ == "__main__":
    main()

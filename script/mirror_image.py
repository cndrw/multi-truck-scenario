import argparse
from pathlib import Path
from PIL import Image

def mirror_image(image_path: Path):
    if not image_path.exists() or not image_path.is_file():
        print(f"Error: File '{image_path}' does not exist.")
        return

    try:
        with Image.open(image_path) as img:
            mirrored = img.transpose(Image.FLIP_TOP_BOTTOM)

            new_filename = image_path.stem + "_mirrored" + image_path.suffix
            output_path = image_path.parent / new_filename

            mirrored.save(output_path)
            print(f"Mirrored image saved as: {output_path}")

    except Exception as e:
        print(f"An error occurred: {e}")

def main():
    parser = argparse.ArgumentParser(description="Mirror an image horizontally.")
    parser.add_argument("image", type=Path, help="Path to the image file")

    args = parser.parse_args()
    mirror_image(args.image)

if __name__ == "__main__":
    main()

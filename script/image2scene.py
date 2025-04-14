from PIL import Image

class ImageSceneAnalyser:

    def __init__(self, image_path):
        #Load image
        self.image = Image.open(image_path)
        self.width, self.height = self.image.size

    def reshape_matrix(self, matrix, width):
        # Reshape the matrix to the specified width
        reshaped_matrix = []
        for i in range(0, len(matrix), width):
            reshaped_matrix.append(matrix[i:i + width])
        return reshaped_matrix
import os
import cv2

from pipeline.pipeline import Pipeline

class SaveImage(Pipeline):

    """Docstring for SaveImage. """

    def __init__(self, src, path):
        """TODO: to be defined.

        :dst: TODO

        """
        self.src = src
        self.path = path
        Pipeline.__init__(self)

    def map(self, data):
        """TODO: Docstring for map.

        :data: TODO
        :returns: TODO

        """
        image = data[self.src]
        image_id = data["image_id"]

        output = image_id.split(os.path.sep)
        dirname = output[:-1]
        if len(dirname) > 0:
            dirname = os.path.join(*dirname)
            dirname = os.path.join(self.path, dirname)
            os.makedirs(dirname, exist_ok=True)
        else:
            dirname = self.path
        img_filename = f"{output[-1].rsplit('.', 1)[0]}.png"
        txt_filename = f"{output[-1].rsplit('.', 1)[0]}.txt"
        img_path = os.path.join(dirname, img_filename)

        cv2.imwrite(img_path, image)

        # if "classes" not in data:
        #     return data

        txt_path = os.path.join(dirname, txt_filename)

        with open(txt_path, "w") as txt_file:
            if "classes" not in data:
                return data
            for i, c in enumerate(data["classes"]):
                p = data["scores"][i]
                txt_file.write(f"{c},{p}\n")
        return data

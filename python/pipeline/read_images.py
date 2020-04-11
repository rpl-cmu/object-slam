import os
import glob
from itertools import chain
import cv2

from pipeline.pipeline import Pipeline


class ReadImages(Pipeline):
    """ReadImages pipeline task: read the image with valid extension from a folder structure, and return an iterable image"""
    def __init__(self, path, valid_exts=[".png", ".jpg"]):
        """TODO: to be defined.

        :valid_exts: TODO
        :".jpg"]: TODO
        :: TODO

        """
        self.path = path
        self.valid_exts = valid_exts

        Pipeline.__init__(self)

    def generator(self):
        """ Yield iterable file after reading from folder structure
        :returns: TODO

        """

        filelist = (chain.from_iterable(chain.from_iterable(glob.glob(os.path.join(x[0], "*.color") + ext)
                                        for ext in self.valid_exts)
                                        for x in os.walk(self.path)))

        for filename in filelist:
            image_file = filename
            image = cv2.imread(image_file)
            data = {
                    'image_id': image_file,
                    'image': image
                   }
            if self.filter(data):
                yield self.map(data)

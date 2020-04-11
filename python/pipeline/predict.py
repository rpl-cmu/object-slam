from pipeline.pipeline import Pipeline

from detectron2.engine.defaults import DefaultPredictor

class Predict(Pipeline):

    """Pipeline task to perform inference"""

    def __init__(self, cfg):
        """
        :cfg: TODO
        """
        self.predictor = DefaultPredictor(cfg)
        Pipeline.__init__(self)

    def map(self, data):
        """

        :data: TODO
        :returns: TODO

        """
        image  = data["image"]
        predictions = self.predictor(image)
        data["predictions"] = predictions

        return data

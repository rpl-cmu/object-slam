import numpy as np
import matplotlib.pyplot as plt
from pipeline.pipeline import Pipeline

class StoreMasks(Pipeline):

    """Docstring for StoreMasks. """

    def __init__(self, dst):
        """TODO: to be defined. """
        self.dst = dst
        self.dst_classes = "classes"
        self.dst_scores = "scores"
        self.NUM_MASKS = 16
        Pipeline.__init__(self)

    def map(self, data):
        """TODO: Docstring for map.

        :data: TODO
        :returns: TODO

        """
        self.store_mask(data)

        return data

    def store_mask(self, data):
        """TODO: Docstring for store_mask.

        :data: TODO
        :returns: TODO

        """
        if "predictions" not in data:
            return

        predictions = data["predictions"]
        if "instances" not in predictions:
            return

        instances = predictions["instances"]
        if not instances.has("pred_masks"):
            return

        mask = instances.pred_masks.cpu()
        mask = mask.permute(1, 2, 0).numpy()

        dst_mask = np.zeros((data["image"]).shape[:2], dtype=np.uint16)
        # bitwise encode a max of 16 masks in 16-bit image
        for i in range(min(self.NUM_MASKS, mask.shape[2])):
            current_mask = np.where(mask[:, :, i] > 0, 2**i, 0).astype(np.uint16)
            dst_mask = np.bitwise_or(dst_mask, current_mask)

        if instances.has("pred_classes") and instances.has("scores"):
            if len(instances.pred_classes.cpu().numpy()) > 0:
                data[self.dst_classes] = instances.pred_classes.cpu().numpy()
            if len(instances.scores.cpu().numpy()) > 0:
                data[self.dst_scores] = instances.scores.cpu().numpy()
        # plt.imshow(dst_mask, cmap="gray", vmin=0, vmax=4096)
        # plt.show()
        data[self.dst] = dst_mask.astype(np.uint16)

#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import zmq
import struct

import torch
from detectron2.config import get_cfg
from detectron2.engine.defaults import DefaultPredictor

import sys
sys.path.insert(1, "../build/msg/")
from image_pb2 import *

sys.path.insert(1, "../3rdparty/detectron2/projects/PointRend")
import point_rend


def create_cfg(config_file, weights_file, cpu=False):
    """Create configuration to obtain masks from detectron2 model
    :returns: TODO

    """
    cfg = get_cfg()
    point_rend.add_pointrend_config(cfg)
    cfg.merge_from_file(config_file)

    cfg.MODEL.WEIGHTS = weights_file

    cfg.MODEL.RETINANET.SCORE_THRESH_TEST = 0.6
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.6
    cfg.MODEL.PANOPTIC_FPN.COMBINE.INSTANCES_CONFIDENCE_THRESH = 0.6

    if cpu or not torch.cuda.is_available():
        cfg.MODEL.DEVICE = "cpu"
    cfg.freeze()

    return cfg

def get_masks(predictions, image):
    """TODO: Docstring for get_masks.

    :arg1: TODO
    :returns: TODO

    """
    if "instances" not in predictions:
        return

    instances = predictions["instances"]
    if not instances.has("pred_masks"):
        return

    mask = instances.pred_masks.cpu()
    mask = mask.permute(1, 2, 0).numpy()

    dst_mask = np.zeros((image).shape[:2], dtype=np.uint16)
    # bitwise encode a max of 16 masks in 16-bit image
    for i in range(min(16, mask.shape[2])):
        current_mask = np.where(mask[:, :, i] > 0, 2**i, 0).astype(np.uint16)
        dst_mask = np.bitwise_or(dst_mask, current_mask)

    classes, scores = [], []
    if instances.has("pred_classes") and instances.has("scores"):
        if len(instances.pred_classes.cpu().numpy()) > 0:
            classes = instances.pred_classes.cpu().numpy()
        if len(instances.scores.cpu().numpy()) > 0:
            scores = instances.scores.cpu().numpy()
    processed_image = dst_mask.astype(np.uint16)

    return processed_image, classes, scores



def main():
    """TODO: Docstring for main.

    :args: TODO
    :returns: TODO

    """

    context = zmq.Context()
    receiver = context.socket(zmq.SUB)
    receiver.connect("tcp://localhost:4242")
    receiver.setsockopt_string(zmq.SUBSCRIBE, "")
    receiver.setsockopt(zmq.LINGER, 0)
    receiver.setsockopt(zmq.CONFLATE, 1)

    publisher = context.socket(zmq.PUB)
    publisher.bind("tcp://*:4243")
    publisher.setsockopt(zmq.LINGER, 0)

    config_file = "../3rdparty/detectron2/projects/PointRend/configs/InstanceSegmentation/pointrend_rcnn_R_50_FPN_3x_coco.yaml"
    weights_file = "https://dl.fbaipublicfiles.com/detectron2/PointRend/InstanceSegmentation/pointrend_rcnn_R_50_FPN_3x_coco/164955410/model_final_3c3198.pkl"

    cfg = create_cfg(config_file, weights_file)
    predictor = DefaultPredictor(cfg)
    while True:
        try:
            # Receiving image in bytes
            color_image = ColorImage()
            data = receiver.recv()
            color_image.ParseFromString(data)
            print(color_image.width)
            print(color_image.height)
            print(color_image.num_channels)
            image_bytes = np.frombuffer(color_image.data, dtype=np.uint8)
            image = np.reshape(image_bytes, (color_image.width, color_image.height, color_image.num_channels))
            image = image[20:-20, 20:-20]
            print(image.shape)
            predictions = predictor(image)
            processed_image, classes, scores = get_masks(predictions, image)
            processed_image = np.pad(processed_image, 20, 'constant')
            print(processed_image.shape)
            mask_image = MaskImage()
            mask_image.width = color_image.width
            mask_image.height = color_image.height
            mask_image.num_channels = 1
            mask_image.bytes_per_channel = 2
            mask_image.data = processed_image.tobytes()
            mask_image.labels.extend(classes)
            mask_image.scores.extend(scores)
            publisher.send(mask_image.SerializeToString())
        except KeyboardInterrupt:
            print("Keyboard interrupt received stopping")
            receiver.close()
            publisher.close()
            context.term()
            break


if __name__ == "__main__":
    main()

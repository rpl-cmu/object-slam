#!/usr/bin/env python
import os
import argparse
from tqdm import tqdm
import multiprocessing as mp
import torch

from detectron2.config import get_cfg
import sys
sys.path.insert(1, "/home/akashsharma/Documents/projects/detectron2/projects/PointRend")
import point_rend

from pipeline.read_images import ReadImages
from pipeline.predict import Predict
from pipeline.store_masks import StoreMasks
from pipeline.save_image import SaveImage


def create_cfg(config_file, weights_file, cpu=False):
    """Create configuration to obtain masks from detectron2 model
    :returns: TODO

    """
    cfg = get_cfg()
    point_rend.add_pointrend_config(cfg)
    cfg.merge_from_file(config_file)

    cfg.MODEL.WEIGHTS= weights_file


    cfg.MODEL.RETINANET.SCORE_THRESH_TEST = 0.6
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.6
    cfg.MODEL.PANOPTIC_FPN.COMBINE.INSTANCES_CONFIDENCE_THRESH = 0.6

    if cpu or not torch.cuda.is_available():
        cfg.MODEL.DEVICE = "cpu"
    cfg.freeze()

    return cfg


def create_arg_parser():
    """TODO: Docstring for create_arg_parse.
    :returns: TODO

    """
    parser = argparse.ArgumentParser(description="Process dataset to obtain instance masks")
    parser.add_argument("-i", "--input", required=True, help="path to dataset containing images")
    parser.add_argument("-o", "--output", default="../../output", help="path to processed dataset (default: ../../output)")

    # parser.add_argument("--config-file", default="/home/akashsharma/Documents/projects/detectron2/configs/quick_schedules/mask_rcnn_R_50_FPN_instant_test.yaml",
    #                     help="path to config file (default: home/akashsharma/Documents/projects/detectron2/configs/COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml)")
    parser.add_argument("--config-file", default="/home/akashsharma/Documents/projects/detectron2/projects/PointRend/configs/InstanceSegmentation/pointrend_rcnn_R_50_FPN_3x_coco.yaml",
                        help="path to config file (default: home/akashsharma/Documents/projects/detectron2/configs/COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml)")


    # parser.add_argument("--weights-file", default="detectron2://COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x/137849600/model_final_f10217.pkl",
    #                     help="weights file for config (default: detectron2://COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x/137849600/model_final_f10217.pkl)")
    parser.add_argument("--weights-file", default="https://dl.fbaipublicfiles.com/detectron2/PointRend/InstanceSegmentation/pointrend_rcnn_R_50_FPN_3x_coco/164955410/model_final_3c3198.pkl",
                        help="weights file for config (default: detectron2://COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x/137849600/model_final_f10217.pkl)")
    return parser

def main(args):
    """TODO: Docstring for main.

    :args: TODO
    :returns: TODO

    """
    # Create output directory if doesn't exist
    os.makedirs(args.output, exist_ok=True)
    cfg = create_cfg(args.config_file, args.weights_file)

    read_images = ReadImages(args.input)
    predict = Predict(cfg)
    store_masks = StoreMasks("mask")
    save_image = SaveImage("mask", args.output)

    pipeline = (read_images | predict | store_masks | save_image)
    try:
        for _ in tqdm(pipeline, desc="Processing"):
            pass
    except StopIteration:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    parser = create_arg_parser()
    args = parser.parse_args()
    main(args)



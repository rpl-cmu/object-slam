# Pre-process dataset

This pipeline pre-processes a dataset containing images into the top 3 masks predictions found by Mask-RCNN. The masks are stored as the 3 channels of the image, and the mask class is stored separately as txt file.

The projects depends on [detectron2](https://github.com/facebookresearch/detectron2), and was heavily inspired from [detectron2-pipeline](https://github.com/jagin/detectron2-pipeline)

## Getting started

Install detectron2 on a folder above this project. Refer to [INSTALL.md](https://github.com/facebookresearch/detectron2/blob/master/INSTALL.md) for detailed instructions.

## Running

Run the `process_dataset` script to run the inference on the image folder

    $ python process_dataset.py -i ~/Documents/datasets/sun3d/seq-01/




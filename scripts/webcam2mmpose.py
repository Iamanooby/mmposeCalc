#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Copyright (c) OpenMMLab. All rights reserved.
import logging
import mimetypes
import os
import time
from argparse import ArgumentParser

import cv2
import json_tricks as json

import numpy as np


import mmengine
from mmengine.logging import print_log

import mmcv


from mmpose.apis import inference_topdown
from mmpose.apis import init_model as init_pose_estimator
from mmpose.evaluation.functional import nms
from mmpose.registry import VISUALIZERS
from mmpose.structures import merge_data_samples, split_instances
from mmpose.utils import adapt_mmdet_pipeline

try:
    from mmdet.apis import inference_detector, init_detector
    has_mmdet = True
except (ImportError, ModuleNotFoundError):
    has_mmdet = False


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        #/home/chihan/standChair_ws2/src/mmposeCalc/mmposeCalc/rtmpose/rtmdet/person/rtmdet_nano_320-8xb32_coco-person.py
        directoryRtmPose = 'src/mmposeCalc/mmposeCalc/rtmpose/'

        self.det_config = directoryRtmPose + 'rtmdet/person/rtmdet_nano_320-8xb32_coco-person.py'
        self.det_checkpoint = "https://download.openmmlab.com/mmpose/v1/projects/rtmpose/rtmdet_nano_8xb32-100e_coco-obj365-person-05d8511e.pth "
        self.pose_config  = directoryRtmPose + 'rtmpose/body_2d_keypoint/rtmpose-m_8xb256-420e_coco-256x192.py'
        self.pose_checkpoint = "https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/rtmpose-m_simcc-aic-coco_pt-aic-coco_420e-256x192-63eb25f7_20230126.pth" 
        self.input = "webcam"
        self.output_root = ""
        self.show = True
        self.save_predictions = False
        self.device = "cuda:0"
        self.det_cat_id = 0
        self.bbox_thr = 0.3
        self.nms_thr = 0.3
        self.kpt_thr=0.3
        self.draw_heatmap = False
        self.show_kpt_idx = False
        self.skeleton_style = "mmpose"
        self.radius = 3
        self.thickness = 1
        self.show_interval = 0
        self.alpha = 0.8
        self.draw_bbox = False

        

        output_file = None
        if self.output_root:
            mmengine.mkdir_or_exist(self.output_root)
            output_file = os.path.join(self.output_root,
                                    os.path.basename(self.input))
            if self.input == 'webcam':
                output_file += '.mp4'

        if self.save_predictions:
            assert self.output_root != ''
            self.pred_save_path = f'{self.output_root}/results_' \
                f'{os.path.splitext(os.path.basename(self.input))[0]}.json'

        # build detector
        self.detector = init_detector(
            self.det_config, self.det_checkpoint, device=self.device)
        self.detector.cfg = adapt_mmdet_pipeline(self.detector.cfg)

        # build pose estimator
        self.pose_estimator = init_pose_estimator(
            self.pose_config,
            self.pose_checkpoint,
            device=self.device,
            cfg_options=dict(
                model=dict(test_cfg=dict(output_heatmaps=self.draw_heatmap))))

        # build visualizer
        self.pose_estimator.cfg.visualizer.radius = self.radius
        self.pose_estimator.cfg.visualizer.alpha = self.alpha
        self.pose_estimator.cfg.visualizer.line_width = self.thickness
        self.visualizer = VISUALIZERS.build(self.pose_estimator.cfg.visualizer)
        # the dataset_meta is loaded from the checkpoint and
        # then pass to the model in init_pose_estimator
        self.visualizer.set_dataset_meta(
            self.pose_estimator.dataset_meta, skeleton_style=self.skeleton_style)

        self.cap = cv2.VideoCapture(2)


        self.video_writer = None
        self.pred_instances_list = []
        self.frame_idx = 0               


    def process_one_image(self,
                        img,
                        detector,
                        pose_estimator,
                        visualizer=None,
                        show_interval=0):
        """Visualize predicted keypoints (and heatmaps) of one image."""

        # predict bbox
        det_result = inference_detector(detector, img)
        pred_instance = det_result.pred_instances.cpu().numpy()
        bboxes = np.concatenate(
            (pred_instance.bboxes, pred_instance.scores[:, None]), axis=1)
        bboxes = bboxes[np.logical_and(pred_instance.labels == self.det_cat_id,
                                    pred_instance.scores > self.bbox_thr)]
        bboxes = bboxes[nms(bboxes, self.nms_thr), :4]

        # predict keypoints
        pose_results = inference_topdown(pose_estimator, img, bboxes)

        # print(pose_results)
        data_samples = merge_data_samples(pose_results)

        # show the results
        if isinstance(img, str):
            img = mmcv.imread(img, channel_order='rgb')
        elif isinstance(img, np.ndarray):
            img = mmcv.bgr2rgb(img)

        if visualizer is not None:
            visualizer.add_datasample(
                'result',
                img,
                data_sample=data_samples,
                draw_gt=False,
                draw_heatmap=self.draw_heatmap,
                draw_bbox=self.draw_bbox,
                show_kpt_idx=self.show_kpt_idx,
                skeleton_style=self.skeleton_style,
                show=self.show,
                wait_time=show_interval,
                kpt_thr=self.kpt_thr)

        # if there is no instance detected, return None
        return data_samples.get('pred_instances', None)


    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

        if self.cap.isOpened():
            success, frame = self.cap.read()
            self.frame_idx += 1

            if not success:
                return

            # topdown pose estimation
            pred_instances = self.process_one_image(frame, self.detector,
                                               self.pose_estimator, self.visualizer,
                                               0.001)
            print("Left Ankle X,%:",pred_instances.keypoints[0][15][0],pred_instances.keypoint_scores[0][15],"Right Ankle X,%:",pred_instances.keypoints[0][16][0],pred_instances.keypoint_scores[0][16])

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

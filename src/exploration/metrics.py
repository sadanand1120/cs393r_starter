import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from scipy.ndimage import zoom


def is_occupied(pgm_path):
    pgm_image = Image.open(pgm_path)
    pgm_array = np.array(pgm_image)
    NUM = 210
    pgm_copy = pgm_array.copy()
    pgm_copy[pgm_array > NUM] = 0
    pgm_copy[pgm_array <= NUM] = 1
    return pgm_copy


def resize_matrix(matrix, new_shape):
    zoom_factors = [n / o for n, o in zip(new_shape, matrix.shape)]
    resized_matrix = zoom(matrix, zoom_factors, order=1)  # Using order=1 (linear interpolation)
    resized_matrix = (resized_matrix > 0.5).astype(int)
    return resized_matrix


def coverage(gt_mat, pred_mat):
    return np.sum(np.logical_and(gt_mat == 1, pred_mat == 1)) / np.sum(gt_mat == 1)


def iou(pred_mat, gt_mat):
    intersection = np.logical_and(pred_mat, gt_mat)
    union = np.logical_or(pred_mat, gt_mat)
    iou = np.sum(intersection) / np.sum(union)
    return iou


if __name__ == "__main__":
    slam_pgm_path = "maze_map_with_WFD__final.pgm"
    gt_pgm_path = "maze.pgm"
    RES_SHAPE = (400, 400)
    slam_is_occupied = resize_matrix(is_occupied(slam_pgm_path), RES_SHAPE)
    gt_is_occupied = resize_matrix(is_occupied(gt_pgm_path), RES_SHAPE)
    print("Coverage: ", round(100 * coverage(gt_is_occupied, slam_is_occupied), 2))
    print("IoU: ", round(100 * iou(slam_is_occupied, gt_is_occupied), 2))

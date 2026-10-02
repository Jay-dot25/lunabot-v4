"""Dependency-light multiclass segmentation metrics."""
from __future__ import annotations


def confusion_matrix(prediction, target, classes=9, ignore_index=0):
    matrix = [[0 for _ in range(classes)] for _ in range(classes)]
    for truth, pred in zip(target, prediction):
        truth, pred = int(truth), int(pred)
        if truth == ignore_index:
            continue
        if not (0 <= truth < classes and 0 <= pred < classes):
            raise ValueError("class ID out of range")
        matrix[truth][pred] += 1
    return matrix


def merge_confusion(left, right):
    if len(left) != len(right):
        raise ValueError("matrix sizes differ")
    return [[a + b for a, b in zip(arow, brow)] for arow, brow in zip(left, right)]


def report(matrix, names=None, hazardous=(5, 6)):
    count = len(matrix)
    names = names or [str(index) for index in range(count)]
    rows, ious = [], []
    for index in range(count):
        tp = matrix[index][index]
        fn = sum(matrix[index]) - tp
        fp = sum(row[index] for row in matrix) - tp
        precision = tp / (tp + fp) if tp + fp else 0.0
        recall = tp / (tp + fn) if tp + fn else 0.0
        iou = tp / (tp + fp + fn) if tp + fp + fn else None
        f1 = 2 * precision * recall / (precision + recall) if precision + recall else 0.0
        if iou is not None:
            ious.append(iou)
        rows.append({"id": index, "name": names[index], "iou": iou,
                     "precision": precision, "recall": recall, "f1": f1,
                     "support": tp + fn})
    hazard_fn = sum(sum(matrix[index]) - matrix[index][index] for index in hazardous)
    hazard_total = sum(sum(matrix[index]) for index in hazardous)
    return {"per_class": rows, "mean_iou": sum(ious) / len(ious) if ious else 0.0,
            "hazard_false_negative_rate": hazard_fn / hazard_total if hazard_total else 0.0,
            "confusion_matrix": matrix}

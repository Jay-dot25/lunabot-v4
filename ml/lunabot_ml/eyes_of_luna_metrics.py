"""Dependency-free confusion metrics for Phase 2 held-out evaluation."""
from __future__ import annotations

TERRAIN_NAMES = ("regolith", "bedrock", "rock_boulder", "crater")


def confusion_matrix(prediction, target, classes: int, ignore_id: int = 255) -> list[list[int]]:
    if classes <= 0:
        raise ValueError("classes must be positive")
    if len(prediction) != len(target):
        raise ValueError("prediction and target lengths differ")
    matrix = [[0 for _ in range(classes)] for _ in range(classes)]
    for truth, predicted in zip(target, prediction):
        truth, predicted = int(truth), int(predicted)
        if truth == ignore_id:
            continue
        if not 0 <= truth < classes or not 0 <= predicted < classes:
            raise ValueError("class ID out of range")
        matrix[truth][predicted] += 1
    return matrix


def merge_confusion(left: list[list[int]], right: list[list[int]]) -> list[list[int]]:
    if len(left) != len(right) or any(len(a) != len(b) for a, b in zip(left, right)):
        raise ValueError("matrix sizes differ")
    return [[a + b for a, b in zip(left_row, right_row)]
            for left_row, right_row in zip(left, right)]


def terrain_report(matrix: list[list[int]]) -> dict:
    count = len(matrix)
    if count != len(TERRAIN_NAMES) or any(len(row) != count for row in matrix):
        raise ValueError("terrain confusion matrix must be 4x4")
    rows, ious = [], []
    total, correct = 0, 0
    for index, name in enumerate(TERRAIN_NAMES):
        true_positive = matrix[index][index]
        support = sum(matrix[index])
        predicted_count = sum(row[index] for row in matrix)
        false_negative = support - true_positive
        false_positive = predicted_count - true_positive
        precision = true_positive / predicted_count if predicted_count else 0.0
        recall = true_positive / support if support else 0.0
        union = true_positive + false_positive + false_negative
        iou = true_positive / union if union else None
        f1 = (2 * precision * recall / (precision + recall)
              if precision + recall else 0.0)
        if iou is not None:
            ious.append(iou)
        total += support
        correct += true_positive
        rows.append({"id": index, "name": name, "iou": iou,
                     "precision": precision, "recall": recall,
                     "f1": f1, "support": support})
    return {
        "per_class": rows,
        "mean_iou": sum(ious) / len(ious) if ious else 0.0,
        "pixel_accuracy": correct / total if total else 0.0,
        "evaluated_pixels": total,
        "confusion_matrix": matrix,
        "missing_classes": [row["name"] for row in rows if row["support"] == 0],
    }


def shadow_report(matrix: list[list[int]]) -> dict:
    if len(matrix) != 2 or any(len(row) != 2 for row in matrix):
        raise ValueError("shadow confusion matrix must be 2x2")
    true_negative, false_positive = matrix[0]
    false_negative, true_positive = matrix[1]
    support_negative = true_negative + false_positive
    support_positive = false_negative + true_positive
    precision = true_positive / (true_positive + false_positive) if true_positive + false_positive else 0.0
    recall = true_positive / support_positive if support_positive else 0.0
    iou_denominator = true_positive + false_positive + false_negative
    iou = true_positive / iou_denominator if iou_denominator else None
    f1 = 2 * precision * recall / (precision + recall) if precision + recall else 0.0
    total = support_negative + support_positive
    return {
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "iou": iou,
        "support": support_positive,
        "negative_support": support_negative,
        "false_positive": false_positive,
        "false_negative": false_negative,
        "false_negative_rate": false_negative / support_positive if support_positive else 0.0,
        "accuracy": (true_positive + true_negative) / total if total else 0.0,
        "evaluated_pixels": total,
        "confusion_matrix": matrix,
    }

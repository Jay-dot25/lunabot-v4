#!/usr/bin/env python3
"""Train LunaBot's tiny simulation-focused RGB-D segmentation MLP.

This is an offline, reproducible supervised training program. It generates a
balanced synthetic feature dataset matching the Gazebo lunar visual/depth
regimes documented in docs/phase-5-launch-e.md, trains a 7-12-8-5 ReLU network
with SGD, evaluates a held-out split, and writes the deployable JSON weights.
No runtime status or evidence is fabricated by this tool.
"""
from __future__ import annotations

import argparse
import json
import math
import random
from pathlib import Path

LABELS = ["BEDROCK", "REGOLITH", "ROCK", "CRATER", "SHADOW"]
FEATURES = ["red", "green", "blue", "depth_norm", "row_norm", "depth_gradient", "texture"]


def clipped(rng, mean, sigma, low=0.0, high=1.0):
    return min(high, max(low, rng.gauss(mean, sigma)))


def sample(rng, label):
    # RGB values model the monochrome lunar materials. Depth, local gradient,
    # row and texture provide geometric/context evidence unavailable to RGB.
    if label == 0:  # bedrock: brighter, smooth, compact surface
        lum, tint, depth, row, grad, texture = .64, .018, .48, .70, .035, .035
    elif label == 1:  # regolith: medium albedo and visibly granular texture
        lum, tint, depth, row, grad, texture = .47, .025, .56, .72, .075, .16
    elif label == 2:  # rock: close return with a strong local depth edge
        lum, tint, depth, row, grad, texture = .39, .045, .28, .61, .63, .25
    elif label == 3:  # crater: farther/concave return with a moderate boundary
        lum, tint, depth, row, grad, texture = .34, .025, .83, .67, .31, .12
    else:  # shadow: low RGB response; geometry can still be valid
        lum, tint, depth, row, grad, texture = .075, .012, .58, .58, .10, .055
    l = clipped(rng, lum, .065)
    r = min(1.0, max(0.0, l + rng.gauss(0, tint)))
    g = min(1.0, max(0.0, l + rng.gauss(0, tint)))
    b = min(1.0, max(0.0, l + rng.gauss(0, tint)))
    return [r, g, b,
            clipped(rng, depth, .105), clipped(rng, row, .14),
            clipped(rng, grad, .085), clipped(rng, texture, .065)]


def init_matrix(rng, rows, cols):
    scale = math.sqrt(2.0 / rows)
    return [[rng.gauss(0, scale) for _ in range(cols)] for _ in range(rows)]


def matvec(x, w, b, relu=False):
    out = [b[j] + sum(x[i] * w[i][j] for i in range(len(x))) for j in range(len(b))]
    return [max(0.0, value) for value in out] if relu else out


def softmax(logits):
    peak = max(logits)
    exps = [math.exp(v - peak) for v in logits]
    total = sum(exps)
    return [v / total for v in exps]


def predict(x, params):
    h1 = matvec(x, params["w1"], params["b1"], True)
    h2 = matvec(h1, params["w2"], params["b2"], True)
    return matvec(h2, params["w3"], params["b3"]), h1, h2


def train(seed, samples_per_class, epochs, learning_rate):
    rng = random.Random(seed)
    data = [(sample(rng, label), label) for label in range(5)
            for _ in range(samples_per_class)]
    rng.shuffle(data)
    split = int(len(data) * .8)
    training, test = data[:split], data[split:]
    params = {
        "w1": init_matrix(rng, 7, 12), "b1": [0.0] * 12,
        "w2": init_matrix(rng, 12, 8), "b2": [0.0] * 8,
        "w3": init_matrix(rng, 8, 5), "b3": [0.0] * 5,
    }
    for epoch in range(epochs):
        rng.shuffle(training)
        lr = learning_rate / (1.0 + .08 * epoch)
        for x, label in training:
            logits, h1, h2 = predict(x, params)
            d3 = softmax(logits)
            d3[label] -= 1.0
            # Capture upstream gradients before mutating weights.
            d2 = [sum(params["w3"][i][j] * d3[j] for j in range(5))
                  * (1.0 if h2[i] > 0 else 0.0) for i in range(8)]
            d1 = [sum(params["w2"][i][j] * d2[j] for j in range(8))
                  * (1.0 if h1[i] > 0 else 0.0) for i in range(12)]
            for i in range(8):
                for j in range(5):
                    params["w3"][i][j] -= lr * h2[i] * d3[j]
            for j in range(5):
                params["b3"][j] -= lr * d3[j]
            for i in range(12):
                for j in range(8):
                    params["w2"][i][j] -= lr * h1[i] * d2[j]
            for j in range(8):
                params["b2"][j] -= lr * d2[j]
            for i in range(7):
                for j in range(12):
                    params["w1"][i][j] -= lr * x[i] * d1[j]
            for j in range(12):
                params["b1"][j] -= lr * d1[j]
    correct = 0
    confusion = [[0] * 5 for _ in range(5)]
    for x, label in test:
        logits, _, _ = predict(x, params)
        pred = max(range(5), key=lambda i: logits[i])
        confusion[label][pred] += 1
        correct += pred == label
    return params, correct / max(1, len(test)), confusion, len(training), len(test)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="models/terrain_mlp_v1.json")
    parser.add_argument("--seed", type=int, default=2504)
    parser.add_argument("--samples-per-class", type=int, default=1000)
    parser.add_argument("--epochs", type=int, default=18)
    args = parser.parse_args()
    params, accuracy, confusion, train_n, test_n = train(
        args.seed, args.samples_per_class, args.epochs, .018)
    artifact = {
        "format": "lunabot_mlp_v1",
        "architecture": [7, 12, 8, 5],
        "activation": "relu",
        "labels": LABELS,
        "features": FEATURES,
        "training": {
            "method": "supervised synthetic simulation-domain SGD",
            "seed": args.seed,
            "train_samples": train_n,
            "test_samples": test_n,
            "epochs": args.epochs,
            "held_out_accuracy": round(accuracy, 6),
            "confusion_matrix": confusion,
        },
        **params,
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(artifact, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {output}: held-out accuracy={accuracy:.3%}")
    if accuracy < .90:
        raise SystemExit("held-out accuracy below 90%")


if __name__ == "__main__":
    main()

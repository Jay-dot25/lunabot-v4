"""Paired, seedable lunar image augmentation preserving semantic labels."""
from __future__ import annotations
import random


def augment(image, mask, seed, enabled=True):
    if not enabled:
        return image, mask
    import numpy as np
    from PIL import Image, ImageDraw, ImageEnhance, ImageFilter, ImageOps
    rng = random.Random(seed)
    image = ImageEnhance.Brightness(image).enhance(rng.uniform(0.65, 1.35))
    image = ImageEnhance.Contrast(image).enhance(rng.uniform(0.7, 1.3))
    gamma = rng.uniform(0.75, 1.3)
    image = image.point(lambda value: round(255 * (value / 255) ** gamma))
    if rng.random() < 0.35:  # sensor noise
        array = np.asarray(image, dtype=np.int16)
        noise = np.random.default_rng(seed).normal(0, rng.uniform(1, 8), array.shape)
        image = Image.fromarray(np.clip(array + noise, 0, 255).astype(np.uint8), "RGB")
    if rng.random() < 0.3:
        image = image.filter(ImageFilter.GaussianBlur(rng.uniform(0.1, 1.2)))
    if rng.random() < 0.25:  # exposure clipping
        array = np.asarray(image)
        image = Image.fromarray(np.clip(array, rng.randint(0, 20), rng.randint(210, 255)).astype(np.uint8), "RGB")
    if rng.random() < 0.25:  # synthetic cast shadow affects image, never labels
        overlay = Image.new("L", image.size, 0); draw = ImageDraw.Draw(overlay)
        x = rng.randint(0, image.width); width = rng.randint(max(1, image.width // 8), max(2, image.width // 2))
        draw.polygon([(x, 0), (x + width, 0), (x, image.height), (x - width, image.height)], fill=rng.randint(45, 130))
        dark = ImageEnhance.Brightness(image).enhance(rng.uniform(0.25, 0.65)); image = Image.composite(dark, image, overlay)
    if rng.random() < 0.3:  # mild paired affine perspective proxy
        shear = rng.uniform(-0.06, 0.06)
        coefficients = (1, shear, -shear * image.height / 2, shear, 1, -shear * image.width / 2)
        image = image.transform(image.size, Image.Transform.AFFINE, coefficients, Image.Resampling.BILINEAR)
        mask = mask.transform(mask.size, Image.Transform.AFFINE, coefficients, Image.Resampling.NEAREST)
    if rng.random() < 0.5:
        image, mask = ImageOps.mirror(image), ImageOps.mirror(mask)
    return image, mask

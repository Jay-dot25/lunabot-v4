"""Masked multitask losses for independently annotated terrain and shadow masks."""
from __future__ import annotations


def multitask_loss(terrain_logits, shadow_logits, terrain_target, shadow_target,
                   terrain_dice_weight=0.5, shadow_loss_weight=1.0,
                   shadow_dice_weight=0.5, ignore_id=255):
    """Return total loss and detached component scalars; 255 pixels are ignored."""
    try:
        import torch
        import torch.nn.functional as F
    except ImportError as exc:
        raise RuntimeError("PyTorch is required for Eyes of Luna losses") from exc

    if terrain_logits.ndim != 4 or terrain_logits.shape[1] != 4:
        raise ValueError("terrain logits must have shape Bx4xHxW")
    if shadow_logits.ndim != 4 or shadow_logits.shape[1] != 1:
        raise ValueError("shadow logits must have shape Bx1xHxW")
    expected = (terrain_logits.shape[0], terrain_logits.shape[2], terrain_logits.shape[3])
    if tuple(terrain_target.shape) != expected or tuple(shadow_target.shape) != expected:
        raise ValueError("terrain and shadow targets must match the logits' BxHxW shape")

    terrain_valid = terrain_target != ignore_id
    shadow_valid = shadow_target != ignore_id
    if torch.any(terrain_valid & ((terrain_target < 0) | (terrain_target >= 4))):
        raise ValueError("terrain target contains an invalid class ID")
    if torch.any(shadow_valid & ((shadow_target < 0) | (shadow_target > 1))):
        raise ValueError("shadow target must contain only 0, 1, or ignore_id")

    if terrain_valid.any():
        terrain_ce = F.cross_entropy(
            terrain_logits, terrain_target.long(), ignore_index=int(ignore_id))
    else:
        terrain_ce = terrain_logits.sum() * 0.0
    safe_terrain = terrain_target.long().masked_fill(~terrain_valid, 0)
    terrain_one_hot = F.one_hot(safe_terrain, num_classes=4).permute(0, 3, 1, 2)
    terrain_mask = terrain_valid.unsqueeze(1).to(dtype=terrain_logits.dtype)
    terrain_probabilities = terrain_logits.softmax(dim=1) * terrain_mask
    terrain_one_hot = terrain_one_hot.to(dtype=terrain_logits.dtype) * terrain_mask
    terrain_intersection = (terrain_probabilities * terrain_one_hot).sum((0, 2, 3))
    terrain_denominator = terrain_probabilities.sum((0, 2, 3)) + terrain_one_hot.sum((0, 2, 3))
    terrain_dice = 1.0 - ((2.0 * terrain_intersection + 1e-6)
                          / (terrain_denominator + 1e-6)).mean()

    shadow_values = shadow_logits[:, 0]
    shadow_binary = (shadow_target == 1).to(dtype=shadow_logits.dtype)
    shadow_mask = shadow_valid.to(dtype=shadow_logits.dtype)
    if shadow_valid.any():
        pixel_bce = F.binary_cross_entropy_with_logits(
            shadow_values, shadow_binary, reduction="none")
        shadow_bce = (pixel_bce * shadow_mask).sum() / shadow_mask.sum()
    else:
        shadow_bce = shadow_logits.sum() * 0.0
    shadow_probabilities = shadow_values.sigmoid() * shadow_mask
    shadow_target_masked = shadow_binary * shadow_mask
    shadow_intersection = (shadow_probabilities * shadow_target_masked).sum()
    shadow_denominator = shadow_probabilities.sum() + shadow_target_masked.sum()
    shadow_dice = 1.0 - (2.0 * shadow_intersection + 1e-6) / (shadow_denominator + 1e-6)

    total = (terrain_ce + float(terrain_dice_weight) * terrain_dice
             + float(shadow_loss_weight)
             * (shadow_bce + float(shadow_dice_weight) * shadow_dice))
    components = {
        "terrain_cross_entropy": terrain_ce.detach(),
        "terrain_dice": terrain_dice.detach(),
        "shadow_binary_cross_entropy": shadow_bce.detach(),
        "shadow_dice": shadow_dice.detach(),
        "total": total.detach(),
    }
    return total, components

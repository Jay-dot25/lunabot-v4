"""Safety-aware cross-entropy and Dice segmentation loss."""
def build_loss(class_weights, dice_weight=0.5, ignore_index=0):
    try:
        import torch
        import torch.nn.functional as F
    except ImportError as exc:
        raise RuntimeError("PyTorch is required") from exc
    weights = torch.tensor(class_weights, dtype=torch.float32)
    def loss(logits, target):
        ce = F.cross_entropy(logits, target, weight=weights.to(logits.device), ignore_index=ignore_index)
        valid = target != ignore_index
        safe_target = target.masked_fill(~valid, 0)
        one_hot = F.one_hot(safe_target, logits.shape[1]).permute(0, 3, 1, 2).float()
        mask = valid.unsqueeze(1)
        probabilities = logits.softmax(1) * mask
        one_hot = one_hot * mask
        intersection = (probabilities * one_hot).sum((0, 2, 3))
        denominator = probabilities.sum((0, 2, 3)) + one_hot.sum((0, 2, 3))
        dice = 1.0 - ((2 * intersection + 1e-6) / (denominator + 1e-6)).mean()
        return ce + float(dice_weight) * dice
    return loss

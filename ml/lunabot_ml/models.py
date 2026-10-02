"""Small segmentation architectures with a stable construction contract."""
from __future__ import annotations


def build_model(config):
    try:
        import torch
        from torch import nn
        import torch.nn.functional as F
    except ImportError as exc:
        raise RuntimeError("PyTorch is required; install ml/requirements.txt") from exc
    architecture = config["model"]["architecture"]
    channels = int(config["model"].get("base_channels", 32))
    classes = len(config["classes"])

    class Block(nn.Sequential):
        def __init__(self, incoming, outgoing, dilation=1):
            super().__init__(nn.Conv2d(incoming, outgoing, 3, padding=dilation,
                                      dilation=dilation, bias=False),
                             nn.BatchNorm2d(outgoing), nn.ReLU(inplace=True),
                             nn.Conv2d(outgoing, outgoing, 3, padding=1, bias=False),
                             nn.BatchNorm2d(outgoing), nn.ReLU(inplace=True))

    class UNet(nn.Module):
        def __init__(self):
            super().__init__()
            self.e1, self.e2 = Block(3, channels), Block(channels, channels * 2)
            self.bridge = Block(channels * 2, channels * 4)
            self.d2, self.d1 = Block(channels * 6, channels * 2), Block(channels * 3, channels)
            self.head = nn.Conv2d(channels, classes, 1)
        def forward(self, x):
            a = self.e1(x); b = self.e2(F.max_pool2d(a, 2))
            c = self.bridge(F.max_pool2d(b, 2))
            d = self.d2(torch.cat((F.interpolate(c, b.shape[-2:], mode="bilinear", align_corners=False), b), 1))
            d = self.d1(torch.cat((F.interpolate(d, a.shape[-2:], mode="bilinear", align_corners=False), a), 1))
            return self.head(d)

    class DeepLabV3Plus(nn.Module):
        def __init__(self):
            super().__init__()
            self.stem = nn.Sequential(Block(3, channels), nn.MaxPool2d(2), Block(channels, channels * 2))
            self.aspp = nn.ModuleList([Block(channels * 2, channels, rate) for rate in (1, 2, 4)])
            self.head = nn.Conv2d(channels * 3, classes, 1)
        def forward(self, x):
            size = x.shape[-2:]; features = self.stem(x)
            return F.interpolate(self.head(torch.cat([branch(features) for branch in self.aspp], 1)), size,
                                 mode="bilinear", align_corners=False)

    class SegFormerB0(nn.Module):
        def __init__(self):
            super().__init__()
            encoder = nn.TransformerEncoderLayer(channels, 4, channels * 4, batch_first=True)
            self.patch = nn.Conv2d(3, channels, 7, stride=4, padding=3)
            self.encoder = nn.TransformerEncoder(encoder, 2)
            self.head = nn.Conv2d(channels, classes, 1)
        def forward(self, x):
            size = x.shape[-2:]; x = self.patch(x); h, w = x.shape[-2:]
            x = self.encoder(x.flatten(2).transpose(1, 2)).transpose(1, 2).reshape(-1, channels, h, w)
            return F.interpolate(self.head(x), size, mode="bilinear", align_corners=False)

    return {"unet": UNet, "deeplabv3plus": DeepLabV3Plus,
            "segformer_b0": SegFormerB0}[architecture]()

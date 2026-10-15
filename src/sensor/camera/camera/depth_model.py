"""RegNetY-800MF + FPN + residual U-Net for metric monocular depth.

Inputs
------
image:        [B, 3, H, W]
camera:       [B, 4] = [fx/W, fy/H, cx/W, cy/H]
camera_valid: [B, 1], 1 when camera is known, otherwise 0

Output
------
depth: [B, 1, H, W] in metres
"""
from __future__ import annotations

from collections import OrderedDict
from typing import Dict, List, Optional

import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision.models import RegNet_Y_800MF_Weights, regnet_y_800mf
from torchvision.models.feature_extraction import create_feature_extractor


class ConvBNAct(nn.Sequential):
    def __init__(self, in_ch: int, out_ch: int, kernel_size: int = 3, stride: int = 1):
        super().__init__(
            nn.Conv2d(in_ch, out_ch, kernel_size, stride=stride,
                      padding=kernel_size // 2, bias=False),
            nn.BatchNorm2d(out_ch),
            nn.SiLU(inplace=True),
        )


class ResidualDecoderBlock(nn.Module):
    def __init__(self, in_ch: int, skip_ch: int, out_ch: int):
        super().__init__()
        merged = in_ch + skip_ch
        self.conv1 = ConvBNAct(merged, out_ch)
        self.conv2 = nn.Sequential(
            nn.Conv2d(out_ch, out_ch, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_ch),
        )
        self.proj = nn.Sequential(
            nn.Conv2d(merged, out_ch, 1, bias=False),
            nn.BatchNorm2d(out_ch),
        )
        self.act = nn.SiLU(inplace=True)

    def forward(self, x: torch.Tensor, skip: torch.Tensor) -> torch.Tensor:
        x = F.interpolate(x, size=skip.shape[-2:], mode="bilinear", align_corners=False)
        x = torch.cat((x, skip), dim=1)
        return self.act(self.conv2(self.conv1(x)) + self.proj(x))


class FPN(nn.Module):
    def __init__(self, in_channels: List[int], out_channels: int = 128):
        super().__init__()
        self.lateral = nn.ModuleList(
            nn.Conv2d(ch, out_channels, 1) for ch in in_channels
        )
        self.smooth = nn.ModuleList(
            ConvBNAct(out_channels, out_channels) for _ in in_channels
        )

    def forward(self, feats: List[torch.Tensor]) -> List[torch.Tensor]:
        laterals = [layer(feat) for layer, feat in zip(self.lateral, feats)]
        results: List[torch.Tensor] = [laterals[0]] * len(laterals)
        top = laterals[-1]
        results[-1] = self.smooth[-1](top)
        for i in range(len(laterals) - 2, -1, -1):
            top = F.interpolate(top, size=laterals[i].shape[-2:], mode="nearest")
            top = top + laterals[i]
            results[i] = self.smooth[i](top)
        return results


class RegNetFPNUNetDepth(nn.Module):
    def __init__(
        self,
        fpn_channels: int = 128,
        pretrained: bool = True,
        min_depth: float = 1e-3,
        max_depth: float = 80.0,
        use_camera_calibration: bool = False,
        camera_film_strength: float = 0.1,
        camera_mode: str = "film",
        canonical_focal_length: float = 1000.0,
    ):
        super().__init__()
        self.min_depth = float(min_depth)
        self.max_depth = float(max_depth)
        self.use_camera_calibration = bool(use_camera_calibration)
        self.camera_film_strength = float(camera_film_strength)
        self.camera_mode = str(camera_mode).lower()
        self.canonical_focal_length = float(canonical_focal_length)
        if self.camera_mode not in {"film", "canonical"}:
            raise ValueError("camera_mode must be 'film' or 'canonical'")
        if self.canonical_focal_length <= 0.0:
            raise ValueError("canonical_focal_length must be positive")
        if self.camera_film_strength < 0.0:
            raise ValueError("camera_film_strength must be non-negative")
        self.last_camera_stats: Dict[str, float] = {}

        weights = RegNet_Y_800MF_Weights.DEFAULT if pretrained else None
        backbone = regnet_y_800mf(weights=weights)
        return_nodes = OrderedDict({
            "stem": "c1",
            "trunk_output.block1": "c2",
            "trunk_output.block2": "c3",
            "trunk_output.block3": "c4",
            "trunk_output.block4": "c5",
        })
        self.encoder = create_feature_extractor(backbone, return_nodes=return_nodes)

        was_training = self.encoder.training
        self.encoder.eval()
        with torch.no_grad():
            output: Dict[str, torch.Tensor] = self.encoder(torch.zeros(1, 3, 480, 640))
        self.encoder.train(was_training)
        channels = [output[name].shape[1] for name in ("c1", "c2", "c3", "c4", "c5")]

        self.fpn = FPN(channels, fpn_channels)
        if self.use_camera_calibration and self.camera_mode == "film":
            self.camera_mlp = nn.Sequential(
                nn.Linear(4, 32),
                nn.SiLU(inplace=True),
                nn.Linear(32, fpn_channels * 2),
            )
            # Start as the RGB-only model; learn camera modulation gradually.
            nn.init.zeros_(self.camera_mlp[-1].weight)
            nn.init.zeros_(self.camera_mlp[-1].bias)

        self.dec4 = ResidualDecoderBlock(fpn_channels, fpn_channels, fpn_channels)
        self.dec3 = ResidualDecoderBlock(fpn_channels, fpn_channels, fpn_channels)
        self.dec2 = ResidualDecoderBlock(fpn_channels, fpn_channels, fpn_channels)
        self.dec1 = ResidualDecoderBlock(fpn_channels, fpn_channels, fpn_channels)
        self.refine = nn.Sequential(
            ConvBNAct(fpn_channels, fpn_channels // 2),
            ConvBNAct(fpn_channels // 2, fpn_channels // 2),
        )
        self.depth_head = nn.Conv2d(fpn_channels // 2, 1, 1)

    def forward(
        self,
        x: torch.Tensor,
        camera: Optional[torch.Tensor] = None,
        camera_valid: Optional[torch.Tensor] = None,
    ) -> torch.Tensor:
        feature_dict = self.encoder(x)
        feats = [feature_dict[name] for name in ("c1", "c2", "c3", "c4", "c5")]
        p1, p2, p3, p4, p5 = self.fpn(feats)

        self.last_camera_stats = {}
        if self.use_camera_calibration and camera is None:
            raise ValueError("camera [B,4] is required when camera calibration is enabled")
        if self.use_camera_calibration and camera is not None:
            if camera.ndim != 2 or camera.shape[1] != 4:
                raise ValueError(f"camera must have shape [B,4], got {tuple(camera.shape)}")
        if self.use_camera_calibration and self.camera_mode == "film":
            if camera_valid is None:
                camera_valid = torch.ones((camera.shape[0], 1), device=camera.device,
                                          dtype=camera.dtype)
            else:
                camera_valid = camera_valid.to(device=camera.device, dtype=camera.dtype)
                if camera_valid.numel() != camera.shape[0]:
                    raise ValueError(
                        "camera_valid must have shape [B] or [B,1], "
                        f"got {tuple(camera_valid.shape)} for batch size {camera.shape[0]}"
                    )
                camera_valid = camera_valid.reshape(camera.shape[0], 1)

            raw_scale, raw_bias = self.camera_mlp(camera).chunk(2, dim=1)
            scale = self.camera_film_strength * torch.tanh(raw_scale)
            bias = self.camera_film_strength * torch.tanh(raw_bias)
            # Mask before adding spatial dimensions so invalid calibration is
            # exactly equivalent to RGB-only FPN features.
            scale = scale * camera_valid
            bias = bias * camera_valid
            p5_before = p5
            p5 = p5 * (1.0 + scale.unsqueeze(-1).unsqueeze(-1))
            p5 = p5 + bias.unsqueeze(-1).unsqueeze(-1)
            self.last_camera_stats = {
                "scale_min": float(scale.detach().min().item()),
                "scale_max": float(scale.detach().max().item()),
                "scale_mean": float(scale.detach().mean().item()),
                "scale_std": float(scale.detach().float().std(unbiased=False).item()),
                "bias_min": float(bias.detach().min().item()),
                "bias_max": float(bias.detach().max().item()),
                "bias_mean": float(bias.detach().mean().item()),
                "bias_std": float(bias.detach().float().std(unbiased=False).item()),
                "p5_norm_before": float(p5_before.detach().float().norm().item()),
                "p5_norm_after": float(p5.detach().float().norm().item()),
            }

        y = self.dec4(p5, p4)
        y = self.dec3(y, p3)
        y = self.dec2(y, p2)
        y = self.dec1(y, p1)
        y = F.interpolate(y, size=x.shape[-2:], mode="bilinear", align_corners=False)
        y = self.refine(y)
        depth = F.softplus(self.depth_head(y)) + self.min_depth
        # Canonical depth can be larger than metric depth by the focal-length
        # normalization ratio. Keep the existing metric clamp for film/RGB
        # models and allow a bounded canonical range for stable training.
        depth_max = self.max_depth if self.camera_mode != "canonical" else self.max_depth * 4.0
        return depth.clamp(self.min_depth, depth_max)


if __name__ == "__main__":
    model = RegNetFPNUNetDepth(pretrained=False, use_camera_calibration=True)
    image = torch.randn(2, 3, 480, 640)
    camera = torch.tensor([[0.8, 1.1, 0.5, 0.5], [0.0, 0.0, 0.0, 0.0]])
    camera_valid = torch.tensor([[1.0], [0.0]])
    depth = model(image, camera, camera_valid)
    print("image", tuple(image.shape), "depth", tuple(depth.shape))

"""Metric depth inference using the architecture and settings saved in best.pt."""
from pathlib import Path

import numpy as np
from PIL import Image
import torch
import torch.nn.functional as F
from torchvision.transforms import functional as TF

from .depth_model import RegNetFPNUNetDepth


class DepthEstimate:
    def __init__(self, checkpoint, device="cuda"):
        checkpoint = Path(checkpoint).expanduser()
        saved = torch.load(checkpoint, map_location="cpu", weights_only=True)
        args = saved.get("args") or saved.get("config") or {}
        self.size = tuple(args.get("image_size", args.get("size", (480, 640))))
        self.calibrated = bool(args.get("use_camera_calibration", args.get("camera_calibration", False)))
        self.mode = args.get("camera_mode", "film")
        self.canonical_focal = float(args.get("canonical_focal_length", 1000.0))
        # Small image preprocessing operations should not fan out across all CPUs.
        torch.set_num_threads(1)
        self.device = torch.device(device)
        if self.device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA unavailable; use device:=cpu or enable Docker GPU access")
        self.model = RegNetFPNUNetDepth(
            pretrained=False, fpn_channels=int(args.get("fpn_channels", 128)),
            min_depth=float(args.get("min_depth", 0.001)),
            max_depth=float(args.get("max_depth", 80.0)),
            use_camera_calibration=self.calibrated, camera_mode=self.mode,
            camera_film_strength=float(args.get("camera_film_strength", 0.1)),
            canonical_focal_length=self.canonical_focal)
        state = saved.get("model", saved.get("state_dict", saved))
        state = {k.removeprefix("module.").removeprefix("model."): v
                 for k, v in state.items() if torch.is_tensor(v)}
        self.model.load_state_dict(state, strict=True)
        self.model.to(self.device).eval()

    @torch.inference_mode()
    def predict(self, rgb, intrinsics=None):
        h, w = rgb.shape[:2]
        image = TF.resize(Image.fromarray(rgb), self.size,
                          interpolation=TF.InterpolationMode.BILINEAR)
        image = TF.normalize(TF.to_tensor(image),
                             [0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        kwargs = {}
        if self.calibrated:
            if intrinsics is None:
                raise ValueError("This checkpoint requires calibrated camera intrinsics")
            fx, fy, cx, cy = intrinsics
            kwargs = dict(camera=torch.tensor([[fx/w, fy/h, cx/w, cy/h]], dtype=torch.float32, device=self.device),
                          camera_valid=torch.ones((1, 1), device=self.device))
        prediction = self.model(image.unsqueeze(0).to(self.device), **kwargs).float()
        if self.mode == "canonical":
            if intrinsics is None:
                raise ValueError("Canonical depth requires camera intrinsics")
            prediction *= (intrinsics[0]/w*self.size[1] + intrinsics[1]/h*self.size[0]) / (2*self.canonical_focal)
        prediction = F.interpolate(prediction, size=(h, w), mode="bilinear", align_corners=False)
        depth = prediction[0, 0].cpu().numpy().astype(np.float32)
        depth[~np.isfinite(depth) | (depth <= 0)] = 0
        return depth

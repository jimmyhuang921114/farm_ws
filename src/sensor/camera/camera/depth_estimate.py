# depth_estimate.py
from contextlib import nullcontext
from pathlib import Path
from typing import Callable, Mapping, Optional, Tuple

import numpy as np
import torch
import torch.nn.functional as F

from models.model_regnet800mf_fpn_unet import RegNetFPNUNetDepth
import evaluate_kitti_camera_depth as base


PreprocessResult = Tuple[
    torch.Tensor,          # image: [1, 3, H, W]
    torch.Tensor,          # camera: [1, 4]
    torch.Tensor,          # camera_valid: [1]
    Optional[torch.Tensor] # canonical_to_metric_scale
]

PreprocessFn = Callable[
    [np.ndarray, Mapping[str, float]],
    PreprocessResult,
]


class DepthEstimate:
    """載入一次深度模型，供相機串流逐張呼叫推論。"""

    def __init__(
        self,
        checkpoint: str,
        preprocess_fn: PreprocessFn,
        device: str = "cuda",
        camera_mode: str = "film",
        fpn_channels: int = 128,
        min_depth: float = 0.001,
        max_depth: float = 80.0,
        camera_film_strength: float = 0.1,
        canonical_focal_length: float = 1000.0,
        use_amp: bool = True,
    ) -> None:
        if camera_mode not in ("film", "canonical"):
            raise ValueError("camera_mode 必須是 'film' 或 'canonical'")

        if device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError(
                f"指定了 device={device}，但目前 CUDA 不可用"
            )

        self.device = torch.device(device)
        self.camera_mode = camera_mode
        self.min_depth = float(min_depth)
        self.max_depth = float(max_depth)
        self.preprocess_fn = preprocess_fn
        self.use_amp = bool(use_amp and self.device.type == "cuda")

        self.model = RegNetFPNUNetDepth(
            fpn_channels=fpn_channels,
            pretrained=False,
            min_depth=self.min_depth,
            max_depth=self.max_depth,
            use_camera_calibration=True,
            camera_film_strength=camera_film_strength,
            camera_mode=camera_mode,
            canonical_focal_length=canonical_focal_length,
        )

        checkpoint_path = Path(checkpoint).expanduser().resolve()
        if not checkpoint_path.is_file():
            raise FileNotFoundError(f"找不到 checkpoint：{checkpoint_path}")

        # 沿用你的評估程式所使用的 checkpoint 載入方式。
        state_dict = base.load_checkpoint_state(checkpoint_path)
        missing, unexpected = self.model.load_state_dict(
            state_dict,
            strict=False,
        )

        camera_missing = [
            key for key in missing if key.startswith("camera_mlp.")
        ]
        if camera_mode == "film" and camera_missing:
            raise RuntimeError(
                "film 模式需要訓練過的 camera_mlp 權重，"
                f"但 checkpoint 缺少：{camera_missing}"
            )

        if missing:
            print(f"[WARN] checkpoint 缺少權重（前 8 項）：{missing[:8]}")
        if unexpected:
            print(f"[WARN] checkpoint 多出權重（前 8 項）：{unexpected[:8]}")

        self.model.to(self.device)
        self.model.eval()

    @torch.inference_mode()
    def predict(
        self,
        image_bgr: np.ndarray,
        intrinsics: Mapping[str, float],
    ) -> np.ndarray:
        """
        Args:
            image_bgr: OpenCV BGR 影像，形狀 [H, W, 3]。
            intrinsics: 相機內參，包含 fx、fy、cx、cy。

        Returns:
            float32 深度圖，形狀 [H, W]，單位為公尺。
        """
        if image_bgr is None or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
            raise ValueError("image_bgr 必須是有效的 [H, W, 3] 影像")

        original_height, original_width = image_bgr.shape[:2]

        # 必須和訓練／評估使用相同的色彩順序、resize、normalize、
        # 相機參數格式及 camera_valid 計算方式。
        image, camera, camera_valid, canonical_scale = self.preprocess_fn(
            image_bgr,
            intrinsics,
        )

        image = image.to(self.device, dtype=torch.float32)
        camera = camera.to(self.device, dtype=torch.float32)
        camera_valid = camera_valid.to(self.device, dtype=torch.bool)

        if image.ndim == 3:
            image = image.unsqueeze(0)
        if camera.ndim == 1:
            camera = camera.unsqueeze(0)
        if camera_valid.ndim == 0:
            camera_valid = camera_valid.unsqueeze(0)

        if image.shape[0] != 1:
            raise ValueError(f"目前 predict() 一次處理一張影像，收到 batch={image.shape[0]}")

        amp_context = (
            torch.autocast(device_type="cuda", dtype=torch.float16)
            if self.use_amp
            else nullcontext()
        )

        with amp_context:
            prediction = self.model(
                image,
                camera=camera,
                camera_valid=camera_valid,
            )

        if self.camera_mode == "canonical":
            if canonical_scale is None:
                raise ValueError(
                    "canonical 模式需要 preprocess_fn 回傳 "
                    "canonical_to_metric_scale"
                )

            scale = torch.as_tensor(
                canonical_scale,
                dtype=prediction.dtype,
                device=self.device,
            ).reshape(-1, 1, 1, 1)

            prediction = prediction * scale

        prediction = prediction.float()

        # 還原到相機原始解析度，讓輸出能和原始 RGB 對齊。
        if prediction.shape[-2:] != (original_height, original_width):
            prediction = F.interpolate(
                prediction,
                size=(original_height, original_width),
                mode="bilinear",
                align_corners=False,
            )

        prediction = prediction.clamp(self.min_depth, self.max_depth)

        return prediction[0, 0].cpu().numpy().astype(np.float32)
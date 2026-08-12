#!/usr/bin/env python3
"""CUDA dual-fisheye stitcher and EquiLib panorama/perspective publisher."""

from __future__ import annotations

import math
from typing import Any

import numpy as np
import rclpy
import torch
import torch.nn.functional as torch_f
from cv_bridge import CvBridge
from equilib import Equi2Equi, Equi2Pers
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class EquiLibStitcher(Node):
    """Stitch an Insta360 dual-fisheye frame and project it with EquiLib."""

    def __init__(self) -> None:
        super().__init__("equilib_stitcher")
        self._declare_parameters()
        self.bridge = CvBridge()
        self.device = self._select_device()
        self.stitch_grid: torch.Tensor | None = None
        self.input_shape: tuple[int, int] | None = None
        self.maps_dirty = True
        self.transforms_key: tuple[Any, ...] | None = None
        self.equi_rotation: Equi2Equi | None = None
        self.perspective_projection: Equi2Pers | None = None

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            Image, "/dual_fisheye/image", self.image_callback, qos
        )
        self.equirectangular_publisher = self.create_publisher(
            Image, "/equirectangular/image", qos
        )
        self.perspective_publisher = self.create_publisher(
            Image, "/perspective/image", qos
        )
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.get_logger().info(
            f"EquiLib stitcher using {self.device.type.upper()} "
            f"(CUDA available: {torch.cuda.is_available()})"
        )

    def _select_device(self) -> torch.device:
        """Use CUDA only when this PyTorch build supports the installed GPU."""
        if not self.get_parameter("gpu").value:
            return torch.device("cpu")
        if not torch.cuda.is_available():
            self.get_logger().warning("CUDA requested but unavailable; using CPU")
            return torch.device("cpu")

        capability = torch.cuda.get_device_capability()
        architecture = f"sm_{capability[0]}{capability[1]}"
        supported_architectures = set(torch.cuda.get_arch_list())
        if architecture not in supported_architectures:
            supported = ", ".join(sorted(supported_architectures)) or "none"
            self.get_logger().warning(
                f"CUDA device capability {architecture} is not supported by this PyTorch build "
                f"({supported}); using CPU"
            )
            return torch.device("cpu")
        return torch.device("cuda")

    def _declare_parameters(self) -> None:
        defaults: dict[str, Any] = {
            "cx_offset": 0.0,
            "cy_offset": 0.0,
            "crop_size": 1152,
            "translation": [0.0, 0.0, -0.105],
            "rotation_deg": [-0.5, 0.0, 1.1],
            "gpu": True,
            "out_width": 2304,
            "out_height": 1152,
            "equi_roll_deg": 0.0,
            "equi_pitch_deg": 0.0,
            "equi_yaw_deg": 0.0,
            "fisheye_fov_deg": 195.0,
            "publish_equirectangular": True,
            "publish_perspective": False,
            "perspective_width": 1280,
            "perspective_height": 720,
            "perspective_fov_x": 100.0,
            "perspective_roll_deg": 0.0,
            "perspective_pitch_deg": 0.0,
            "perspective_yaw_deg": 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def parameter_callback(self, parameters: list[Any]) -> SetParametersResult:
        map_parameters = {
            "cx_offset", "cy_offset", "crop_size", "translation", "rotation_deg",
            "out_width", "out_height", "gpu", "fisheye_fov_deg"
        }
        if any(parameter.name in map_parameters for parameter in parameters):
            self.maps_dirty = True
        return SetParametersResult(successful=True)

    def _read_parameters(self) -> dict[str, Any]:
        return {name: self.get_parameter(name).value for name in self._parameter_names}

    @property
    def _parameter_names(self) -> tuple[str, ...]:
        return (
            "cx_offset", "cy_offset", "crop_size", "translation", "rotation_deg", "gpu",
            "out_width", "out_height", "equi_roll_deg", "equi_pitch_deg", "equi_yaw_deg",
            "fisheye_fov_deg",
            "publish_equirectangular", "publish_perspective", "perspective_width",
            "perspective_height", "perspective_fov_x", "perspective_roll_deg",
            "perspective_pitch_deg", "perspective_yaw_deg",
        )

    def _rebuild_stitch_grid(self, source_height: int, source_width: int) -> None:
        params = self._read_parameters()
        if source_width % 2:
            raise ValueError("dual-fisheye image width must be even")
        lens_width = source_width // 2
        crop_size = int(params["crop_size"])
        if crop_size <= 0 or crop_size > min(source_height, lens_width):
            raise ValueError(
                f"crop_size={crop_size} must be within the {lens_width}x{source_height} lens image"
            )

        y_offset = (source_height - crop_size) // 2
        x_offset = (lens_width - crop_size) // 2
        output_height = int(params["out_height"])
        output_width = int(params["out_width"])

        # Create output coordinate meshgrid directly on GPU
        y, x = torch.meshgrid(
            torch.arange(output_height, device=self.device, dtype=torch.float32),
            torch.arange(output_width, device=self.device, dtype=torch.float32),
            indexing="ij",
        )

        longitude = (x / output_width) * (2.0 * math.pi) - math.pi
        latitude = (y / output_height) * math.pi - (math.pi / 2.0)

        cos_lat = torch.cos(latitude)
        sin_lat = torch.sin(latitude)
        cos_lon = torch.cos(longitude)
        sin_lon = torch.sin(longitude)

        orig_x = cos_lat * sin_lon
        orig_y = sin_lat

        x_val = orig_y
        y_val = -orig_x
        z_val = cos_lat * cos_lon

        front = z_val >= 0.0

        roll, pitch, yaw = [math.radians(value) for value in params["rotation_deg"]]
        rotation = self._rotation_matrix(roll, pitch, yaw)
        translation = torch.tensor(params["translation"], device=self.device, dtype=torch.float32)

        points = torch.stack((x_val, y_val, z_val), dim=-1)
        transformed = points @ rotation.T + translation

        final_x = torch.where(front, x_val, -transformed[..., 0])
        final_y = torch.where(front, y_val, transformed[..., 1])
        final_z = torch.where(front, z_val, transformed[..., 2])

        radius = torch.sqrt(final_x.square() + final_y.square()).clamp_min_(1e-6)
        theta = torch.atan2(radius, final_z.abs())

        half_fov = math.radians(float(params["fisheye_fov_deg"])) / 2.0
        fisheye_radius = (theta / half_fov) * (crop_size / 2.0)

        cx = crop_size / 2.0 + float(params["cx_offset"])
        cy = crop_size / 2.0 + float(params["cy_offset"])
        u = cx + (final_x / radius) * fisheye_radius
        v = cy + (final_y / radius) * fisheye_radius

        # Sensor 90-degree lens rotation integrated into CUDA map
        rot_u = torch.where(front, (crop_size - 1.0) - v, v)
        rot_v = torch.where(front, u, (crop_size - 1.0) - u)

        source_x = torch.where(front, lens_width + x_offset + rot_u, x_offset + rot_u)
        source_y = y_offset + rot_v

        # Correct normalization relative to full source tensor frame
        grid_x = source_x / (source_width - 1) * 2.0 - 1.0
        grid_y = source_y / (source_height - 1) * 2.0 - 1.0
        self.stitch_grid = torch.stack((grid_x, grid_y), dim=-1).unsqueeze(0)
        self.input_shape = (source_height, source_width)
        self.maps_dirty = False
        self.get_logger().info(
            f"Built {self.device.type.upper()} stitch map: {source_width}x{source_height} -> "
            f"{output_width}x{output_height}"
        )

    def _rotation_matrix(self, roll: float, pitch: float, yaw: float) -> torch.Tensor:
        cos, sin = math.cos, math.sin
        rx = torch.tensor(((1.0, 0.0, 0.0), (0.0, cos(roll), -sin(roll)),
                           (0.0, sin(roll), cos(roll))), device=self.device)
        ry = torch.tensor(((cos(pitch), 0.0, sin(pitch)), (0.0, 1.0, 0.0),
                           (-sin(pitch), 0.0, cos(pitch))), device=self.device)
        rz = torch.tensor(((cos(yaw), -sin(yaw), 0.0), (sin(yaw), cos(yaw), 0.0),
                           (0.0, 0.0, 1.0)), device=self.device)
        return rz @ ry @ rx

    @staticmethod
    def _rotation(roll: float, pitch: float, yaw: float) -> dict[str, float]:
        return {"roll": math.radians(roll), "pitch": math.radians(pitch), "yaw": math.radians(yaw)}

    def _ensure_equilib_transforms(self, params: dict[str, Any]) -> None:
        """Create cached EquiLib transforms when their configuration changes."""
        key = (
            int(params["out_height"]),
            int(params["out_width"]),
            int(params["perspective_height"]),
            int(params["perspective_width"]),
            float(params["perspective_fov_x"]),
        )
        if key == self.transforms_key:
            return
        self.equi_rotation = Equi2Equi(height=key[0], width=key[1], mode="bilinear")
        self.perspective_projection = Equi2Pers(
            height=key[2], width=key[3], fov_x=key[4], mode="bilinear"
        )
        self.transforms_key = key

    def image_callback(self, message: Image) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            source_height, source_width = bgr.shape[:2]
            if self.maps_dirty or self.input_shape != (source_height, source_width):
                self._rebuild_stitch_grid(source_height, source_width)
            assert self.stitch_grid is not None

            # Stream direct frame into GPU without CPU rotations or slicing
            rgb = np.ascontiguousarray(bgr[..., ::-1])
            source = torch.from_numpy(rgb).to(self.device, non_blocking=True)
            source = source.permute(2, 0, 1).unsqueeze(0).float() / 255.0

            panorama = torch_f.grid_sample(
                source, self.stitch_grid, mode="bilinear", padding_mode="zeros", align_corners=True
            )

            params = self._read_parameters()
            self._ensure_equilib_transforms(params)
            assert self.equi_rotation is not None
            # The node uses a BxCxHxW tensor, so EquiLib requires one rotation
            # dictionary per batch entry.
            panorama = self.equi_rotation(
                src=panorama,
                rots=[self._rotation(
                    params["equi_roll_deg"], params["equi_pitch_deg"], params["equi_yaw_deg"]
                )],
            )
            if params["publish_equirectangular"]:
                self._publish_rgb(panorama, self.equirectangular_publisher, message)

            if params["publish_perspective"]:
                assert self.perspective_projection is not None
                perspective = self.perspective_projection(
                    equi=panorama,
                    rots=[self._rotation(
                        params["perspective_roll_deg"], params["perspective_pitch_deg"],
                        params["perspective_yaw_deg"],
                    )],
                )
                self._publish_rgb(perspective, self.perspective_publisher, message)
        except Exception as error:  # Keep the camera pipeline alive after a malformed frame.
            self.get_logger().error(f"Failed to stitch dual-fisheye frame: {error}")

    def _publish_rgb(self, tensor: torch.Tensor, publisher: Any, source: Image) -> None:
        image = tensor.squeeze(0).permute(1, 2, 0).clamp(0.0, 1.0)
        rgb = (image * 255.0).byte().cpu().numpy()
        output = self.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
        output.header = source.header
        publisher.publish(output)


def main() -> None:
    rclpy.init()
    node = EquiLibStitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

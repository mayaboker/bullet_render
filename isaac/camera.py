"""Isaac 6.2 RTX camera with the existing Bullet FLU pose/RGBA interface."""
import math


def usd_camera_orientation(orientation):
    """Compose world-from-body WXYZ with body-from-USD-camera WXYZ.

    Bullet uses +X forward, +Y left, +Z up. USD cameras use -Z forward,
    +X right, +Y up. Positions and source timestamps are unchanged.
    """
    w, x, y, z = orientation
    return ((w - x + y + z) / 2, (w + x - y + z) / 2,
            (-w + x + y + z) / 2, (-w - x - y + z) / 2)


class Camera:
    def __init__(self, path, *, width, height, fov_x_deg):
        import warp as wp
        from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera
        from pxr import UsdGeom
        import omni.usd

        self.authoring = RtxCamera(path, tick_rate=0.0)
        prim = UsdGeom.Camera(omni.usd.get_context().get_stage().GetPrimAtPath(path))
        aperture = 20.0 * math.tan(math.radians(fov_x_deg) / 2)
        prim.GetFocalLengthAttr().Set(10.0)
        prim.GetHorizontalApertureAttr().Set(aperture)
        prim.GetVerticalApertureAttr().Set(aperture * height / width)
        prim.GetClippingRangeAttr().Set((0.01, 900.0))
        # The 6.2 runtime takes (height, width), unlike the legacy Camera API.
        self.sensor = CameraSensor(self.authoring, resolution=(height, width), annotators=['rgba'])
        self.buffer = wp.empty((height, width, 4), dtype=wp.uint8, device='cpu')

    def set_world_pose(self, position, orientation):
        self.authoring.set_world_poses(positions=[position],
                                       orientations=[usd_camera_orientation(orientation)])

    def get_rgba(self):
        data, _ = self.sensor.get_data('rgba', out=self.buffer)
        return None if data is None else data.numpy()

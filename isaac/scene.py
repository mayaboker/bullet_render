"""In-memory USD world, target and spectator-only camera marker."""
from pathlib import Path
import re

from isaac.scenario import DEFAULT_TARGET_M, TRUCK_PARTS, vector
from isaac.surface import box_triangles, fan_triangles, heights_at

FRONT_CAMERA = '/World/FrontCamera'
PERSPECTIVE_CAMERA = '/OmniverseKit_Persp'
# Kit's "active" viewport is whichever viewport window was clicked last. The panel,
# marker and focus always address the main window so a second viewport cannot steal them.
MAIN_VIEWPORT_WINDOW = 'Viewport'


def main_viewport():
    """The main viewport window's API, falling back to Kit's active viewport."""
    from omni.kit.viewport.utility import get_active_viewport, get_viewport_from_window_name
    return get_viewport_from_window_name(MAIN_VIEWPORT_WINDOW) or get_active_viewport()


class Scene:
    def __init__(self, world=None, target_position=DEFAULT_TARGET_M):
        import omni.usd
        from pxr import UsdGeom, UsdLux
        self.context = omni.usd.get_context()
        self.context.new_stage()
        self.stage = self.context.get_stage()
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)
        UsdGeom.Xform.Define(self.stage, '/World')
        self.stage.SetDefaultPrim(self.stage.GetPrimAtPath('/World'))
        light = UsdLux.DomeLight.Define(self.stage, '/World/BulletLight')
        light.CreateIntensityAttr(1000)
        self.box('/World/Ground', (0, 0, -.1), (500, 500, .2), (.15, .28, .12))
        UsdGeom.Xform.Define(self.stage, '/World/Bullet')
        target = UsdGeom.Xform.Define(self.stage, '/World/Bullet/RedTruck')
        self.target_translate = target.AddTranslateOp()
        for name, centre, size, colour in TRUCK_PARTS:
            self.box('/World/Bullet/RedTruck/' + name, centre, size, colour)
        self.set_target(target_position)
        drone = UsdGeom.Xform.Define(self.stage, '/World/Bullet/Drone')
        self.drone_translate = drone.AddTranslateOp()
        self.drone_orient = drone.AddOrientOp()
        self.world_path = ''
        self.world_warnings = []
        self.disabled_instancers = 0
        self.last_position = (0, 0, .015)
        if world:
            self.load_world(world)

    def box(self, path, position, size, colour):
        from pxr import Gf, UsdGeom
        cube = UsdGeom.Cube.Define(self.stage, path)
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(*colour)])
        cube.AddTranslateOp().Set(Gf.Vec3d(*position))
        cube.AddScaleOp().Set(Gf.Vec3d(*size))

    def set_target(self, position):
        from pxr import Gf
        self.target_position = vector(position)
        self.target_translate.Set(Gf.Vec3d(*position))

    def load_world(self, path):
        """Reference the source default prim without saving or adopting render settings."""
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdLux
        path = Path(path).expanduser().resolve()
        if not path.is_file():
            raise ValueError(f'World file does not exist: {path}')
        source = Usd.Stage.Open(str(path), load=Usd.Stage.LoadNone)
        if source is None or not source.GetDefaultPrim():
            raise ValueError('World needs a valid USD default prim')
        units = UsdGeom.GetStageMetersPerUnit(source)
        axis = UsdGeom.GetStageUpAxis(source)
        if not 0 < units <= 1000 or axis not in ('Y', 'Z'):
            raise ValueError('World must use finite positive metre units and Y or Z up')
        candidate_path = '/World/EnvironmentCandidate'
        self.stage.RemovePrim(candidate_path)
        candidate = UsdGeom.Xform.Define(self.stage, candidate_path)
        candidate.AddScaleOp().Set(Gf.Vec3d(units))
        if axis == 'Y':
            candidate.AddRotateXOp().Set(90)
        content = UsdGeom.Xform.Define(self.stage, candidate_path + '/Content').GetPrim()
        try:
            if not content.GetReferences().AddReference(str(path), source.GetDefaultPrim().GetPath()):
                raise ValueError('USD reference could not be composed')
            errors = {}
            has_light = False
            instancers = []
            # A referenced environment is visual only. Do not simulate its bodies
            # when the camera's render timeline plays. All overrides are in memory.
            for prim in Usd.PrimRange(content):
                for error in prim.GetPrimIndex().localErrors:
                    text = str(error)
                    match = re.search(r'@([^@]+)@', text)
                    errors[match.group(1) if match else text] = text
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
                if prim.IsA(UsdPhysics.Scene):
                    prim.SetActive(False)
                if prim.HasAPI(UsdLux.LightAPI):
                    has_light = True
                if prim.IsA(UsdGeom.PointInstancer):
                    instancers.append(prim.GetPath())
            # BBoxCache can throw on point-instancer prototype paths whose
            # external assets are missing. Inspect composed geometry directly.
            if not any(prim.IsA(UsdGeom.Gprim) for prim in Usd.PrimRange(content, Usd.TraverseInstanceProxies())):
                raise ValueError('World contains no loaded geometry; check its referenced assets')
            # Missing vegetation prototypes can otherwise emit a warning for
            # every instance and break native bounds queries. Hide only batches
            # with no composed geometry at all; do not discard valid prototypes.
            prototype_geometry = {}
            disabled = 0
            for instancer_path in instancers:
                prim = self.stage.GetPrimAtPath(instancer_path)
                targets = UsdGeom.PointInstancer(prim).GetPrototypesRel().GetTargets()
                for target in targets:
                    if target not in prototype_geometry:
                        root = self.stage.GetPrimAtPath(target)
                        prototype_geometry[target] = bool(root) and any(
                            p.IsA(UsdGeom.Gprim) for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies()))
                if not any(prototype_geometry[target] for target in targets):
                    prim.SetActive(False)
                    disabled += 1
            # Keep a stable root, replacing its contents only after composition succeeds.
            edit = Sdf.BatchNamespaceEdit()
            if self.stage.GetPrimAtPath('/World/Environment'):
                edit.Add('/World/Environment', Sdf.Path.emptyPath)
            edit.Add(candidate_path, '/World/Environment')
            layer = self.stage.GetRootLayer()
            if not layer.CanApply(edit) or not layer.Apply(edit):
                raise ValueError('could not activate loaded world')
            self.stage.GetPrimAtPath('/World/Ground').SetActive(False)
            self.stage.GetPrimAtPath('/World/BulletLight').SetActive(not has_light)
            self.world_path = str(path)
            self.world_warnings = sorted(errors)
            self.disabled_instancers = disabled
            if errors:
                print(f'World loaded with {len(errors)} unresolved external assets; see USD warnings in the log', flush=True)
        except Exception:
            self.stage.RemovePrim(candidate_path)
            raise

    def surface_height(self, x, y):
        """Highest visible world surface under Bullet (x, y) metres: (z, prim path) or None.

        The starter slab top is Z=0. A loaded world uses its composed meshes and
        cubes; point-instancer vegetation and invisible prims are ignored. Roofs
        count as surfaces, so the returned prim path tells the caller what was hit.
        """
        from pxr import Gf, Usd, UsdGeom
        root = self.stage.GetPrimAtPath('/World/Environment' if self.world_path else '/World/Ground')
        if not root:
            return None
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render'], useExtentsHint=True)
        best = None
        prims = iter(Usd.PrimRange(root, Usd.TraverseInstanceProxies()))
        for prim in prims:
            if prim.IsA(UsdGeom.PointInstancer):
                prims.PruneChildren()
                continue
            bounds = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if bounds.IsEmpty() or not (bounds.GetMin()[0] <= x <= bounds.GetMax()[0] and bounds.GetMin()[1] <= y <= bounds.GetMax()[1]):
                prims.PruneChildren()
                continue
            if best is not None and bounds.GetMax()[2] <= best[0]:
                prims.PruneChildren()
                continue
            world = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            if prim.IsA(UsdGeom.Mesh):
                mesh = UsdGeom.Mesh(prim)
                points = mesh.GetPointsAttr().Get()
                counts = mesh.GetFaceVertexCountsAttr().Get()
                indices = mesh.GetFaceVertexIndicesAttr().Get()
                if not points or not counts or not indices:
                    continue
                corners = [tuple(world.Transform(Gf.Vec3d(point))) for point in points]
                triangles = ((corners[a], corners[b], corners[c]) for a, b, c in fan_triangles(counts, indices))
            elif prim.IsA(UsdGeom.Cube):
                size = UsdGeom.Cube(prim).GetSizeAttr().Get()
                triangles = [tuple(tuple(world.Transform(Gf.Vec3d(corner))) for corner in triangle)
                             for triangle in box_triangles((0.0, 0.0, 0.0), (size, size, size))]
            else:
                continue
            heights = heights_at(x, y, triangles)
            if heights and (best is None or heights[0] > best[0]):
                best = (heights[0], str(prim.GetPath()))
        return best

    def update_pose(self, pose):
        from pxr import Gf
        p = pose['position_m']
        x, y, z, w = pose['orientation_xyzw']
        self.last_position = tuple(p)
        self.drone_translate.Set(Gf.Vec3d(*p))
        self.drone_orient.Set(Gf.Quatf(w, Gf.Vec3f(x, y, z)))

    def set_view_camera(self, camera_path):
        """Switch the main viewport window to the given camera prim."""
        viewport = main_viewport()
        if viewport is None:
            raise ValueError('No viewport window is open')
        viewport.set_active_camera(camera_path)

    def focus_drone(self):
        import numpy as np
        from isaacsim.core.utils.viewports import set_camera_view
        p = np.asarray(self.last_position)
        self.set_view_camera(PERSPECTIVE_CAMERA)
        set_camera_view(p + np.array([-12., -18., 12.]), p, PERSPECTIVE_CAMERA, viewport_api=main_viewport())


class DroneMarker:
    """Viewport overlay: never enters the sensor render product or detector."""
    def __init__(self):
        import omni.ui as ui
        from omni.ui import scene as sc
        from omni.kit.viewport.utility import get_active_viewport_window
        self.window = get_active_viewport_window(MAIN_VIEWPORT_WINDOW) or get_active_viewport_window()
        self.viewport = self.window.viewport_api
        with self.window.get_frame('bullet_drone_marker'):
            self.view = sc.SceneView()
        self.viewport.add_scene_view(self.view)
        with self.view.scene:
            self.root = sc.Transform()
            with self.root:
                for a, b in (((-1, 0, 0), (1, 0, 0)), ((0, -1, 0), (0, 1, 0)), ((0, 0, -1), (0, 0, 1))):
                    sc.Line(a, b, color=(0, 1, 1, 1), thickness=3)
                self.heading = []
                for a, b in (((0, 0, 0), (3, 0, 0)), ((3, 0, 0), (2.4, .3, 0)), ((3, 0, 0), (2.4, -.3, 0))):
                    line = sc.Line(a, b, color=(1, 1, 0, 1), thickness=4)
                    self.heading.append((line, a, b))
                with sc.Transform(transform=sc.Matrix44.get_translation_matrix(0, 0, 1.6)):
                    sc.Label('DRONE / FRONT CAMERA', color=(0, 1, 1, 1), size=18, alignment=ui.Alignment.CENTER)

    def update(self, pose):
        from omni.ui import scene as sc
        from pxr import Gf
        p = pose['position_m']
        self.root.transform = sc.Matrix44.get_translation_matrix(*p)
        self.root.visible = str(self.viewport.camera_path) != FRONT_CAMERA
        x, y, z, w = pose['orientation_xyzw']
        rotation = Gf.Rotation(Gf.Quatd(w, Gf.Vec3d(x, y, z)))
        for line, a, b in self.heading:
            line.start = tuple(rotation.TransformDir(Gf.Vec3d(*a)))
            line.end = tuple(rotation.TransformDir(Gf.Vec3d(*b)))

    def close(self):
        self.viewport.remove_scene_view(self.view)
        self.view.scene.clear()

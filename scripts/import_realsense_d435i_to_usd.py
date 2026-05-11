#!/usr/bin/env python3
"""Import the local RealSense D435i URDF into a USD file with Isaac Sim."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from pathlib import Path
import xml.etree.ElementTree as ET

from isaacsim import SimulationApp


EMPTY_VISUAL_LINKS = (
    "camera_bottom_screw_frame",
    "camera_accel_frame",
    "camera_accel_optical_frame",
    "camera_color_frame",
    "camera_color_optical_frame",
    "camera_depth_frame",
    "camera_depth_optical_frame",
    "camera_gyro_frame",
    "camera_gyro_optical_frame",
    "camera_infra1_frame",
    "camera_infra1_optical_frame",
    "camera_infra2_frame",
    "camera_infra2_optical_frame",
)


def _set_if_present(obj: object, name: str, value: object) -> None:
    if hasattr(obj, name):
        setattr(obj, name, value)


def _patch_empty_visual_references(physics_layer_path: Path) -> None:
    from pxr import Sdf

    layer = Sdf.Layer.FindOrOpen(str(physics_layer_path))
    if layer is None:
        raise RuntimeError(f"Could not open USD layer: {physics_layer_path}")

    visuals = Sdf.CreatePrimInLayer(layer, "/visuals")
    visuals.specifier = Sdf.SpecifierDef
    visuals.typeName = "Xform"

    for link_name in EMPTY_VISUAL_LINKS:
        prim = Sdf.CreatePrimInLayer(layer, f"/visuals/{link_name}")
        prim.specifier = Sdf.SpecifierDef
        prim.typeName = "Xform"

    layer.Save()


def _render_camera_x_from_dae(dae_path: Path) -> float:
    _, front_z = _extract_dae_front_contour(dae_path)
    return 0.0043 + front_z + 0.0006


def _patch_camera_prims(sensor_layer_path: Path, root_name: str, dae_path: Path) -> None:
    from pxr import Gf, Usd, UsdGeom

    stage = Usd.Stage.Open(str(sensor_layer_path))
    if stage is None:
        raise RuntimeError(f"Could not open USD layer: {sensor_layer_path}")

    def define_camera(
        prim_path: str,
        camera_x: float,
        focal_length: float,
        horizontal_aperture: float,
        vertical_aperture: float,
    ) -> None:
        parent_path = prim_path.rsplit("/", 1)[0]
        UsdGeom.Xform.Define(stage, parent_path)
        camera = UsdGeom.Camera.Define(stage, prim_path)
        camera.CreateProjectionAttr("perspective")
        camera.CreateFocalLengthAttr(focal_length)
        camera.CreateHorizontalApertureAttr(horizontal_aperture)
        camera.CreateVerticalApertureAttr(vertical_aperture)
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))

        # Match Isaac's D455 convention: render cameras live in the regular
        # camera frames, look along body +X, and use body +Z as image up. The
        # ROS optical frames remain separate coordinate frames.
        xformable = UsdGeom.Xformable(camera.GetPrim())
        xformable.ClearXformOpOrder()
        xformable.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(camera_x, 0.0, 0.0))
        xformable.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Quatd(0.5, 0.5, -0.5, -0.5)
        )

    for old_camera_path in (
        f"/{root_name}/camera_color_optical_frame/ColorCamera",
        f"/{root_name}/camera_depth_optical_frame/DepthCamera",
    ):
        if stage.GetPrimAtPath(old_camera_path):
            stage.RemovePrim(old_camera_path)

    # Approximate D435i FOV from published specs: depth 87x58 deg, RGB 69x42 deg.
    render_camera_x = _render_camera_x_from_dae(dae_path)
    define_camera(
        f"/{root_name}/camera_color_frame/ColorCamera",
        camera_x=render_camera_x,
        focal_length=2.834357587853923,
        horizontal_aperture=3.896,
        vertical_aperture=2.1760158808137096,
    )
    define_camera(
        f"/{root_name}/camera_depth_frame/DepthCamera",
        camera_x=render_camera_x,
        focal_length=2.0527636840473145,
        horizontal_aperture=3.896,
        vertical_aperture=2.275730981121917,
    )

    stage.GetRootLayer().Save()


def _create_mdl_material(stage, material_path: str, source_asset: str, sub_identifier: str):
    from pxr import Sdf, UsdShade

    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
    shader.GetPrim().CreateAttribute("info:implementationSource", Sdf.ValueTypeNames.Token).Set("sourceAsset")
    shader.GetPrim().CreateAttribute("info:mdl:sourceAsset", Sdf.ValueTypeNames.Asset).Set(source_asset)
    shader.GetPrim().CreateAttribute("info:mdl:sourceAsset:subIdentifier", Sdf.ValueTypeNames.Token).Set(sub_identifier)
    shader.CreateOutput("out", Sdf.ValueTypeNames.Token)

    material.CreateSurfaceOutput("mdl").ConnectToSource(shader.ConnectableAPI(), "out")
    material.CreateVolumeOutput("mdl").ConnectToSource(shader.ConnectableAPI(), "out")
    material.CreateDisplacementOutput("mdl").ConnectToSource(shader.ConnectableAPI(), "out")
    return material


def _create_omni_pbr_material(stage, material_path: str, color: tuple[float, float, float], roughness: float):
    from pxr import Gf, Sdf, UsdShade

    material = _create_mdl_material(stage, material_path, "OmniPBR.mdl", "OmniPBR")
    shader = UsdShade.Shader.Get(stage, f"{material_path}/Shader")
    shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)
    shader.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(roughness)
    return material


def _bind_material(prim, material) -> None:
    from pxr import UsdShade

    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)


def _polygon_area(points: list[tuple[float, float]]) -> float:
    area = 0.0
    for (x1, y1), (x2, y2) in zip(points, points[1:] + points[:1]):
        area += x1 * y2 - x2 * y1
    return area / 2.0


def _extract_dae_front_contour(dae_path: Path) -> tuple[list[tuple[float, float]], float]:
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    root = ET.parse(dae_path).getroot()
    geometries = root.findall(".//c:library_geometries/c:geometry", ns)
    if not geometries:
        raise RuntimeError(f"No geometry found in {dae_path}")

    # The RealSense D435 DAE stores the body as the largest geometry. Its
    # front-face boundary is the source of truth for the glass outline.
    body = max(
        geometries,
        key=lambda geom: int(geom.find(".//c:source/c:float_array", ns).attrib["count"]),
    )
    float_array = body.find(".//c:source/c:float_array", ns)
    triangles = body.find(".//c:triangles/c:p", ns)
    if float_array is None or triangles is None:
        raise RuntimeError(f"Could not parse body geometry from {dae_path}")

    values = [float(value) for value in float_array.text.split()]
    vertices = [tuple(values[index : index + 3]) for index in range(0, len(values), 3)]
    raw_indices = [int(value) for value in triangles.text.split()]
    faces = [tuple(raw_indices[index : index + 3]) for index in range(0, len(raw_indices), 3)]
    front_z = max(vertex[2] for vertex in vertices)

    def is_front_vertex(index: int) -> bool:
        return abs(vertices[index][2] - front_z) <= 1e-6

    def key(index: int) -> tuple[float, float]:
        x, y, _ = vertices[index]
        return (round(x, 7), round(y, 7))

    edge_counts: Counter[tuple[tuple[float, float], tuple[float, float]]] = Counter()
    for face in faces:
        if not all(is_front_vertex(index) for index in face):
            continue
        for start, end in ((face[0], face[1]), (face[1], face[2]), (face[2], face[0])):
            a = key(start)
            b = key(end)
            if a == b:
                continue
            edge_counts[tuple(sorted((a, b)))] += 1

    boundary_edges = [edge for edge, count in edge_counts.items() if count == 1]
    adjacency: dict[tuple[float, float], list[tuple[float, float]]] = defaultdict(list)
    for start, end in boundary_edges:
        adjacency[start].append(end)
        adjacency[end].append(start)

    loops: list[list[tuple[float, float]]] = []
    visited: set[tuple[float, float]] = set()
    for start in adjacency:
        if start in visited:
            continue
        loop: list[tuple[float, float]] = []
        previous = None
        current = start
        while True:
            loop.append(current)
            visited.add(current)
            candidates = [node for node in adjacency[current] if node != previous]
            if not candidates:
                break
            next_node = candidates[0]
            if next_node == start:
                break
            previous, current = current, next_node
        if len(loop) >= 3:
            loops.append(loop)

    if not loops:
        raise RuntimeError(f"Could not extract front contour from {dae_path}")

    return max(loops, key=lambda loop: abs(_polygon_area(loop))), front_z


def _define_dae_front_glass_mesh(stage, mesh_path: str, dae_path: Path):
    from pxr import Gf, UsdGeom

    outline, front_z = _extract_dae_front_contour(dae_path)
    thickness = 0.0006
    camera_x = 0.0043 + front_z + 0.00045
    x_front = camera_x + thickness / 2.0
    x_back = camera_x - thickness / 2.0
    points = [Gf.Vec3f(x_front, raw_x - 0.0175, raw_y) for raw_x, raw_y in outline]
    points += [Gf.Vec3f(x_back, raw_x - 0.0175, raw_y) for raw_x, raw_y in outline]

    count = len(outline)
    face_vertex_counts = [count, count]
    face_vertex_indices = list(range(count)) + list(range(2 * count - 1, count - 1, -1))
    for idx in range(count):
        next_idx = (idx + 1) % count
        face_vertex_counts.append(4)
        face_vertex_indices.extend([idx, next_idx, next_idx + count, idx + count])

    mesh = UsdGeom.Mesh.Define(stage, mesh_path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
    mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)
    mesh.CreateDoubleSidedAttr(True)
    return mesh


def _patch_glass_cover(base_layer_path: Path, root_name: str, dae_path: Path) -> None:
    from pxr import Gf, Usd, UsdGeom

    stage = Usd.Stage.Open(str(base_layer_path))
    if stage is None:
        raise RuntimeError(f"Could not open USD layer: {base_layer_path}")

    glass = _create_mdl_material(
        stage,
        f"/{root_name}/Looks/OmniGlass",
        source_asset="OmniGlass.mdl",
        sub_identifier="OmniGlass",
    )
    front_mask = _create_omni_pbr_material(
        stage,
        f"/{root_name}/Looks/MatteBlackFrontMask",
        color=(0.0, 0.018800832, 0.033755302),
        roughness=0.72,
    )

    assembly = UsdGeom.Xform.Define(stage, f"/{root_name}/camera_link/D435iFrontGlassAssembly")

    cover = _define_dae_front_glass_mesh(
        stage,
        f"{assembly.GetPath()}/GlassCover",
        dae_path=dae_path,
    )
    _bind_material(cover.GetPrim(), glass)

    window_specs = (
        ("ColorWindow", 0.015, 0.0046),
        ("LeftIRWindow", 0.0, 0.0042),
        ("ProjectorWindow", -0.025, 0.0057),
        ("RightIRWindow", -0.050, 0.0042),
    )
    for name, y_offset, radius in window_specs:
        window = UsdGeom.Cylinder.Define(stage, f"{assembly.GetPath()}/{name}")
        window.CreateAxisAttr("X")
        window.CreateHeightAttr(0.0012)
        window.CreateRadiusAttr(radius)
        window.AddTranslateOp().Set(Gf.Vec3d(0.0049, y_offset, 0.0))
        _bind_material(window.GetPrim(), front_mask)

    stage.GetRootLayer().Save()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--urdf",
        default="src/asset/urdf/realsense_d435i/realsense_d435i.urdf",
        help="Input D435i URDF path, relative to the workspace by default.",
    )
    parser.add_argument(
        "--out",
        default="src/asset/usd/realsense/d435i/realsense_d435i.usd",
        help="Output USD path, relative to the workspace by default.",
    )
    args = parser.parse_args()

    workspace = Path(__file__).resolve().parents[1]
    urdf_path = Path(args.urdf)
    out_path = Path(args.out)
    if not urdf_path.is_absolute():
        urdf_path = workspace / urdf_path
    if not out_path.is_absolute():
        out_path = workspace / out_path

    out_path.parent.mkdir(parents=True, exist_ok=True)

    app = SimulationApp(
        {
            "headless": True,
            "disable_viewport_updates": True,
            "multi_gpu": False,
            "width": 64,
            "height": 64,
        }
    )

    import omni.kit.commands
    import omni.usd
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.asset.importer.urdf")
    app.update()

    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not status:
        raise RuntimeError("Failed to create Isaac Sim URDF import config")

    # Keep fixed camera frames so downstream ROS/Isaac code can address each
    # optical frame explicitly.
    _set_if_present(import_config, "merge_fixed_joints", False)
    _set_if_present(import_config, "convex_decomp", False)
    _set_if_present(import_config, "import_inertia_tensor", True)
    _set_if_present(import_config, "fix_base", True)
    _set_if_present(import_config, "distance_scale", 1.0)
    _set_if_present(import_config, "collision_from_visuals", False)
    _set_if_present(import_config, "make_default_prim", True)

    status, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(urdf_path),
        import_config=import_config,
        dest_path=str(out_path),
        get_articulation_root=False,
    )
    if not status:
        raise RuntimeError(f"URDF import failed for {urdf_path}")

    base_layer_path = out_path.parent / "configuration" / f"{out_path.stem}_base.usd"
    if base_layer_path.exists():
        _patch_empty_visual_references(base_layer_path)
    physics_layer_path = out_path.parent / "configuration" / f"{out_path.stem}_physics.usd"
    if physics_layer_path.exists():
        _patch_empty_visual_references(physics_layer_path)

    if base_layer_path.exists():
        _patch_glass_cover(
            base_layer_path,
            root_name=out_path.stem,
            dae_path=urdf_path.parent / "meshes" / "d435.dae",
        )
    sensor_layer_path = out_path.parent / "configuration" / f"{out_path.stem}_sensor.usd"
    if sensor_layer_path.exists():
        _patch_camera_prims(
            sensor_layer_path,
            root_name=out_path.stem,
            dae_path=urdf_path.parent / "meshes" / "d435.dae",
        )

    print(f"Imported {urdf_path}", flush=True)
    print(f"USD {out_path}", flush=True)
    print(f"Root prim {prim_path}", flush=True)

    app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

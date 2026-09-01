#!/usr/bin/env python3
"""Convert a SceneSmith MuJoCo export into a browser-friendly static package."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path


def file_digest(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def optimize_obj(source: Path, destination: Path) -> dict[str, int]:
    """Remove duplicate OBJ attributes while preserving every face and UV exactly."""
    lines = source.read_text(encoding="utf-8", errors="ignore").splitlines(keepends=True)
    kinds = ("v", "vt", "vn")
    mappings: dict[str, list[int]] = {kind: [0] for kind in kinds}
    representatives: dict[str, set[int]] = {kind: set() for kind in kinds}
    values: dict[str, dict[str, int]] = {kind: {} for kind in kinds}

    for line in lines:
        parts = line.split()
        if not parts or parts[0] not in mappings:
            continue
        kind = parts[0]
        value = " ".join(parts[1:])
        mapped = values[kind].get(value)
        if mapped is None:
            mapped = len(values[kind]) + 1
            values[kind][value] = mapped
            representatives[kind].add(len(mappings[kind]))
        mappings[kind].append(mapped)

    original_indices = {kind: 0 for kind in kinds}
    output_lines = []
    for line in lines:
        parts = line.split()
        if parts and parts[0] in mappings:
            kind = parts[0]
            original_indices[kind] += 1
            if original_indices[kind] in representatives[kind]:
                output_lines.append(line)
            continue
        if parts and parts[0] == "f":
            remapped = []
            for vertex in parts[1:]:
                indices = vertex.split("/")
                for offset, kind in enumerate(kinds):
                    if offset >= len(indices) or not indices[offset]:
                        continue
                    index = int(indices[offset])
                    if index <= 0:
                        raise ValueError(f"Negative OBJ index is not supported: {source}")
                    indices[offset] = str(mappings[kind][index])
                remapped.append("/".join(indices))
            output_lines.append("f " + " ".join(remapped) + "\n")
        else:
            output_lines.append(line)

    destination.write_text("".join(output_lines), encoding="utf-8")
    return {
        "verticesBefore": len(mappings["v"]) - 1,
        "verticesAfter": len(values["v"]),
        "texcoordsBefore": len(mappings["vt"]) - 1,
        "texcoordsAfter": len(values["vt"]),
    }


def deduplicate_assets(root: ET.Element, source_meshes: Path) -> dict[str, int]:
    """Merge byte-identical assets and equivalent MJCF definitions."""
    asset = root.find("asset")
    if asset is None:
        return {"meshes": 0, "textures": 0, "materials": 0}

    removed = {"meshes": 0, "textures": 0, "materials": 0}

    def merge(tag: str, reference_attribute: str, report_key: str, hash_file: bool) -> None:
        canonical_by_signature: dict[tuple, ET.Element] = {}
        for element in list(asset.findall(tag)):
            name = element.get("name")
            file_name = element.get("file")
            attributes = dict(element.attrib)
            attributes.pop("name", None)
            if hash_file and file_name:
                attributes["file"] = file_digest(source_meshes / file_name)
            signature = tuple(sorted(attributes.items()))
            canonical = canonical_by_signature.get(signature)
            if canonical is None:
                canonical_by_signature[signature] = element
                continue
            canonical_name = canonical.get("name")
            for consumer in root.iter():
                if consumer.get(reference_attribute) == name:
                    consumer.set(reference_attribute, canonical_name)
            asset.remove(element)
            removed[report_key] += 1

    merge("mesh", "mesh", "meshes", True)
    merge("texture", "texture", "textures", True)
    merge("material", "material", "materials", False)
    return removed


def vector(text: str | None, length: int, default: tuple[float, ...]) -> list[float]:
    values = [float(value) for value in text.split()] if text else list(default)
    if len(values) != length:
        raise ValueError(f"Expected {length} values, got {values}")
    return values


def quaternion_multiply(left: list[float], right: list[float]) -> list[float]:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return [
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ]


def quaternion_rotate(quaternion: list[float], point: list[float]) -> list[float]:
    w, x, y, z = quaternion
    px, py, pz = point
    tx = 2 * (y * pz - z * py)
    ty = 2 * (z * px - x * pz)
    tz = 2 * (x * py - y * px)
    return [
        px + w * tx + (y * tz - z * ty),
        py + w * ty + (z * tx - x * tz),
        pz + w * tz + (x * ty - y * tx),
    ]


def compose(
    parent_position: list[float],
    parent_quaternion: list[float],
    local_position: list[float],
    local_quaternion: list[float],
) -> tuple[list[float], list[float]]:
    offset = quaternion_rotate(parent_quaternion, local_position)
    return (
        [parent_position[index] + offset[index] for index in range(3)],
        quaternion_multiply(parent_quaternion, local_quaternion),
    )


def obj_bounds(path: Path, scale: list[float]) -> tuple[list[float], list[float]]:
    minimum = [math.inf, math.inf, math.inf]
    maximum = [-math.inf, -math.inf, -math.inf]
    with path.open("r", encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            if not line.startswith("v "):
                continue
            values = [float(value) for value in line.split()[1:4]]
            values = [values[index] * scale[index] for index in range(3)]
            for index, value in enumerate(values):
                minimum[index] = min(minimum[index], value)
                maximum[index] = max(maximum[index], value)
    if not all(math.isfinite(value) for value in minimum + maximum):
        raise ValueError(f"OBJ has no vertices: {path}")
    return minimum, maximum


def bounds_corners(minimum: list[float], maximum: list[float]) -> list[list[float]]:
    return [
        [x, y, z]
        for x in (minimum[0], maximum[0])
        for y in (minimum[1], maximum[1])
        for z in (minimum[2], maximum[2])
    ]


def fmt(values: list[float]) -> str:
    return " ".join(f"{value:.7g}" for value in values)


def relative_visual_bounds(
    top_body: ET.Element,
    mesh_definitions: dict[str, ET.Element],
    mesh_directory: Path,
    bounds_cache: dict[tuple[str, tuple[float, ...]], tuple[list[float], list[float]]],
) -> tuple[list[float], list[float]]:
    minimum = [math.inf, math.inf, math.inf]
    maximum = [-math.inf, -math.inf, -math.inf]

    def visit(body: ET.Element, body_position: list[float], body_quaternion: list[float]) -> None:
        for geom in body.findall("geom"):
            mesh_name = geom.get("mesh")
            if not mesh_name or "visual" not in (geom.get("name") or ""):
                continue
            mesh = mesh_definitions[mesh_name]
            file_name = mesh.get("file")
            scale = vector(mesh.get("scale"), 3, (1, 1, 1))
            key = (file_name, tuple(scale))
            if key not in bounds_cache:
                bounds_cache[key] = obj_bounds(mesh_directory / file_name, scale)
            mesh_minimum, mesh_maximum = bounds_cache[key]
            geom_position = vector(geom.get("pos"), 3, (0, 0, 0))
            geom_quaternion = vector(geom.get("quat"), 4, (1, 0, 0, 0))
            position, quaternion = compose(body_position, body_quaternion, geom_position, geom_quaternion)
            for corner in bounds_corners(mesh_minimum, mesh_maximum):
                rotated = quaternion_rotate(quaternion, corner)
                point = [position[index] + rotated[index] for index in range(3)]
                for index, value in enumerate(point):
                    minimum[index] = min(minimum[index], value)
                    maximum[index] = max(maximum[index], value)
        for child in body.findall("body"):
            position, quaternion = compose(
                body_position,
                body_quaternion,
                vector(child.get("pos"), 3, (0, 0, 0)),
                vector(child.get("quat"), 4, (1, 0, 0, 0)),
            )
            visit(child, position, quaternion)

    visit(top_body, [0, 0, 0], [1, 0, 0, 0])
    if not all(math.isfinite(value) for value in minimum + maximum):
        raise ValueError(f"No visual mesh found for body {top_body.get('name')}")
    return minimum, maximum


def remove_descendants(root: ET.Element, tags: set[str]) -> int:
    removed = 0
    for parent in root.iter():
        for child in list(parent):
            if child.tag in tags:
                parent.remove(child)
                removed += 1
    return removed


def world_aabb(
    local_minimum: list[float],
    local_maximum: list[float],
    position: list[float],
    quaternion: list[float],
) -> tuple[list[float], list[float]]:
    points = []
    for corner in bounds_corners(local_minimum, local_maximum):
        rotated = quaternion_rotate(quaternion, corner)
        points.append([position[index] + rotated[index] for index in range(3)])
    return (
        [min(point[index] for point in points) for index in range(3)],
        [max(point[index] for point in points) for index in range(3)],
    )


def find_spawn(
    floor_minimum: list[float],
    floor_maximum: list[float],
    obstacles: list[tuple[list[float], list[float]]],
    robot_radius: float,
) -> tuple[list[float], float]:
    step = 0.10
    margin = robot_radius + 0.08
    best_position = None
    best_clearance = -math.inf
    x = floor_minimum[0] + margin
    while x <= floor_maximum[0] - margin:
        y = floor_minimum[1] + margin
        while y <= floor_maximum[1] - margin:
            wall_clearance = min(
                x - floor_minimum[0], floor_maximum[0] - x,
                y - floor_minimum[1], floor_maximum[1] - y,
            )
            clearance = wall_clearance
            for minimum, maximum in obstacles:
                dx = max(minimum[0] - x, 0, x - maximum[0])
                dy = max(minimum[1] - y, 0, y - maximum[1])
                if dx == 0 and dy == 0:
                    clearance = -1
                    break
                clearance = min(clearance, math.hypot(dx, dy))
            if clearance > best_clearance:
                best_clearance = clearance
                best_position = [round(x, 3), round(y, 3)]
            y += step
        x += step
    if best_position is None or best_clearance < robot_radius:
        raise RuntimeError(f"No collision-free spawn found (best clearance={best_clearance:.3f})")
    return best_position, best_clearance


def build_package(
    source: Path,
    output: Path,
    robot_radius: float,
    scene_id: str,
    source_archive: str,
) -> dict:
    source_xml = source / "scene.xml"
    source_meshes = source / "meshes"
    root = ET.parse(source_xml).getroot()
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", "meshes")
    compiler.set("texturedir", "meshes/scenesmith")

    asset = root.find("asset")
    worldbody = root.find("worldbody")
    if asset is None or worldbody is None:
        raise ValueError("SceneSmith export is missing asset or worldbody")
    mesh_definitions = {mesh.get("name"): mesh for mesh in asset.findall("mesh")}
    bounds_cache = {}
    static_boxes = []
    original_collision_geoms = 0

    ground = worldbody.find("geom[@name='ground_plane']")
    if ground is not None:
        worldbody.remove(ground)

    floor_minimum = None
    floor_maximum = None
    room_body = next(body for body in worldbody.findall("body") if body.get("name", "").startswith("room_geometry_"))
    room_type = room_body.get("name", "room_geometry_unknown").removeprefix("room_geometry_").split("_room_geometry")[0]
    room_position = vector(room_body.get("pos"), 3, (0, 0, 0))
    for geom in room_body.iter("geom"):
        name = geom.get("name", "")
        if "collision" in name:
            geom.set("group", "3")
            geom.set("contype", "1")
            geom.set("conaffinity", "1")
            geom.set("rgba", "0.2 0.8 0.3 0.001")
        if "floor_collision" in name and geom.get("type") == "box":
            center = vector(geom.get("pos"), 3, (0, 0, 0))
            size = vector(geom.get("size"), 3, (0, 0, 0))
            center = [center[index] + room_position[index] for index in range(3)]
            floor_minimum = [center[index] - size[index] for index in range(3)]
            floor_maximum = [center[index] + size[index] for index in range(3)]

    for top_body in list(worldbody.findall("body")):
        if top_body is room_body:
            continue
        name = top_body.get("name", "object")
        local_minimum, local_maximum = relative_visual_bounds(
            top_body, mesh_definitions, source_meshes, bounds_cache
        )
        original_collision_geoms += sum(
            1 for geom in top_body.iter("geom") if "collision" in geom.get("name", "")
        )
        remove_descendants(top_body, {"joint", "freejoint", "inertial"})
        for parent in top_body.iter():
            for geom in list(parent.findall("geom")):
                if "collision" in geom.get("name", ""):
                    parent.remove(geom)
        center = [(local_minimum[index] + local_maximum[index]) * 0.5 for index in range(3)]
        size = [max((local_maximum[index] - local_minimum[index]) * 0.5, 0.005) for index in range(3)]
        top_body.append(ET.Element("geom", {
            "name": f"{name}_static_collision",
            "type": "box",
            "pos": fmt(center),
            "size": fmt(size),
            "group": "3",
            "contype": "1",
            "conaffinity": "1",
            "rgba": "0.2 0.8 0.3 0.001",
            "friction": "0.8 0.02 0.002",
        }))
        position = vector(top_body.get("pos"), 3, (0, 0, 0))
        quaternion = vector(top_body.get("quat"), 4, (1, 0, 0, 0))
        static_boxes.append(world_aabb(local_minimum, local_maximum, position, quaternion))

    for geom in worldbody.iter("geom"):
        if "visual" in geom.get("name", ""):
            geom.set("group", "1")
            geom.set("contype", "0")
            geom.set("conaffinity", "0")

    original_mesh_count = len(asset.findall("mesh"))
    original_texture_count = len([item for item in asset.findall("texture") if item.get("file")])
    deduplicated = deduplicate_assets(root, source_meshes)

    referenced_mesh_names = {geom.get("mesh") for geom in worldbody.iter("geom") if geom.get("mesh")}
    for mesh in list(asset.findall("mesh")):
        if mesh.get("name") not in referenced_mesh_names:
            asset.remove(mesh)
        else:
            mesh.set("file", f"scenesmith/{mesh.get('file')}")

    if floor_minimum is None or floor_maximum is None:
        raise RuntimeError("Could not locate the SceneSmith floor collision")
    spawn_position, spawn_clearance = find_spawn(
        floor_minimum, floor_maximum, static_boxes, robot_radius
    )

    used_files = {Path(mesh.get("file")).name for mesh in asset.findall("mesh")}
    used_files.update(Path(texture.get("file")).name for texture in asset.findall("texture") if texture.get("file"))
    output_meshes = output / "meshes" / "scenesmith"
    if output_meshes.exists():
        shutil.rmtree(output_meshes)
    output_meshes.mkdir(parents=True, exist_ok=True)
    obj_optimization = {
        "verticesBefore": 0,
        "verticesAfter": 0,
        "texcoordsBefore": 0,
        "texcoordsAfter": 0,
    }
    for file_name in sorted(used_files):
        source_file = source_meshes / file_name
        destination_file = output_meshes / file_name
        if source_file.suffix.lower() == ".obj":
            result = optimize_obj(source_file, destination_file)
            for key, value in result.items():
                obj_optimization[key] += value
        else:
            shutil.copy2(source_file, destination_file)

    output.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(output / "scene.xml", encoding="unicode")
    package_files = [f"meshes/scenesmith/{name}" for name in sorted(used_files)]
    (output / "index.json").write_text(json.dumps(package_files, indent=2) + "\n", encoding="utf-8")
    spawn = {
        "schemaVersion": 1,
        "position": spawn_position,
        "yaw": 0,
        "clearanceMeters": round(spawn_clearance, 4),
        "robotRadiusMeters": robot_radius,
        "method": "static-aabb-distance-field",
        "sceneId": scene_id,
    }
    (output / "spawn.json").write_text(json.dumps(spawn, indent=2) + "\n", encoding="utf-8")
    notice = f"""# SceneSmith {source_archive.removesuffix('.tar')}

This runtime package is derived from `Room/{source_archive}` in
`nepfaff/scenesmith-example-scenes`.

- Source: https://huggingface.co/datasets/nepfaff/scenesmith-example-scenes
- SceneSmith project: https://github.com/nepfaff/scenesmith
- License: Apache-2.0 (as declared for the generated Room subset)

`scripts/prepare-scenesmith.py` removes furniture free joints, replaces the
generated convex decomposition with one static collision box per object, keeps
the textured visual meshes, and computes a collision-free robot spawn.
"""
    (output / "NOTICE.md").write_text(notice, encoding="utf-8")
    report = {
        "source": f"nepfaff/scenesmith-example-scenes Room/{source_archive}",
        "sourceLicense": "Apache-2.0",
        "roomType": room_type,
        "furnitureMode": "static",
        "visualMeshCount": len(asset.findall("mesh")),
        "textureCount": len([item for item in asset.findall("texture") if item.get("file")]),
        "assetDeduplication": {
            "meshDefinitionsBefore": original_mesh_count,
            "textureDefinitionsBefore": original_texture_count,
            "removed": deduplicated,
        },
        "objOptimization": obj_optimization,
        "originalFurnitureCollisionGeomCount": original_collision_geoms,
        "staticFurnitureCollisionBoxCount": len(static_boxes),
        "runtimeAssetCount": len(used_files),
        "spawn": spawn,
    }
    (output / "generation_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True, help="Extracted SceneSmith mujoco directory")
    parser.add_argument("--output", type=Path, required=True, help="Runtime environment package directory")
    parser.add_argument("--scene-id", required=True, help="Environment ID written to spawn metadata")
    parser.add_argument("--source-archive", required=True, help="Dataset archive name, e.g. scene_036.tar")
    parser.add_argument("--robot-radius", type=float, default=0.45)
    args = parser.parse_args()
    report = build_package(
        args.source.resolve(),
        args.output.resolve(),
        args.robot_radius,
        args.scene_id,
        args.source_archive,
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()

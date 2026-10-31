"""Convert a Fusion 360 assembly export (USDZ) into Balancio visual meshes (OBJ).

Runs inside Blender (headless):

    pixi run cad-convert path/to/assembly.usdz
    pixi run cad-convert path/to/assembly.usdz --blend balancio.blend   # also save a .blend
    pixi run cad-convert --from-blend balancio.blend                    # re-export an edited .blend

Fusion exports the whole assembly in millimetres, Z up, with part names lost.
This script:

  * imports the USDZ with its materials (converted to metres),
  * bakes every object's transform and splits the parts into
    left wheel / right wheel / body: meshes entirely beyond +/- wheel_y in Y
    and at least min_wheel_part across (so motor shafts stay on the body)
    belong to a wheel,
  * puts the wheel axle (midpoint of both wheel centres) at the origin of the
    body, and each wheel at its own centre,
  * exports meshes/{body,wheel_left,wheel_right}.obj (+ .mtl colours),
  * optionally saves a .blend (open it to tweak materials or add textures, then
    re-export with --from-blend),
  * prints the measured wheel radius / separation for config/balancio.yaml.

By default Fusion's -X (the electronics side) becomes the robot's forward
direction (ROS +X); use --forward +x for the engraved "Balancio 2.0" side.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import bpy
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
DEFAULT_OUT = os.path.join(ROOT, "src", "balancio_description", "meshes")


def parse_args() -> argparse.Namespace:
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("usdz", nargs="?", help="Fusion 360 assembly export (.usdz)")
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--blend", help="Also save the processed scene to this .blend file")
    ap.add_argument("--wheel-y", type=float, default=0.030)
    ap.add_argument(
        "--min-wheel-part",
        type=float,
        default=0.015,
        help="Smaller parts (e.g. motor shafts) stay on the body [m]",
    )
    ap.add_argument("--max-metallic", type=float, default=0.0)
    ap.add_argument(
        "--forward",
        choices=["+x", "-x"],
        default="-x",
        help="Fusion axis that is the robot's front (-x: electronics, +x: engraved side)",
    )
    ap.add_argument(
        "--from-blend",
        metavar="BLEND",
        help="Export from a .blend saved earlier with --blend (keeps your edits)",
    )
    args = ap.parse_args(argv)
    if not args.usdz and not args.from_blend:
        ap.error("pass the Fusion 360 export (.usdz) or --from-blend BLEND")
    return args


def world_bounds(objs) -> tuple[Vector, Vector]:
    lo = Vector((float("inf"),) * 3)
    hi = Vector((float("-inf"),) * 3)
    for o in objs:
        for v in o.data.vertices:
            p = o.matrix_world @ v.co
            lo = Vector(map(min, lo, p))
            hi = Vector(map(max, hi, p))
    return lo, hi


def bake_transforms() -> list:
    """Give every mesh its own data with the world transform applied."""
    meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
    for o in meshes:
        mw = o.matrix_world.copy()
        o.data = o.data.copy()  # USD instances share mesh data
        o.data.transform(mw)
        if mw.determinant() < 0:
            o.data.flip_normals()
        o.parent = None
        o.matrix_world = Matrix.Identity(4)
    for o in [o for o in bpy.context.scene.objects if o.type != "MESH"]:
        bpy.data.objects.remove(o, do_unlink=True)
    return meshes


def join(objs, name: str):
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    if len(objs) > 1:
        bpy.ops.object.join()
    obj = bpy.context.view_layer.objects.active
    obj.name = obj.data.name = name
    return obj


def shift(obj, offset: Vector) -> None:
    obj.data.transform(Matrix.Translation(-offset))


def tame_materials(max_metallic: float) -> None:
    # Fully metallic surfaces render nearly black without an environment map
    # (Gazebo / RViz), so cap metallic and keep some roughness.
    for mat in bpy.data.materials:
        if not mat.use_nodes:
            continue
        for node in mat.node_tree.nodes:
            if node.type == "BSDF_PRINCIPLED":
                m = node.inputs["Metallic"]
                r = node.inputs["Roughness"]
                if not m.is_linked:
                    m.default_value = min(m.default_value, max_metallic)
                if not r.is_linked:
                    r.default_value = max(r.default_value, 0.3)


def process_usdz(args) -> dict:
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.wm.usd_import(filepath=os.path.abspath(args.usdz), import_materials=True)
    meshes = bake_transforms()
    if args.forward == "-x":
        # Turn the robot around so its front is ROS +X (this also swaps the
        # left/right sides, hence before splitting the wheels).
        for o in meshes:
            o.data.transform(Matrix.Rotation(math.pi, 4, "Z"))

    left, right, body = [], [], []
    for o in meshes:
        lo, hi = world_bounds([o])
        big = min((hi - lo).x, (hi - lo).z) >= args.min_wheel_part
        if big and lo.y > args.wheel_y:
            left.append(o)
        elif big and hi.y < -args.wheel_y:
            right.append(o)
        else:
            body.append(o)
    if not left or not right:
        raise SystemExit("Could not find both wheels; adjust --wheel-y.")

    # Wheel centre = centre of its largest part (the tyre): that is the
    # rolling plane, hubs may sit off to one side.
    def tyre_centre(objs) -> Vector:
        def size(o):
            lo, hi = world_bounds([o])
            return (hi - lo).x * (hi - lo).z

        lo, hi = world_bounds([max(objs, key=size)])
        return (lo + hi) / 2

    lc, rc = tyre_centre(left), tyre_centre(right)
    axle = (lc + rc) / 2

    parts = {
        "body": join(body, "body"),
        "wheel_left": join(left, "wheel_left"),
        "wheel_right": join(right, "wheel_right"),
    }
    parts["body"]["wheel_separation"] = lc.y - rc.y
    shift(parts["body"], axle)
    shift(parts["wheel_left"], lc)
    shift(parts["wheel_right"], rc)
    tame_materials(args.max_metallic)

    if args.blend:
        bpy.ops.wm.save_as_mainfile(filepath=os.path.abspath(args.blend), compress=True)
        print(f"saved {args.blend}")
    return parts


def report(parts: dict) -> None:
    llo, lhi = world_bounds([parts["wheel_left"]])
    rlo, rhi = world_bounds([parts["wheel_right"]])
    blo, bhi = world_bounds([parts["body"]])
    radius = ((lhi - llo).x + (lhi - llo).z + (rhi - rlo).x + (rhi - rlo).z) / 8
    # Wheel centres are at the origin of their own meshes; recover the
    # separation from the body frame stored in the blend's custom property.
    sep = parts["body"].get("wheel_separation")
    print("\nMeasured values for config/balancio.yaml:")
    print(f"  r: {radius:.4f}")
    if sep is not None:
        print(f"  d: {sep:.4f}")
    print(f"  body bounds (axle frame): min {tuple(round(v, 4) for v in blo)}")
    print(f"                            max {tuple(round(v, 4) for v in bhi)}")


def fix_mtl(path: str) -> None:
    """Blender writes Ka 1 1 1 (white ambient), which washes out every colour in
    Gazebo; use the diffuse colour as ambient instead."""
    if not os.path.exists(path):
        return
    with open(path) as f:
        text = f.read()
    blocks, cur = [], []
    for line in text.splitlines():
        if line.startswith("newmtl") and cur:
            blocks.append(cur)
            cur = []
        cur.append(line)
    blocks.append(cur)
    lines = []
    for block in blocks:
        kd = next((ln for ln in block if ln.startswith("Kd ")), None)
        for ln in block:
            lines.append("Ka " + kd[3:] if ln.startswith("Ka ") and kd else ln)
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def export(parts: dict, out: str) -> None:
    """OBJ + MTL, written in the ROS frame (Z up, X forward), no axis conversion.

    glTF is avoided on purpose: Gazebo converts glTF from Y-up to Z-up but RViz
    does not, so no single glTF file looks right in both. Both load OBJ as-is.
    """
    os.makedirs(out, exist_ok=True)
    for old in os.listdir(out):
        if old.endswith((".glb", ".gltf", ".obj", ".mtl")):
            os.remove(os.path.join(out, old))
    for name, obj in parts.items():
        bpy.ops.object.select_all(action="DESELECT")
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        path = os.path.join(out, f"{name}.obj")
        bpy.ops.wm.obj_export(
            filepath=path,
            export_selected_objects=True,
            forward_axis="Y",  # identity mapping: keep Blender/ROS axes
            up_axis="Z",
            apply_modifiers=True,
            export_normals=True,
            export_uv=False,
            export_materials=True,
            export_pbr_extensions=False,
            path_mode="STRIP",
        )
        fix_mtl(os.path.join(out, f"{name}.mtl"))
        size = sum(
            os.path.getsize(os.path.join(out, f"{name}.{ext}"))
            for ext in ("obj", "mtl")
            if os.path.exists(os.path.join(out, f"{name}.{ext}"))
        )
        print(f"wrote {path} (+ .mtl, {size / 1e6:.2f} MB)")


def main() -> None:
    args = parse_args()
    if args.from_blend:
        bpy.ops.wm.open_mainfile(filepath=os.path.abspath(args.from_blend))
        parts = {n: bpy.data.objects[n] for n in ("body", "wheel_left", "wheel_right")}
    else:
        parts = process_usdz(args)
    export(parts, args.out)
    report(parts)


if __name__ == "__main__":
    main()

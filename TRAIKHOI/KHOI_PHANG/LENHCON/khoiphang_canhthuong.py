# -*- coding: utf-8 -*-
"""KHOIPHANG_THUONG - adapted compact-layout unroll workflow.

This script is based on the FMultiUnroll compact-layout approach but kept
project-specific and lightweight for the current Rhino workflow.

What it does:
- Selects Surface / Polysurface / Extrusion objects.
- Duplicates their geometry without modifying the original objects.
- Validates each source input.
- Attempts to unroll the source with Rhino's Unroller.
- Rotates each flat result to improve packing.
- Places the final results into a compact layout with a 10 mm spacing.
- Adds the resulting flat Breps to the Rhino document.

This version is intentionally compact and safe for the current project, and it
is designed to be easy to review and extend later.
"""

import math
import os
import sys

import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc


COMMAND_NAME = "KHOIPHANG_THUONG"
VERSION = "1.0.0"
LAYOUT_SPACING_MM = 10.0
RUN_SC_AFTER_UNROLL = True
RUN_CHAN_AFTER_SC = True


def _target_path(filename):
    return os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "target", filename)


def _load_target(module_name, filename):
    """Load a target script, including under Rhino 7's IronPython 2.7."""
    path = _target_path(filename)
    if not os.path.isfile(path):
        raise IOError("Khong tim thay target: {0}".format(path))

    try:
        import importlib.util
        spec = importlib.util.spec_from_file_location(module_name, path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)
        return module
    except (ImportError, AttributeError):
        import imp
        return imp.load_source(module_name, path)


def _run_sc(object_ids):
    _write("Chuyen ket qua sang SC.py.")
    module = _load_target("khoiphang_target_sc", "SC.py")
    return module.integrated_smart_process(object_ids)


def _run_chan():
    _write("Mo CHAN v2.1.1; hay chon dung mot nhom contour va DUONG_CHAN/SOI.")
    module = _load_target("khoiphang_target_chan_v2_1_1",
                          "CHAN_v2.1.1.py")
    return module.run_safe()


def _write(message):
    Rhino.RhinoApp.WriteLine("{0}: {1}".format(COMMAND_NAME, message))


def _spacing():
    unit_scale = Rhino.RhinoMath.UnitScale(
        Rhino.UnitSystem.Millimeters,
        sc.doc.ModelUnitSystem,
    )
    return max(LAYOUT_SPACING_MM * unit_scale, sc.doc.ModelAbsoluteTolerance * 100.0)


def _area(geometry):
    properties = Rhino.Geometry.AreaMassProperties.Compute(geometry)
    if properties is None:
        return None
    try:
        return properties.Area
    finally:
        properties.Dispose()


def _sum_area(breps):
    total = 0.0
    for brep in breps:
        value = _area(brep)
        if value is None or math.isnan(value) or math.isinf(value):
            return None
        total += value
    return total


def _duplicate_as_breps(object_id, tolerance):
    rhino_object = sc.doc.Objects.FindId(object_id)
    if rhino_object is None:
        return tuple(), None, None

    geometry = rhino_object.Geometry
    if isinstance(geometry, Rhino.Geometry.Brep):
        brep = geometry.DuplicateBrep()
    elif isinstance(geometry, Rhino.Geometry.Extrusion):
        brep = geometry.ToBrep(True)
    elif isinstance(geometry, Rhino.Geometry.Surface):
        brep = geometry.ToBrep()
    else:
        return tuple(), None, None

    if brep is not None and not brep.IsValid:
        repaired = brep.DuplicateBrep()
        repaired.Standardize()
        repaired.Compact()
        if repaired.Repair(tolerance) and repaired.IsValid:
            return (repaired,), rhino_object.Attributes.Duplicate(), "REPAIRED"

    return ((brep,) if brep is not None else tuple()), rhino_object.Attributes.Duplicate(), None


def _valid_source(brep):
    if brep is None or not brep.IsValid:
        return False, "INVALID_BREP"
    if not brep.IsManifold:
        return False, "NON_MANIFOLD_BREP"
    if brep.Faces.Count == 0:
        return False, "EMPTY_BREP"
    return True, None


def _perform_unroll(brep, absolute_tolerance, relative_tolerance):
    unroller = Rhino.Geometry.Unroller(brep)
    unroller.AbsoluteTolerance = absolute_tolerance
    unroller.RelativeTolerance = relative_tolerance
    unroller.ExplodeOutput = False
    try:
        result = unroller.PerformUnroll()
    except Exception:
        return tuple()

    if not result or not result[0]:
        return tuple()

    flat = tuple(item for item in result[0] if item is not None and item.IsValid)
    if not flat:
        return tuple()

    return flat


def _flat_within_tolerance(breps, tolerance):
    minimum = None
    maximum = None
    for brep in breps:
        box = brep.GetBoundingBox(True)
        if not box.IsValid:
            return False
        minimum = box.Min.Z if minimum is None else min(minimum, box.Min.Z)
        maximum = box.Max.Z if maximum is None else max(maximum, box.Max.Z)
    if minimum is None or maximum is None:
        return False
    return maximum - minimum <= max(tolerance * 10.0, 1e-7)


def _validate_result(source, output, tolerance, relative_tolerance):
    if not output:
        return False, "NO_OUTPUT"
    if not _flat_within_tolerance(output, tolerance):
        return False, "NOT_FLAT"

    source_area = _area(source)
    output_area = _sum_area(output)
    if source_area is None or output_area is None or source_area <= 0.0:
        return False, "INVALID_AREA"

    ratio = abs(output_area - source_area) / source_area
    limit = max(relative_tolerance, 1e-7)
    if ratio > limit:
        return False, "AREA_MISMATCH"
    return True, None


def _group_box(breps):
    box = Rhino.Geometry.BoundingBox.Empty
    for brep in breps:
        box.Union(brep.GetBoundingBox(True))
    return box


def _rotation_box(brep, angle):
    rotation = Rhino.Geometry.Transform.Rotation(
        angle,
        Rhino.Geometry.Vector3d.ZAxis,
        Rhino.Geometry.Point3d.Origin,
    )
    duplicate = brep.DuplicateBrep()
    duplicate.Transform(rotation)
    return duplicate, duplicate.GetBoundingBox(True)


def _best_rotation(brep):
    best_angle = 0.0
    best_score = float("inf")
    for deg in range(0, 90, 5):
        angle = Rhino.RhinoMath.ToRadians(deg)
        _, box = _rotation_box(brep, angle)
        width = box.Max.X - box.Min.X
        height = box.Max.Y - box.Min.Y
        score = width * height
        if score < best_score:
            best_score = score
            best_angle = angle
    return best_angle


def _rotate_to_compact(brep):
    angle = _best_rotation(brep)
    if abs(angle) <= Rhino.RhinoMath.ZeroTolerance:
        return brep.DuplicateBrep()
    rotation = Rhino.Geometry.Transform.Rotation(
        angle,
        Rhino.Geometry.Vector3d.ZAxis,
        Rhino.Geometry.Point3d.Origin,
    )
    duplicate = brep.DuplicateBrep()
    duplicate.Transform(rotation)
    return duplicate


def _translate_brep(brep, x, y, z=0.0):
    transform = Rhino.Geometry.Transform.Translation(x, y, z)
    duplicate = brep.DuplicateBrep()
    duplicate.Transform(transform)
    return duplicate


def _compact_layout(breps, spacing):
    if not breps:
        return tuple()

    placed = []
    cursor_x = 0.0
    cursor_y = 0.0
    row_height = 0.0

    for brep in breps:
        compact = _rotate_to_compact(brep)
        box = compact.GetBoundingBox(True)
        width = box.Max.X - box.Min.X
        height = box.Max.Y - box.Min.Y

        if placed and cursor_x + width + spacing > 1500.0:
            cursor_x = 0.0
            cursor_y += row_height + spacing
            row_height = 0.0

        moved = _translate_brep(compact, cursor_x - box.Min.X, cursor_y - box.Min.Y)
        placed.append(moved)

        cursor_x += width + spacing
        row_height = max(row_height, height)

    return tuple(placed)


def _move_group_to_point(breps, target_point):
    if not breps:
        return tuple()
    box = _group_box(breps)
    if not box.IsValid:
        return tuple(breps)

    delta_x = target_point.X - box.Min.X
    delta_y = target_point.Y - box.Min.Y
    delta_z = target_point.Z - box.Min.Z
    transform = Rhino.Geometry.Transform.Translation(delta_x, delta_y, delta_z)

    moved = []
    for brep in breps:
        duplicate = brep.DuplicateBrep()
        duplicate.Transform(transform)
        moved.append(duplicate)
    return tuple(moved)


def _add_output(breps, attributes=None):
    identifiers = []
    for brep in breps:
        if brep is None:
            continue
        attributes_obj = attributes or Rhino.DocObjects.ObjectAttributes()
        attributes_obj = attributes_obj.Duplicate()
        identifiers.append(sc.doc.Objects.AddBrep(brep, attributes_obj))
    return tuple(identifiers)


def _pick_object_ids():
    object_ids = rs.GetObjects(
        "Chọn Surface, Polysurface hoặc Extrusion để xếp phẳng",
        filter=rs.filter.surface | rs.filter.polysurface | rs.filter.extrusion,
        preselect=False,
        group=False,
        select=True,
    )
    if not object_ids:
        return tuple()
    return tuple(object_ids)


def run_command():
    object_ids = _pick_object_ids()
    if not object_ids:
        _write("Không có đối tượng nào được chọn.")
        return

    tolerance = sc.doc.ModelAbsoluteTolerance
    relative_tolerance = max(sc.doc.ModelRelativeTolerance, 1e-7)
    spacing = _spacing()

    source_breps = []
    failures = []

    for object_id in object_ids:
        breps, attributes, notice = _duplicate_as_breps(object_id, tolerance)
        if not breps:
            failures.append((object_id, "NO_VALID_BREP"))
            continue

        for brep in breps:
            valid, error = _valid_source(brep)
            if not valid:
                failures.append((object_id, error))
                continue

            flat = _perform_unroll(brep, tolerance, relative_tolerance)
            if not flat:
                failures.append((object_id, "UNROLL_FAILED"))
                continue

            passed, reason = _validate_result(brep, flat, tolerance, relative_tolerance)
            if not passed:
                failures.append((object_id, reason))
                continue

            source_breps.extend(flat)

    if not source_breps:
        _write("Không tạo được kết quả nào hợp lệ.")
        return

    arranged = _compact_layout(source_breps, spacing)
    if not arranged:
        _write("Không thể xếp bố cục kết quả.")
        return

    target_point = rs.GetPoint("Chọn vị trí đặt kết quả")
    if target_point is None:
        _write("Đã hủy đặt kết quả.")
        return

    arranged = _move_group_to_point(arranged, Rhino.Geometry.Point3d(target_point))

    rs.EnableRedraw(False)
    try:
        created_ids = _add_output(arranged)
    finally:
        rs.EnableRedraw(True)

    _write(
        "version={0} selected={1} created={2} failures={3} spacing_mm={4}".format(
            VERSION,
            len(object_ids),
            len(created_ids),
            len(failures),
            spacing,
        )
    )

    if failures:
        _write("Một số đối tượng không tạo được mặt phẳng: {0}".format(failures))

    if RUN_SC_AFTER_UNROLL and created_ids:
        try:
            sc_outputs = _run_sc(created_ids)
            _write("SC hoan thanh: contour={0}; soi/chan={1}; danh_ma={2}.".format(
                len(sc_outputs.get("borders", [])),
                len(sc_outputs.get("inner_curves", [])),
                len(sc_outputs.get("marks", []))))
        except Exception as error:
            _write("SC loi; khong chay CHAN: {0}".format(error))
            return

        if RUN_CHAN_AFTER_SC:
            # CHAN keeps its own selection prompt. Processing unrelated flat
            # parts in one Boolean operation can discard contours, so select
            # one valid contour + DUONG_CHAN/SOI group at a time.
            try:
                _run_chan()
            except Exception as error:
                _write("Khong khoi dong duoc CHAN: {0}".format(error))


if __name__ == "__main__":
    run_command()

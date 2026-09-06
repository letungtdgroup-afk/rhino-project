# -*- coding: utf-8 -*-
"""CHAN v2.1.1 - base v1.4, giu du hai contour cho ca 2 hinh + 1 duong chan.

Moi DUONG_CHAN tao hai HCN 2 x 0.2 mm, mot HCN cho moi endpoint. Truong hop
thuong dat goc HCN lui ra ngoai endpoint 0.1 mm
theo huong nguoc vector tu endpoint vao dau con lai. TH3B va TH4 van dung
logic lui dong cu. Endpoint TH3B/TH4 duoc mac dinh xac nhan ma khong dem dau
U tren contour cuoi; cac truong hop khac van phai co dau U. KXD hoac thieu
mot endpoint duoc xac nhan thi giu DUONG_CHAN de nguoi dung bo sung dinh nghia.
"""
from __future__ import print_function

import datetime
import os
import time
import traceback

import Rhino  # type: ignore[reportMissingImports]
import scriptcontext as sc  # type: ignore[reportMissingImports]
import System  # type: ignore[reportMissingImports]


VERSION = "2.1.1"
COMMAND_NAME = "CHAN"
BEND_LAYER_LEAVES = ("DUONG_CHAN", "DANHMA", "SOI")
MARK_LENGTH = 2.0
MARK_WIDTH = 0.2
MIN_W = 0.01
MAX_SHIFT = 24.0
SHIFT_STEP = 2.0
MAX_SHIFT_RETRIES = 12
TH1B_MAX_RETREAT = MAX_SHIFT
NORMAL_RETREAT = 0.1
MAX_U_RAIL = 2.1
DEBUG_ROOT = "BOC::CHAN_DEBUG"
SCRIPT_PATH = os.path.abspath(globals().get("__file__", "CHAN_v1.4.py"))
LOG_PATH = os.path.splitext(SCRIPT_PATH)[0] + "_log.md"


class ChanError(Exception):
    pass


FINAL_STEP = "Khoi dong"


def _set_step(step):
    global FINAL_STEP
    FINAL_STEP = step


def _write(message):
    line = "CHAN v{0}: {1}".format(VERSION, message)
    Rhino.RhinoApp.WriteLine(line)
    try:
        entry = "{0} {1}{2}".format(
            datetime.datetime.now().isoformat(), line,
            System.Environment.NewLine)
        System.IO.File.AppendAllText(
            LOG_PATH, entry, System.Text.Encoding.UTF8)
    except Exception as error:
        Rhino.RhinoApp.WriteLine("CHAN v{0}: KHONG GHI LOG: {1}".format(
            VERSION, error))


def _geo_tol():
    return min(max(sc.doc.ModelAbsoluteTolerance, 1e-6), 0.001)


def _node_tol():
    return max(sc.doc.ModelAbsoluteTolerance * 2.0, 0.001)


def _copy_point(point):
    return Rhino.Geometry.Point3d(point)


def _segments(curve):
    items = curve.DuplicateSegments()
    return list(items) if items and len(items) else [curve.DuplicateCurve()]


def _unit(vector):
    result = Rhino.Geometry.Vector3d(vector)
    return result if result.Unitize() else None


def _bend_axis(bend, end_index):
    p0 = bend.PointAtStart if end_index == 0 else bend.PointAtEnd
    other = bend.PointAtEnd if end_index == 0 else bend.PointAtStart
    return _unit(other - p0)


def _make_marker(origin, axis):
    axis = _unit(axis)
    if axis is None:
        return None
    side = _unit(Rhino.Geometry.Vector3d(-axis.Y, axis.X, 0.0))
    if side is None:
        return None
    half = MARK_WIDTH * 0.5
    far = origin + axis * MARK_LENGTH
    nl, nr = origin + side * half, origin - side * half
    fl, fr = far + side * half, far - side * half
    parts = {
        "side_left": Rhino.Geometry.LineCurve(nl, fl),
        "cap_far": Rhino.Geometry.LineCurve(fl, fr),
        "side_right": Rhino.Geometry.LineCurve(fr, nr),
        "cap_near": Rhino.Geometry.LineCurve(nr, nl),
    }
    order = ("side_left", "cap_far", "side_right", "cap_near")
    joined = Rhino.Geometry.Curve.JoinCurves(
        [parts[name] for name in order], _geo_tol())
    if not joined or len(joined) != 1 or not joined[0].IsClosed:
        return None
    parts.update({"curve": joined[0], "axis": axis,
                  "origin": _copy_point(origin), "far": _copy_point(far)})
    return parts


def _closest(curve, point, tolerance):
    result = curve.ClosestPoint(point)
    if not result or not result[0]:
        return None
    parameter = result[1]
    closest = curve.PointAt(parameter)
    if closest.DistanceTo(point) > tolerance:
        return None
    return parameter, closest


def _curve_records_at_point(curves, point):
    tolerance = _node_tol()
    records = []
    for curve in curves:
        closest = _closest(curve, point, tolerance)
        if closest is not None:
            records.append((curve, closest[0]))
    return records


def _curve_is_curved(curve, parameter):
    try:
        curvature = curve.CurvatureAt(parameter)
        if curvature.IsValid:
            return curvature.Length > 1e-9
    except Exception:
        pass
    return not curve.IsLinear(sc.doc.ModelAbsoluteTolerance)


def _intersection_points(first, second):
    tolerance = _geo_tol()
    events = Rhino.Geometry.Intersect.Intersection.CurveCurve(
        first, second, tolerance, tolerance)
    points = []
    for event in events or []:
        candidates = []
        if event.IsPoint:
            candidates.append(event.PointA)
        elif event.IsOverlap:
            candidates.extend((first.PointAt(event.OverlapA.T0),
                               first.PointAt(event.OverlapA.T1)))
        for point in candidates:
            if not any(point.DistanceTo(saved) <= tolerance
                       for saved in points):
                points.append(_copy_point(point))
    return points


def _point_covered(point, curves, tolerance):
    return any(_closest(curve, point, tolerance) is not None for curve in curves)


def _cap_coverage(cap, curves):
    """Measure covered intervals on a 0.2 mm cap using dense local sampling."""
    tolerance = max(_geo_tol() * 4.0, 0.002)
    sample_count = 101
    flags = []
    points = []
    for index in range(sample_count):
        fraction = index / float(sample_count - 1)
        point = cap.PointAt(cap.Domain.ParameterAt(fraction))
        points.append(point)
        flags.append(_point_covered(point, curves, tolerance))
    intervals = []
    start = None
    for index, covered in enumerate(flags):
        if covered and start is None:
            start = index
        if start is not None and (not covered or index == sample_count - 1):
            end = index if covered and index == sample_count - 1 else index - 1
            p1, p2 = points[start], points[end]
            width = p1.DistanceTo(p2)
            if width > tolerance:
                intervals.append((width, _copy_point(p1), _copy_point(p2)))
            start = None
    # Exact intersections improve interval endpoints when Rhino supplies them.
    hits = []
    for curve in curves:
        hits.extend(_intersection_points(cap, curve))
    unique = []
    for point in hits:
        if not any(point.DistanceTo(saved) <= tolerance for saved in unique):
            unique.append(point)
    if len(unique) >= 2:
        for i in range(len(unique)):
            for j in range(i + 1, len(unique)):
                middle = Rhino.Geometry.Point3d(
                    (unique[i].X + unique[j].X) * 0.5,
                    (unique[i].Y + unique[j].Y) * 0.5,
                    (unique[i].Z + unique[j].Z) * 0.5)
                if _point_covered(middle, curves, tolerance):
                    intervals.append((unique[i].DistanceTo(unique[j]),
                                      unique[i], unique[j]))
    return max(intervals, key=lambda item: item[0]) if intervals else None


def _curve_endpoint_direction(curve, point):
    """Direction from an endpoint into its finite source curve."""
    tolerance = _node_tol()
    if curve.PointAtStart.DistanceTo(point) <= tolerance:
        return _unit(curve.PointAtEnd - curve.PointAtStart)
    if curve.PointAtEnd.DistanceTo(point) <= tolerance:
        return _unit(curve.PointAtStart - curve.PointAtEnd)
    return None


def _cap_midpoint_and_corner(cap, coverage):
    """Return exact cap midpoint/corner despite 0.002-mm sample padding."""
    if coverage is None:
        return None
    # Dense coverage sampling expands a true 0.100-mm half cap to about
    # 0.104 mm.  Use that measured band only for TH1B's shape signature,
    # then return the exact geometric midpoint/corner for later intersections.
    half_tolerance = max(_node_tol() * 6.0, MARK_WIDTH * 0.06)
    if abs(coverage[0] - MARK_WIDTH * 0.5) > half_tolerance:
        return None
    tolerance = half_tolerance
    first, second = coverage[1], coverage[2]
    midpoint = cap.PointAt(cap.Domain.ParameterAt(0.5))
    endpoints = (cap.PointAtStart, cap.PointAtEnd)
    for middle, corner in ((first, second), (second, first)):
        if middle.DistanceTo(midpoint) > tolerance:
            continue
        if any(corner.DistanceTo(endpoint) <= tolerance
               for endpoint in endpoints):
            exact_corner = min(endpoints,
                               key=lambda endpoint: corner.DistanceTo(endpoint))
            return _copy_point(midpoint), _copy_point(exact_corner)
    return None


def _forward_long_side_hit(marker, point, direction):
    """Intersect the supporting line at P-mid with the retreated HCN."""
    direction = _unit(direction)
    if direction is None:
        return None
    reach = MARK_LENGTH + TH1B_MAX_RETREAT + 1.0
    line = Rhino.Geometry.LineCurve(
        point - direction * reach, point + direction * reach)
    candidates = []
    for side_name in ("side_left", "side_right"):
        for hit in _intersection_points(line, marker[side_name]):
            delta = hit - point
            distance = delta * direction
            if distance <= _node_tol():
                continue
            # The extension must cut the interior of a 2-mm side, not touch
            # its cap corner.  Corner contacts belong to the other TH1B arm.
            if (hit.DistanceTo(marker[side_name].PointAtStart) <= _node_tol() or
                    hit.DistanceTo(marker[side_name].PointAtEnd) <= _node_tol()):
                continue
            candidates.append((distance, _copy_point(hit)))
    return min(candidates, key=lambda item: item[0])[1] if candidates else None


def _curve_cuts_long_side_interior(curve, side):
    """A cap-corner touch is not a cut through the 2-mm HCN side."""
    tolerance = _node_tol()
    for hit in _intersection_points(curve, side):
        if (hit.DistanceTo(side.PointAtStart) > tolerance and
                hit.DistanceTo(side.PointAtEnd) > tolerance):
            return True
    return False


def _marker_corners(marker):
    return [marker["cap_near"].PointAtStart,
            marker["cap_near"].PointAtEnd,
            marker["cap_far"].PointAtStart,
            marker["cap_far"].PointAtEnd]


def _retreat_from_long_side_rails(marker, source_point, source_direction):
    """Use P1/P2 support intersections with the two infinite long-side rails."""
    retreat_axis = Rhino.Geometry.Vector3d(marker["axis"])
    retreat_axis *= -1.0
    retreat_axis = _unit(retreat_axis)
    direction = _unit(source_direction)
    if retreat_axis is None or direction is None:
        return None
    reach = MARK_LENGTH + TH1B_MAX_RETREAT + 1.0
    support = Rhino.Geometry.LineCurve(
        source_point - direction * reach,
        source_point + direction * reach)
    candidates = []
    for side_name in ("side_left", "side_right"):
        side = marker[side_name]
        side_axis = _unit(side.PointAtEnd - side.PointAtStart)
        if side_axis is None:
            continue
        rail = Rhino.Geometry.LineCurve(
            side.PointAtStart - side_axis * TH1B_MAX_RETREAT,
            side.PointAtEnd + side_axis * TH1B_MAX_RETREAT)
        for hit in _intersection_points(rail, support):
            for corner in (side.PointAtStart, side.PointAtEnd):
                distance = (hit - corner) * retreat_axis
                if distance <= _node_tol() or distance > TH1B_MAX_RETREAT:
                    continue
                candidate = _make_marker(
                    marker["origin"] + retreat_axis * distance,
                    marker["axis"])
                if candidate is None:
                    continue
                matching = [item for item in _marker_corners(candidate)
                            if item.DistanceTo(hit) <= _node_tol()]
                if matching:
                    candidates.append((distance, candidate,
                                       _copy_point(matching[0])))
    return min(candidates, key=lambda item: item[0]) if candidates else None


def _cap_branch_pair(cap, branch_records, require_midpoint):
    midpoint = cap.PointAt(cap.Domain.ParameterAt(0.5))
    hits = []
    for curve, parameter in branch_records:
        for point in _intersection_points(cap, curve):
            hits.append((point, curve, parameter))
    candidates = []
    for i in range(len(hits)):
        for j in range(i + 1, len(hits)):
            if hits[i][1] is hits[j][1]:
                continue
            p1, p2 = hits[i][0], hits[j][0]
            if require_midpoint and min(
                    p1.DistanceTo(midpoint),
                    p2.DistanceTo(midpoint)) > _node_tol():
                continue
            candidates.append((p1.DistanceTo(p2), p1, p2, midpoint,
                               hits[i][1], hits[j][1]))
    if not candidates:
        return None
    valid = [item for item in candidates if item[0] >= MIN_W]
    return min(valid or candidates, key=lambda item: item[0])


def _candidate_shifts(p0, axis, branch_records):
    """Test P0, then shift exactly 2 mm per retry, at most twelve times."""
    unused = p0, axis, branch_records
    values = []
    for retry in range(MAX_SHIFT_RETRIES + 1):
        distance = retry * SHIFT_STEP
        for sign in (1.0, -1.0):
            values.append((sign, distance))
    return values


def _result(case_name, bend_id, bend_index, end_index, p0, marker,
            cap_name, w_record, shift, sign, reason=""):
    valid = bool(w_record and w_record[0] >= MIN_W)
    result = {
        "valid": valid, "case": case_name,
        "reason": reason, "bend_id": bend_id,
        "bend_index": bend_index, "end_index": end_index,
        "p0": _copy_point(p0), "marker": marker,
        "cap_name": cap_name, "shift": shift, "sign": sign,
    }
    if w_record:
        result.update({"w": w_record[0], "p1": _copy_point(w_record[1]),
                       "p2": _copy_point(w_record[2])})
    else:
        result["w"] = 0.0
    if not valid and not result["reason"]:
        result["reason"] = "W={0:.6f} < {1:.3f}".format(
            result["w"], MIN_W)
    return result


def _analyze_th5(bend_id, bend_index, end_index, p0, marker, branches):
    """TH5: hai nhanh nguon THANG (cuc bo tai P0) gap nhau tai goc nhon.

    Dau hieu nhan biet: giao diem cua canh ngan 0.2mm gan (cap_near) voi
    hai nhanh gan nhu trung nhau va trung P0 (W~0, P1~P2~P0). Chi phat
    hien/gan nhan TH5A o day; phan biet TH5A/TH5B (hai HCN trung vi tri do
    hai DUONG_CHAN khac nhau cung cham 1 diem goc) duoc lam o buoc hau xu
    ly _classify_th5b(). Ket qua tra ve luon "valid": False vi chua co cach
    xu ly Boolean/trim cho TH5 - chi phuc vu kiem tra phat hien.
    """
    pair = _cap_branch_pair(marker["cap_near"], branches, False)
    if pair is None:
        return None
    w, p1, p2 = pair[0], pair[1], pair[2]
    if w > MIN_W:
        return None
    node_tolerance = _node_tol()
    if (p1.DistanceTo(p0) > node_tolerance or
            p2.DistanceTo(p0) > node_tolerance):
        return None
    return _result(
        "TH5A", bend_id, bend_index, end_index, p0, marker, "cap_near",
        (w, p1, p2), 0.0, 1.0,
        "TH5: 2 nhanh thang gap tai P0, W={0:.6f} (~0 tai canh ngan HCN); "
        "chua code cach xu ly Boolean/trim, can xac nhan truoc khi Apply"
        .format(w))


def _analyze_th1b(bend_id, bend_index, end_index, p0, marker,
                  source_curves, coverages):
    """Resolve only TH1B by retreating a HCN corner onto the P1/P2 support."""
    for cap_name, coverage in coverages:
        pattern = _cap_midpoint_and_corner(marker[cap_name], coverage)
        if pattern is None:
            continue
        midpoint, corner = pattern
        midpoint_curves = [curve for curve in source_curves
                           if _curve_endpoint_direction(curve, midpoint)]
        if not midpoint_curves:
            continue
        # TH1 (not TH1B) has a real source cut through a long HCN side.
        if any(_curve_cuts_long_side_interior(curve, marker["side_left"]) or
               _curve_cuts_long_side_interior(curve, marker["side_right"])
               for curve in midpoint_curves):
            continue

        # One finite branch overlaps midpoint-to-corner on the 0.2 cap.  The
        # other branch at that midpoint is the P1/P2 line that determines the
        # exact retreat corner (image case 1 -> case 2).
        carrier_curves = [curve for curve in midpoint_curves
                          if _closest(curve, corner, _node_tol()) is not None]
        if not carrier_curves:
            continue
        midpoint_curve = carrier_curves[0]
        midpoint_direction = _curve_endpoint_direction(midpoint_curve, midpoint)
        remaining = [(midpoint, curve) for curve in midpoint_curves
                     if curve is not midpoint_curve]
        # This fallback keeps the same rule valid if the second P point is a
        # separate endpoint at the cap corner rather than at the midpoint.
        if not remaining:
            remaining = [(corner, curve) for curve in source_curves
                         if curve is not midpoint_curve and
                         _curve_endpoint_direction(curve, corner)]
        candidates = []
        for remaining_point, remaining_curve in remaining:
            corner_direction = _curve_endpoint_direction(
                remaining_curve, remaining_point)
            retreat = _retreat_from_long_side_rails(
                marker, remaining_point, corner_direction)
            if retreat is None:
                continue
            distance, retreated, retreated_corner = retreat
            extension_end = _forward_long_side_hit(
                retreated, midpoint, midpoint_direction)
            if extension_end is not None:
                candidates.append((distance, retreated, retreated_corner,
                                   extension_end, remaining_point))
        if candidates:
            distance, retreated, retreated_corner, extension_end, remaining_point = min(
                candidates, key=lambda item: item[0])
            result = _result(
                "TH1B", bend_id, bend_index, end_index, p0, retreated,
                cap_name, (extension_end.DistanceTo(retreated_corner),
                           extension_end, retreated_corner), distance, -1.0,
                "TH1B: lui HCN {0:.6f} mm de goc HCN nam tren duong "
                "chua P1/P2".format(distance))
            result["th1b_extensions"] = [
                {"start": _copy_point(midpoint),
                 "end": _copy_point(extension_end)},
                {"start": _copy_point(remaining_point),
                 "end": _copy_point(retreated_corner)},
            ]
            return result

        # No safe retreat is available.  Keep the original HCN and remove
        # its observed half-cap W (P1-P2) from both Boolean inputs instead.
        result = _result(
            "TH1B", bend_id, bend_index, end_index, p0, marker,
            cap_name, (midpoint.DistanceTo(corner), midpoint, corner),
            0.0, -1.0,
            "TH1B fallback: khong tim duoc khoang lui; trim W P1-P2 "
            "tren HCN va curve nguon")
        result["th1b_trim_w"] = True
        return result
    return None


def _analyze_th3b_th4_first(bend_id, bend_index, end_index, p0, axis,
                            source_curves):
    """Chi giu logic lui dong cu cho TH3B va TH4, truoc HCN thuong."""
    branches = _curve_records_at_point(source_curves, p0)
    if len(branches) != 2:
        return None
    curved_count = sum(1 for curve, parameter in branches
                       if _curve_is_curved(curve, parameter))
    case_name = "TH3" if curved_count == 1 else (
        "TH4" if curved_count == 2 else None)
    if case_name is None:
        return None

    best_invalid = None
    attempts = []
    for sign, shift in _candidate_shifts(p0, axis, branches):
        direction = Rhino.Geometry.Vector3d(axis)
        direction *= sign
        candidate = _make_marker(p0 + direction * shift, axis)
        attempt = {
            "marker": candidate, "shift": shift, "sign": sign,
            "w": 0.0, "p1": None, "p2": None,
        }
        for cap_name in ("cap_near", "cap_far"):
            pair = _cap_branch_pair(
                candidate[cap_name], branches, case_name != "TH4")
            if pair is None:
                continue
            if pair[0] > attempt["w"]:
                attempt.update({"w": pair[0],
                                "p1": _copy_point(pair[1]),
                                "p2": _copy_point(pair[2])})
            if best_invalid is None or pair[0] > best_invalid[0][0]:
                best_invalid = (pair, candidate, cap_name, shift, sign)
            if pair[0] < MIN_W:
                continue
            # TH3 tai P0 la TH3A va di theo HCN thuong lui ngoai 0.1 mm.
            if case_name == "TH3" and shift <= _geo_tol():
                return None
            resolved = "TH3B" if case_name == "TH3" else "TH4"
            return _result(resolved, bend_id, bend_index, end_index, p0,
                           candidate, cap_name, pair, shift, sign)
        attempts.append(attempt)

    invalid_pair = best_invalid[0] if best_invalid else None
    invalid_marker = best_invalid[1] if best_invalid else _make_marker(p0, axis)
    invalid_cap = best_invalid[2] if best_invalid else None
    invalid_shift = best_invalid[3] if best_invalid else 0.0
    invalid_sign = best_invalid[4] if best_invalid else 1.0
    invalid_case = "TH3B" if case_name == "TH3" else "TH4"
    result = _result(
        invalid_case, bend_id, bend_index, end_index, p0,
        invalid_marker, invalid_cap, invalid_pair, invalid_shift, invalid_sign,
        "da lui tung buoc {0:g} mm, du {1} lan/{2:g} mm; "
        "van khong tim duoc W >= {3:.3f}"
        .format(SHIFT_STEP, MAX_SHIFT_RETRIES, MAX_SHIFT, MIN_W))
    result["attempts"] = attempts
    return result


def _analyze_endpoint(bend_record, bend_index, end_index, source_curves):
    bend_id, bend, unused_attributes = bend_record
    p0 = bend.PointAtStart if end_index == 0 else bend.PointAtEnd
    axis = _bend_axis(bend, end_index)
    if axis is None:
        return _result("KXD", bend_id, bend_index, end_index, p0,
                       None, None, None, 0.0, 1.0,
                       "khong lay duoc vector DUONG_CHAN")

    special = _analyze_th3b_th4_first(
        bend_id, bend_index, end_index, p0, axis, source_curves)
    if special is not None:
        return special

    # axis huong tu endpoint vao dau con lai; dau tru la huong lui ra ngoai.
    origin = p0 - axis * NORMAL_RETREAT
    marker = _make_marker(origin, axis)
    if marker is None:
        return _result("KXD", bend_id, bend_index, end_index, p0,
                       None, None, None, 0.0, -1.0,
                       "khong tao duoc HCN 2x0.2 lui ngoai 0.1 mm")
    cap = marker["cap_near"]
    pair = (cap.GetLength(), cap.PointAtStart, cap.PointAtEnd)
    return _result("THUONG", bend_id, bend_index, end_index, p0,
                   marker, "cap_near", pair, NORMAL_RETREAT, -1.0,
                   "HCN lui nguoc vector DUONG_CHAN 0.1 mm")


def _markers_intersect(first, second):
    events = Rhino.Geometry.Intersect.Intersection.CurveCurve(
        first["marker"]["curve"], second["marker"]["curve"],
        _geo_tol(), _geo_tol())
    return bool(events and any(event.IsPoint or event.IsOverlap
                               for event in events))


def _classify_th2(results):
    limit = System.Math.Sin(Rhino.RhinoMath.ToRadians(1.0))
    candidates = [item for item in results if (
        item["case"] == "TH1" and item["valid"]) or
        (item["case"] == "TH1B" and
         (not item["valid"] or item.get("th1b_trim_w")))]
    for i in range(len(candidates)):
        for j in range(i + 1, len(candidates)):
            first, second = candidates[i], candidates[j]
            if abs(first["marker"]["axis"] *
                   second["marker"]["axis"]) > limit:
                continue
            if not _markers_intersect(first, second):
                continue
            for item in (first, second):
                item["case"] = "TH2"
                # TH2 takes precedence over the TH1B fallback.  Its Boolean
                # rule already determines the correct trim around P0.
                item.pop("th1b_trim_w", None)
                if not item["valid"]:
                    # TH2 uses the intersecting HCN pair directly; unlike
                    # TH1B it has no separate P1/P2 retreat requirement.
                    item["valid"] = True
                    item["p1"] = _copy_point(item["p0"])
                    item["p2"] = _copy_point(item["p0"])
                    item["reason"] = "TH2: HCN vuong goc giao nhau; uu tien TH2 truoc TH1B"


def _classify_th5b(results):
    """TH5B: hai endpoint TH5A cua hai DUONG_CHAN khac nhau cung chung P0.

    Ca hai se dat mot HCN o dung diem goc, nen tao ra hai HCN trung vi tri
    nhau. TH5A con lai (khong co ban sao P0) giu nguyen nhan TH5A.
    """
    tolerance = _node_tol()
    th5 = [item for item in results if item["case"] == "TH5A"]
    for i in range(len(th5)):
        for j in range(i + 1, len(th5)):
            if th5[i]["bend_index"] == th5[j]["bend_index"]:
                continue
            if th5[i]["p0"].DistanceTo(th5[j]["p0"]) <= tolerance:
                th5[i]["case"] = "TH5B"
                th5[j]["case"] = "TH5B"


def _layer(path, color):
    index = sc.doc.Layers.FindByFullPath(path, -1)
    if index >= 0:
        return index
    parent_path, separator, leaf = path.rpartition("::")
    parent_id = System.Guid.Empty
    if separator:
        parent_id = sc.doc.Layers[_layer(parent_path, color)].Id
    layer = Rhino.DocObjects.Layer()
    layer.Name = leaf if separator else path
    layer.ParentLayerId = parent_id
    layer.Color = color
    return sc.doc.Layers.Add(layer)


def _debug_attributes(case_name):
    colors = {"TH1": System.Drawing.Color.LimeGreen,
              "TH1B": System.Drawing.Color.GreenYellow,
              "TH2": System.Drawing.Color.YellowGreen,
              "TH3A": System.Drawing.Color.Orange,
              "TH3B": System.Drawing.Color.OrangeRed,
              "TH4": System.Drawing.Color.DeepSkyBlue,
              "TH5A": System.Drawing.Color.Purple,
              "TH5B": System.Drawing.Color.DarkMagenta,
              "KXD": System.Drawing.Color.Red}
    attributes = Rhino.DocObjects.ObjectAttributes()
    attributes.LayerIndex = _layer(
        DEBUG_ROOT + "::" + case_name,
        colors.get(case_name, System.Drawing.Color.Red))
    return attributes


def _clear_old_debug():
    """Remove only diagnostic objects created by earlier CHAN1 runs."""
    prefix = DEBUG_ROOT.upper()
    for layer in sc.doc.Layers:
        if layer is None or not layer.FullPath.upper().startswith(prefix):
            continue
        objects = sc.doc.Objects.FindByLayer(layer)
        for obj in list(objects) if objects else []:
            sc.doc.Objects.Delete(obj.Id, True)


def _draw_debug(results):
    for number, item in enumerate(results, 1):
        attributes = _debug_attributes(item["case"])
        attempts = item.get("attempts") or []
        for attempt in attempts:
            attempt_marker = attempt.get("marker")
            if attempt_marker:
                sc.doc.Objects.AddCurve(
                    attempt_marker["curve"].DuplicateCurve(), attributes)
            for point_name in ("p1", "p2"):
                point = attempt.get(point_name)
                if point:
                    sc.doc.Objects.AddPoint(point, attributes)
            if attempt_marker:
                attempt_text = "TH4 sign={0:+.0f} lui={1:g} W={2:.6f}".format(
                    attempt["sign"], attempt["shift"], attempt["w"])
                anchor = attempt_marker["curve"].GetBoundingBox(True).Center
                sc.doc.Objects.AddTextDot(
                    attempt_text, anchor, attributes)
        marker = item.get("marker")
        if marker and not attempts:
            sc.doc.Objects.AddCurve(marker["curve"].DuplicateCurve(), attributes)
        for name in ("p0", "p1", "p2"):
            if item.get(name):
                sc.doc.Objects.AddPoint(item[name], attributes)
        text = "Dau {0} {1} W={2:.4f} sign={3:+.0f} lui={4:.4f}".format(
            number, item["case"], item["w"], item["sign"], item["shift"])
        if not item["valid"]:
            text += " LOI " + item["reason"]
        anchor = marker["curve"].GetBoundingBox(True).Center if marker else item["p0"]
        sc.doc.Objects.AddTextDot(text, anchor, attributes)
    sc.doc.Views.Redraw()


def _split_open_or_closed(curve, first_point, second_point):
    work = curve.DuplicateCurve()
    tolerance = _node_tol()
    if work.IsClosed:
        domain = work.Domain
        candidates = [domain.ParameterAt(value) for value in (0.25, 0.5, 0.75)]
        seam = max(candidates, key=lambda value: min(
            work.PointAt(value).DistanceTo(first_point),
            work.PointAt(value).DistanceTo(second_point)))
        work.ChangeClosedCurveSeam(seam)
    first = _closest(work, first_point, tolerance)
    second = _closest(work, second_point, tolerance)
    if first is None or second is None:
        return []
    values = sorted((first[0], second[0]))
    pieces = work.Split(values)
    return [piece for piece in (list(pieces) if pieces else [])
            if piece.GetLength() > _geo_tol()]


def _trim_w_from_marker(item):
    cap = item["marker"][item["cap_name"]]
    cap_parts = _split_open_or_closed(cap, item["p1"], item["p2"])
    midpoint = Rhino.Geometry.Point3d(
        (item["p1"].X + item["p2"].X) * 0.5,
        (item["p1"].Y + item["p2"].Y) * 0.5,
        (item["p1"].Z + item["p2"].Z) * 0.5)
    covered = [part for part in cap_parts
               if _closest(part, midpoint, _node_tol()) is not None]
    if covered:
        remove = min(covered, key=lambda part: abs(part.GetLength() - item["w"]))
        kept_cap = [part for part in cap_parts if part is not remove]
    elif item["w"] >= MARK_WIDTH - _geo_tol():
        # P1/P2 coincide with both endpoints: the whole 0.2-mm cap is W.
        kept_cap = []
    else:
        raise ChanError("khong trim chinh xac W khoi canh ngan HCN")
    other_names = [name for name in
                   ("side_left", "cap_far", "side_right", "cap_near")
                   if name != item["cap_name"]]
    return ([item["marker"][name].DuplicateCurve() for name in other_names] +
            [curve.DuplicateCurve() for curve in kept_cap])


def _trim_w_from_sources(item, source_records):
    """Trim only source pieces lying between P1/P2; open curves are valid."""
    if item["case"] == "TH2":
        return _trim_th2_sources(item, source_records)
    affected = []
    for record in source_records:
        object_id, curve, attributes = record
        has_p1 = _closest(curve, item["p1"], _node_tol()) is not None
        has_p2 = _closest(curve, item["p2"], _node_tol()) is not None
        if item["case"] in ("TH1", "TH2") or item.get("th1b_trim_w"):
            if has_p1 and has_p2:
                affected.append(record)
        elif has_p1 or has_p2:
            affected.append(record)
    if not affected:
        raise ChanError("khong tim thay curve goc chua W")
    replacement = []
    if item["case"] in ("TH1", "TH2") or item.get("th1b_trim_w"):
        midpoint = Rhino.Geometry.Point3d(
            (item["p1"].X + item["p2"].X) * 0.5,
            (item["p1"].Y + item["p2"].Y) * 0.5,
            (item["p1"].Z + item["p2"].Z) * 0.5)
        trimmed = 0
        for object_id, curve, attributes in affected:
            length = curve.GetLength()
            if (abs(length - item["w"]) <= _node_tol() and
                    _closest(curve, midpoint, _node_tol()) is not None):
                trimmed += 1
                continue
            pieces = _split_open_or_closed(curve, item["p1"], item["p2"])
            covered = [part for part in pieces
                       if _closest(part, midpoint, _node_tol()) is not None]
            if not covered:
                raise ChanError(
                    "khong xac dinh duoc doan W tren curve goc")
            remove = min(
                covered, key=lambda part: abs(part.GetLength() - item["w"]))
            replacement.extend(
                (part.DuplicateCurve(), attributes.Duplicate())
                for part in pieces if part is not remove)
            trimmed += 1
        if trimmed == 0:
            raise ChanError("khong trim duoc W tren curve goc")
    else:
        for object_id, curve, attributes in affected:
            target = item["p1"] if _closest(
                curve, item["p1"], _node_tol()) is not None else item["p2"]
            pieces = _split_open_or_closed(curve, item["p0"], target)
            if not pieces:
                replacement.append((curve.DuplicateCurve(),
                                    attributes.Duplicate()))
                continue
            remove = min(pieces, key=lambda part: part.GetLength())
            replacement.extend((part.DuplicateCurve(), attributes.Duplicate())
                               for part in pieces if part is not remove)
    return affected, replacement


def _trim_th2_sources(item, source_records):
    """BOC v1.1.0 rule: trim P0 to nearest HCN hit on two incident arms."""
    incident = []
    for record in source_records:
        curve = record[1]
        if (curve.PointAtStart.DistanceTo(item["p0"]) <= _node_tol() or
                curve.PointAtEnd.DistanceTo(item["p0"]) <= _node_tol()):
            incident.append(record)
    selected = []
    for record in incident:
        hits = [point for point in _intersection_points(
            record[1], item["marker"]["curve"])
                if point.DistanceTo(item["p0"]) > _node_tol()]
        if hits:
            selected.append((record, min(
                hits, key=lambda point: point.DistanceTo(item["p0"]))))
    if len(selected) != 2:
        raise ChanError(
            "TH2 tai P0 can dung 2 nhanh co giao HCN; tim thay {0}".format(
                len(selected)))
    affected = []
    replacement = []
    for record, hit in selected:
        object_id, curve, attributes = record
        pieces = _split_open_or_closed(curve, item["p0"], hit)
        local = []
        for part in pieces:
            start, end = part.PointAtStart, part.PointAtEnd
            if ((start.DistanceTo(item["p0"]) <= _node_tol() and
                 end.DistanceTo(hit) <= _node_tol()) or
                    (end.DistanceTo(item["p0"]) <= _node_tol() and
                     start.DistanceTo(hit) <= _node_tol())):
                local.append(part)
        if not local:
            raise ChanError("TH2 khong tach duoc khoang P0 den giao HCN")
        remove = min(local, key=lambda part: part.GetLength())
        affected.append(record)
        replacement.extend((part.DuplicateCurve(), attributes.Duplicate())
                           for part in pieces if part is not remove)
    return affected, replacement


def _join_without_boolean(source_records, marker_records):
    """Join all selected output while proving every marker belongs to a closed loop."""
    all_records = list(source_records) + list(marker_records)
    curves = [record[1] for record in all_records]
    if not curves:
        raise ChanError("khong co curve de Join")
    joined = Rhino.Geometry.Curve.JoinCurves(curves, _node_tol())
    joined = list(joined) if joined else []
    if not joined:
        raise ChanError("JoinCurves khong tra ve ket qua")
    input_length = sum(curve.GetLength() for curve in curves)
    output_length = sum(curve.GetLength() for curve in joined)
    length_tolerance = max(_node_tol() * len(curves), 0.001)
    if abs(input_length - output_length) > length_tolerance:
        raise ChanError(
            "Join lam thay doi tong chieu dai {0:.6f} mm".format(
                output_length - input_length))
    closed = [curve for curve in joined if curve.IsClosed]
    if not closed:
        raise ChanError("khong tao duoc contour khep kin")
    failed_markers = []
    for unused_id, marker, unused_attributes in marker_records:
        midpoint = marker.PointAt(
            (marker.Domain.T0 + marker.Domain.T1) * 0.5)
        if not any(_closest(curve, midpoint, _node_tol()) is not None
                   for curve in closed):
            failed_markers.append(marker)
    if failed_markers:
        _write("JOIN LOI: canh HCN ngoai contour kin={0}; "
               "se ve ung vien de kiem tra".format(len(failed_markers)))
        raise ChanError(
            "co {0} canh HCN khong nam tren contour khep kin".format(
                len(failed_markers)))
    _write("JOIN KHONG BOOLEAN: input={0}; output={1}; contour kin={2}".format(
        len(curves), len(joined), len(closed)))
    return joined, all_records


def _draw_candidate_geometry(source_records, marker_records):
    source_attributes = Rhino.DocObjects.ObjectAttributes()
    source_attributes.LayerIndex = _layer(
        DEBUG_ROOT + "::UNG_VIEN_SOURCE", System.Drawing.Color.Magenta)
    marker_attributes = Rhino.DocObjects.ObjectAttributes()
    marker_attributes.LayerIndex = _layer(
        DEBUG_ROOT + "::UNG_VIEN_HCN", System.Drawing.Color.Red)
    for unused_id, curve, unused_attributes in source_records:
        sc.doc.Objects.AddCurve(curve.DuplicateCurve(), source_attributes)
    for unused_id, curve, unused_attributes in marker_records:
        sc.doc.Objects.AddCurve(curve.DuplicateCurve(), marker_attributes)
    sc.doc.Views.Redraw()


def _is_bend(obj, curve):
    if obj is None or curve is None or curve.IsClosed:
        return False
    layer = sc.doc.Layers[obj.Attributes.LayerIndex]
    if layer is None:
        return False
    return layer.Name.upper() in BEND_LAYER_LEAVES


def _get_selection():
    getter = Rhino.Input.Custom.GetObject()
    getter.SetCommandPrompt(
        "Quet chon curve goc va duong chan DANHMA/DUONG_CHAN/SOI; bam chuot trai hoac Enter")
    getter.GeometryFilter = Rhino.DocObjects.ObjectType.Curve
    getter.SubObjectSelect = False
    getter.EnablePreSelect(True, True)
    getter.GetMultiple(1, 0)
    if getter.CommandResult() != Rhino.Commands.Result.Success:
        return getter.CommandResult(), [], []
    sources, bends, counts = [], [], {}
    for index in range(getter.ObjectCount):
        ref = getter.Object(index)
        curve = ref.Curve()
        obj = sc.doc.Objects.FindId(ref.ObjectId)
        if curve is None or obj is None:
            continue
        layer = sc.doc.Layers[obj.Attributes.LayerIndex]
        path = layer.FullPath if layer else "<NO_LAYER>"
        counts[path] = counts.get(path, 0) + 1
        record = (ref.ObjectId, curve.DuplicateCurve(),
                  obj.Attributes.Duplicate())
        (bends if _is_bend(obj, curve) else sources).append(record)
    _write("CHON layer={0}".format("; ".join(
        "{0}:{1}".format(key, value) for key, value in sorted(counts.items()))))
    if not sources or not bends:
        return Rhino.Commands.Result.Failure, sources, bends
    return Rhino.Commands.Result.Success, sources, bends


def _get_mode():
    getter = Rhino.Input.Custom.GetOption()
    getter.SetCommandPrompt("Che do CHAN v1.4 RunPythonScript")
    getter.AddOption("Diagnostic")
    getter.AddOption("Apply")
    getter.SetDefaultString("Diagnostic")
    result = getter.Get()
    if result == Rhino.Input.GetResult.Cancel:
        return None
    if result == Rhino.Input.GetResult.Option:
        return "Apply" if getter.Option().EnglishName == "Apply" else "Diagnostic"
    return "Diagnostic"


def _analyze(bends, source_records):
    source_curves = []
    for unused_id, curve, unused_attributes in source_records:
        source_curves.extend(_segments(curve))
    results = []
    for bend_index, bend_record in enumerate(bends):
        for end_index in (0, 1):
            results.append(_analyze_endpoint(
                bend_record, bend_index, end_index, source_curves))
    _classify_th2(results)
    _classify_th5b(results)
    return results


def _report(results):
    for number, item in enumerate(results, 1):
        _write("Dau={0}; TH={1}; W={2:.6f}; sign={3:+.0f}; lui={4:.6f}; "
               "{5}.".format(number, item["case"], item["w"],
                              item["sign"], item["shift"],
                              "DAT" if item["valid"] else "LOI=" + item["reason"]))


def _region_curve_area(curves, tolerance):
    planar = Rhino.Geometry.Brep.CreatePlanarBreps(curves, tolerance)
    if not planar:
        return 0.0
    total = 0.0
    for brep in planar:
        properties = Rhino.Geometry.AreaMassProperties.Compute(brep)
        if properties is None:
            continue
        total += properties.Area
        properties.Dispose()
    return total


def _boolean_regions_contours(inputs, phase, preserve_all_regions=False,
                              minimum_contours=None, combine_regions=False):
    """Create and select closed Boolean regions without the external BOC file."""
    tolerances = []
    for value in (0.0005, 0.005, sc.doc.ModelAbsoluteTolerance):
        if not any(abs(value - saved) <= 1e-12 for saved in tolerances):
            tolerances.append(value)
    for tolerance in tolerances:
        started = time.time()
        try:
            regions = Rhino.Geometry.Curve.CreateBooleanRegions(
                inputs, Rhino.Geometry.Plane.WorldXY,
                bool(combine_regions), tolerance)
        except Exception as error:
            _write("{0}: tolerance={1:.4f}; LOI={2}; dt={3:.3f}s".format(
                phase, tolerance, error, time.time() - started))
            continue
        if regions is None or regions.RegionCount == 0:
            _write("{0}: tolerance={1:.4f}; region=0; dt={2:.3f}s".format(
                phase, tolerance, time.time() - started))
            continue
        candidates = []
        for region_index in range(regions.RegionCount):
            region_curves = regions.RegionCurves(region_index)
            curves = list(region_curves) if region_curves else []
            if not curves or any(not curve.IsClosed for curve in curves):
                continue
            area = _region_curve_area(curves, tolerance)
            if area > 0.0:
                candidates.append((area, curves, region_index))
        if not candidates:
            continue
        if preserve_all_regions:
            selected = []
            for unused_area, curves, unused_index in candidates:
                joined = Rhino.Geometry.Curve.JoinCurves(curves, tolerance)
                result = list(joined) if joined else curves
                if not result or any(not curve.IsClosed for curve in result):
                    selected = []
                    break
                selected.extend(result)
            if selected:
                _write("{0} DAT: tolerance={1:.4f}; regions={2}; "
                       "contour={3}; dt={4:.3f}s".format(
                           phase, tolerance, regions.RegionCount,
                           len(selected), time.time() - started))
                return selected
            continue
        if minimum_contours is not None and not combine_regions:
            target = max(1, int(minimum_contours))
            selected = []
            for unused_area, curves, unused_index in sorted(
                    candidates, key=lambda item: item[0], reverse=True):
                joined = Rhino.Geometry.Curve.JoinCurves(curves, tolerance)
                result = list(joined) if joined else curves
                if not result or any(not curve.IsClosed for curve in result):
                    continue
                selected.extend(result)
                if len(selected) >= target:
                    break
            if len(selected) >= target:
                _write("{0} DAT: tolerance={1:.4f}; regions={2}; "
                       "target={3}; contour={4}; dt={5:.3f}s".format(
                           phase, tolerance, regions.RegionCount, target,
                           len(selected), time.time() - started))
                return selected
            continue
        if minimum_contours is not None and combine_regions:
            target = max(1, int(minimum_contours))
            combined = []
            for area, curves, region_index in candidates:
                joined = Rhino.Geometry.Curve.JoinCurves(curves, tolerance)
                result = list(joined) if joined else curves
                if (result and all(curve.IsClosed for curve in result) and
                        len(result) >= target):
                    combined.append((area, result, region_index))
            if combined:
                unused_area, result, unused_index = max(
                    combined, key=lambda item: item[0])
                _write("{0} DAT: tolerance={1:.4f}; regions={2}; "
                       "contour={3}; dt={4:.3f}s".format(
                           phase, tolerance, regions.RegionCount,
                           len(result), time.time() - started))
                return result
            continue
        unused_area, curves, unused_index = max(
            candidates, key=lambda item: item[0])
        joined = Rhino.Geometry.Curve.JoinCurves(curves, tolerance)
        result = list(joined) if joined else curves
        if result and all(curve.IsClosed for curve in result):
            _write("{0} DAT: tolerance={1:.4f}; regions={2}; contour={3}; "
                   "dt={4:.3f}s".format(
                       phase, tolerance, regions.RegionCount, len(result),
                       time.time() - started))
            return result
    return []


def _boolean_final_contours(marker_records, boundary_curves,
                            preserve_all_regions=False,
                            minimum_contours=None, combine_regions=False):
    inputs = [curve.DuplicateCurve() for curve in boundary_curves]
    for record in marker_records:
        sources = record.get("boolean_curves") or [record["curve"]]
        inputs.extend(curve.DuplicateCurve() for curve in sources)
    return _boolean_regions_contours(
        inputs, "BOOLEAN_1", preserve_all_regions,
        minimum_contours, combine_regions)


def _boolean_cleanup_contours(boundary_curves, preliminary_contours,
                              preserve_all_regions=False,
                              minimum_contours=None, combine_regions=False):
    inputs = ([curve.DuplicateCurve() for curve in boundary_curves] +
              [curve.DuplicateCurve() for curve in preliminary_contours])
    return _boolean_regions_contours(
        inputs, "BOOLEAN_2_FALLBACK", preserve_all_regions,
        minimum_contours, combine_regions)


def _as_boc_marker(item):
    marker = item["marker"]
    record = {
        "curve": marker["curve"].DuplicateCurve(),
        "side_left": marker["side_left"].DuplicateCurve(),
        "cap_far": marker["cap_far"].DuplicateCurve(),
        "side_right": marker["side_right"].DuplicateCurve(),
        "cap_near": marker["cap_near"].DuplicateCurve(),
        "p0": _copy_point(item["p0"]),
        "p1": _copy_point(item["p1"]),
        "p2": _copy_point(item["p2"]),
        "index": item["bend_index"] * 2 + item["end_index"],
        "detected_case": item["case"],
        "case": 3 if item["case"] in ("TH3A", "TH3B", "TH4") else (
            2 if item["case"] == "TH2" else 1),
    }
    if record["case"] == 3:
        record["intersecting_cap"] = item["cap_name"]
    return record


def _curve_fully_on_contours(curve, contours, tolerance):
    sample_count = 21
    for index in range(sample_count):
        fraction = index / float(sample_count - 1)
        point = curve.PointAt(curve.Domain.ParameterAt(fraction))
        if not _point_covered(point, contours, tolerance):
            return False
    return True


def _rail_length_on_contours(rail, from_start, contours, tolerance):
    """Do phan lien tuc cua canh dai, bat dau tu goc noi voi cap."""
    sample_count = 101
    last_fraction = 0.0
    for index in range(sample_count):
        fraction = index / float(sample_count - 1)
        if not from_start:
            fraction = 1.0 - fraction
        point = rail.PointAt(rail.Domain.ParameterAt(fraction))
        if not _point_covered(point, contours, tolerance):
            break
        last_fraction = index / float(sample_count - 1)
    return rail.GetLength() * last_fraction


def _detect_u_mark(item, contours):
    """Tim dau U a -> cap 0.2 -> b da dinh vao contour Boolean cuoi."""
    marker = item.get("marker")
    if marker is None:
        return False, None, 0.0, 0.0
    tolerance = max(_node_tol(), 0.002)
    if item["case"] in ("TH3B", "TH4") and item.get("cap_name"):
        cap_names = ["cap_far" if item["cap_name"] == "cap_near"
                     else "cap_near"]
    else:
        cap_names = ["cap_near", "cap_far"]
    for cap_name in cap_names:
        cap = marker[cap_name]
        if abs(cap.GetLength() - MARK_WIDTH) > tolerance:
            continue
        if not _curve_fully_on_contours(cap, contours, tolerance):
            continue
        if cap_name == "cap_near":
            length_a = _rail_length_on_contours(
                marker["side_left"], True, contours, tolerance)
            length_b = _rail_length_on_contours(
                marker["side_right"], False, contours, tolerance)
        else:
            length_a = _rail_length_on_contours(
                marker["side_left"], False, contours, tolerance)
            length_b = _rail_length_on_contours(
                marker["side_right"], True, contours, tolerance)
        if (length_a > _geo_tol() and length_b > _geo_tol() and
                length_a <= MAX_U_RAIL + tolerance and
                length_b <= MAX_U_RAIL + tolerance):
            return True, cap_name, length_a, length_b
    return False, None, 0.0, 0.0


def _apply(results, sources, bends, minimum_final_contours=None,
           preserve_all_regions=False, combine_regions=False):
    apply_started = time.time()
    blocking = [item for item in results
                if not item["valid"] and item["case"] != "KXD"]
    if blocking:
        _write("CO {0} endpoint TH3B/TH4 chua DAT; bo qua endpoint do, "
               "giu DUONG_CHAN va van xu ly cac dau DAT.".format(
                   len(blocking)))
    valid_results = [item for item in results if item["valid"]]
    if not valid_results:
        _write("KXD: khong co endpoint DAT de Boolean; giu nguyen input.")
        return 0, 0
    markers = [_as_boc_marker(item) for item in valid_results]
    working_sources = list(sources)
    for item in valid_results:
        extensions = item.get("th1b_extensions")
        if item["case"] != "TH1B" or not extensions:
            continue
        lengths = []
        for extension in extensions:
            attributes = None
            for unused_id, curve, source_attributes in working_sources:
                if _closest(curve, extension["start"], _node_tol()) is not None:
                    attributes = source_attributes.Duplicate()
                    break
            if attributes is None:
                raise ChanError("TH1B khong tim thay source de noi tam")
            extension_curve = Rhino.Geometry.LineCurve(
                extension["start"], extension["end"])
            if extension_curve.GetLength() <= _geo_tol():
                continue
            working_sources.append((System.Guid.NewGuid(), extension_curve,
                                    attributes))
            lengths.append(extension_curve.GetLength())
        if not lengths:
            raise ChanError("TH1B khong tao duoc doan noi tam")
        _write("B1 TH1B: them noi tam={0}; HCN lui={1:.6f} mm; "
               "doan={2}".format(
                   len(lengths), item["shift"],
                   ",".join("{0:.6f}".format(length) for length in lengths)))
    for item in valid_results:
        if (item["case"] not in ("TH3B", "TH4") and
                not item.get("th1b_trim_w")):
            continue
        affected, replacement = _trim_w_from_sources(item, working_sources)
        affected_ids = set(record[0] for record in affected)
        working_sources = [record for record in working_sources
                           if record[0] not in affected_ids]
        working_sources.extend((System.Guid.NewGuid(), curve, attributes)
                               for curve, attributes in replacement)
        _write("B1 {0}: trim phan bien thua P0-P1/P2; affected={1}; "
               "replacement={2}.".format(
                   item["case"], len(affected), len(replacement)))
    boundaries = [curve.DuplicateCurve()
                  for unused_id, curve, unused_attributes in working_sources]
    for item, marker in zip(valid_results, markers):
        if (item["case"] in ("TH3A", "TH3B", "TH4") or
                item.get("th1b_trim_w")):
            marker["boolean_curves"] = _trim_w_from_marker(item)
            _write("B1 {0}: bo W={1:.6f} tren cap 0.2; U={2} doan.".format(
                item["case"], item["w"], len(marker["boolean_curves"])))
    _write("DUONG_CHAN: da loai khoi input Boolean ngay sau khi tao HCN; "
           "object Rhino chi xoa sau khi contour cuoi DAT.")
    _write("HYBRID: source={0}; marker={1}; TH1={2}; TH1B={3}; TH2={4}; "
           "TH3A={5}; TH3B={6}; TH4={7}; TH5A={8}; TH5B={9}".format(
               len(boundaries), len(markers),
               sum(1 for item in valid_results if item["case"] == "TH1"),
               sum(1 for item in valid_results if item["case"] == "TH1B"),
               sum(1 for item in valid_results if item["case"] == "TH2"),
               sum(1 for item in valid_results if item["case"] == "TH3A"),
               sum(1 for item in valid_results if item["case"] == "TH3B"),
               sum(1 for item in valid_results if item["case"] == "TH4"),
               sum(1 for item in valid_results if item["case"] == "TH5A"),
               sum(1 for item in valid_results if item["case"] == "TH5B")))
    _write("TIMING | stage=prepare_boolean | dt={0:.3f}s".format(
        time.time() - apply_started))
    try:
        boolean_boundaries = boundaries
        _write("B2 BooleanB All: boundary={0}; marker={1}; "
               "khong dung dieu kien split HCN=4.".format(
                   len(boolean_boundaries), len(markers)))
        boolean_started = time.time()
        preliminary = _boolean_final_contours(
            markers, boolean_boundaries, preserve_all_regions,
            minimum_final_contours, combine_regions)
        if not preliminary:
            raise ChanError("BooleanB lan 1 khong tao duoc contour")
        first_ready = (
            all(curve.IsClosed for curve in preliminary) and
            (minimum_final_contours is None or
             len(preliminary) >= int(minimum_final_contours)))
        if first_ready:
            final_curves = preliminary
            _write("BOOLEAN_2 BO_QUA: Boolean lan 1 da du contour kin.")
        else:
            final_curves = _boolean_cleanup_contours(
                boolean_boundaries, preliminary, preserve_all_regions,
                minimum_final_contours, combine_regions)
            if not final_curves:
                raise ChanError("BooleanB cleanup fallback khong tao duoc contour")
        _write("TIMING | stage=boolean_total | dt={0:.3f}s".format(
            time.time() - boolean_started))
        if any(not curve.IsClosed for curve in final_curves):
            raise ChanError("Boolean tra ve contour ho")
        if (minimum_final_contours is not None and
                len(final_curves) < int(minimum_final_contours)):
            raise ChanError(
                "BooleanB lam sut contour: final={0}; toi_thieu={1}; "
                "giu nguyen C1 de kiem tra".format(
                    len(final_curves), int(minimum_final_contours)))

        signs_started = time.time()
        detected_signs = set()
        for item in valid_results:
            if item["case"] in ("TH3B", "TH4"):
                detected_signs.add((item["bend_index"], item["end_index"]))
                _write("DAU_CHAN MAC_DINH bend={0} endpoint={1}; {2}; "
                       "bo qua dem dau U tren contour cuoi.".format(
                           item["bend_index"] + 1, item["end_index"] + 1,
                           item["case"]))
                continue
            found, cap_name, length_a, length_b = _detect_u_mark(
                item, final_curves)
            if found:
                detected_signs.add((item["bend_index"], item["end_index"]))
                _write("DAU_CHAN DAT bend={0} endpoint={1}; {2}; "
                       "a={3:.6f}; b={4:.6f}.".format(
                           item["bend_index"] + 1, item["end_index"] + 1,
                           cap_name, length_a, length_b))
            else:
                _write("DAU_CHAN KXD bend={0} endpoint={1}; khong tim thay "
                       "a->cap0.2->b tren contour cuoi.".format(
                           item["bend_index"] + 1,
                           item["end_index"] + 1))
        completed_bend_indexes = set(
            bend_index for bend_index in range(len(bends))
            if ((bend_index, 0) in detected_signs and
                (bend_index, 1) in detected_signs))
        _write("TIMING | stage=verify_signs | dt={0:.3f}s".format(
            time.time() - signs_started))
    except Exception:
        _draw_debug(results)
        _write("HYBRID LOI: input giu nguyen; da ve marker chan doan")
        raise

    commit_started = time.time()
    undo = sc.doc.BeginUndoRecord(COMMAND_NAME)
    created = []
    try:
        attributes = Rhino.DocObjects.ObjectAttributes()
        attributes.LayerIndex = _layer(
            "BOC::DUONG_CAT", System.Drawing.Color.Cyan)
        attributes.Name = "BOC_DUONG_CAT"
        for curve in final_curves:
            object_id = sc.doc.Objects.AddCurve(curve, attributes)
            if object_id == System.Guid.Empty:
                raise ChanError("khong them duoc contour Boolean cuoi")
            created.append(object_id)
        replaced_sources = []
        # Equivalent to CurveBoolean DeleteInput=All, restricted to the
        # source curves explicitly selected for this CHAN1 run.
        for object_id, unused_curve, unused_attributes in sources:
            if not sc.doc.Objects.Delete(object_id, True):
                raise ChanError("khong xoa duoc source cu")
            replaced_sources.append(object_id)
        bends_to_delete = [record for index, record in enumerate(bends)
                           if index in completed_bend_indexes]
        for object_id, unused_curve, unused_attributes in bends_to_delete:
            if not sc.doc.Objects.Delete(object_id, True):
                raise ChanError("khong xoa duoc DUONG_CHAN cu")
        _write("HYBRID DAT: Boolean1={0}; final kin={1}; "
               "DeleteInput=All source={2}; source ngoai giu={3}; "
               "xoa DUONG_CHAN={4}".format(
                   len(preliminary), len(final_curves), len(replaced_sources),
                   0, len(bends_to_delete)))
        remaining = len(bends) - len(bends_to_delete)
        if remaining:
            _write("KXD/THIEU DAU: giu lai {0} DUONG_CHAN chua du hai dau U."
                   .format(remaining))
        _write("TIMING | stage=commit_document | dt={0:.3f}s".format(
            time.time() - commit_started))
        return len(bends_to_delete), len(detected_signs)
    except Exception:
        for object_id in created:
            sc.doc.Objects.Delete(object_id, True)
        raise
    finally:
        if undo >= 0:
            sc.doc.EndUndoRecord(undo)


def analyze_chan(bend_records, source_records):
    """Analyze CHAN endpoints without changing Rhino document geometry.

    Records use the internal tuple contract ``(object_id, curve, attributes)``.
    This is the supported integration point for another BOC task that needs
    CHAN classification or preview before committing document changes.
    """
    return _analyze(bend_records, source_records)


def draw_chan_debug(results):
    """Ve marker chan doan cho cac endpoint chua DAT, khong sua input."""
    _draw_debug(results)


def apply_chan(results, source_records, bend_records,
               minimum_final_contours=None, preserve_all_regions=False,
               combine_regions=False):
    """Apply a previously validated CHAN analysis in one Rhino undo record."""
    return _apply(results, source_records, bend_records,
                  minimum_final_contours, preserve_all_regions,
                  combine_regions)


def run_chan(source_records, bend_records):
    """Run the reusable CHAN engine on caller-supplied Rhino object records."""
    analyze_started = time.time()
    results = analyze_chan(bend_records, source_records)
    _write("TIMING | stage=analyze_endpoints | dt={0:.3f}s".format(
        time.time() - analyze_started))
    _report(results)
    invalid = [item for item in results if not item["valid"]]
    kxd_bend_ids = set(item["bend_id"] for item in invalid
                       if item["case"] == "KXD")
    blocking = [item for item in invalid if item["case"] != "KXD"]
    diagnostics = list(blocking)
    if kxd_bend_ids:
        # Show both 2x0.2 HCNs of every affected DUONG_CHAN, not only the
        # endpoint classified KXD, so the user can resolve it manually.
        diagnostics.extend(item for item in results
                           if item["bend_id"] in kxd_bend_ids)
    if diagnostics:
        _draw_debug(diagnostics)
    if kxd_bend_ids:
        _write("KXD: da ve HCN/ghi chu cho {0} DUONG_CHAN; van Boolean "
               "cac endpoint DAT khac.".format(len(kxd_bend_ids)))
    if blocking:
        _write("CHAN DOAN: con {0} endpoint TH3B/TH4 chua DAT; "
               "endpoint do khong vao Boolean.".format(len(blocking)))
    # Ca 2 hinh + 1 DUONG_CHAN tuong duong BooleanB click ca hai vung.
    # v1.4 de minimum_final_contours=None nen chi lay vung lon nhat.
    minimum_final_contours = (
        2 if len(source_records) == 2 and len(bend_records) == 1 else None)
    if minimum_final_contours == 2:
        _write("BOOLEAN_2_HINH: yeu cau giu toi thieu 2 contour, "
               "tuong duong click ca hai vung trong BooleanB.")
    completed_bends, completed_ends = apply_chan(
        results, source_records, bend_records,
        minimum_final_contours=minimum_final_contours)
    return completed_bends, completed_ends, invalid


def run_chan_script():
    sc.doc = Rhino.RhinoDoc.ActiveDoc
    _set_step("Kiem tra document")
    _write("=== BAT DAU {0} v{1}: base v1.4; BooleanB giu ca 2 hinh ===".format(
        COMMAND_NAME, VERSION))
    if sc.doc.ModelUnitSystem != Rhino.UnitSystem.Millimeters:
        _write("LOI: document phai dung millimeters")
        return Rhino.Commands.Result.Failure
    _clear_old_debug()
    _set_step("Chon input")
    selection_result, sources, bends = _get_selection()
    if selection_result != Rhino.Commands.Result.Success:
        _write("LOI: can chon curve goc va duong chan tren "
               "DANHMA/DUONG_CHAN/SOI")
        return selection_result
    _write("NHAN: curve goc={0}; duong chan={1}".format(
        len(sources), len(bends)))
    try:
        _set_step("Phan tich endpoint va BooleanB")
        engine_started = time.time()
        completed_bends, completed_ends, invalid = run_chan(sources, bends)
        _write("TIMING | stage=engine_total | dt={0:.3f}s".format(
            time.time() - engine_started))
        sc.doc.Views.Redraw()
        remaining_bends = len(bends) - completed_bends
        _set_step("Hoan thanh")
        _write("HOAN THANH: xoa DUONG_CHAN={0}; dau U={1}; "
               "giu DUONG_CHAN={2}; endpoint loi truoc Boolean={3}".format(
                   completed_bends, completed_ends, remaining_bends,
                   len(invalid)))
        return (Rhino.Commands.Result.Success if completed_bends else
                Rhino.Commands.Result.Failure)
    except Exception as error:
        _set_step("Loi")
        _write("LOI: " + str(error))
        _write(traceback.format_exc())
        sc.doc.Views.Redraw()
        return Rhino.Commands.Result.Failure


def run_safe():
    started = time.time()
    status = "Failure"
    try:
        result = run_chan_script()
        if result == Rhino.Commands.Result.Success:
            status = "Success"
        elif result == Rhino.Commands.Result.Cancel:
            status = "Cancel"
        return result
    except Exception as error:
        _set_step("Loi ngoai")
        _write("LOI NGOAI: " + str(error))
        _write(traceback.format_exc())
        return Rhino.Commands.Result.Failure
    finally:
        _write("RUNTIME | runtime={0:.3f}s | final_step={1} | status={2}"
               .format(time.time() - started, FINAL_STEP, status))


if __name__ == "__main__":
    run_safe()

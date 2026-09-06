
# -*- coding: utf-8 -*-


STICKY_KEY = "SMART_PROCESS_SETTINGS_V_FINAL_PRO"

def intedgrated_smart_process(ids=None):
    """Process caller-supplied surfaces, or ask the user when omitted.

    Returns the ids created by SC so a parent command can continue safely.
    """
    # 1. Chọn đối tượng
    if ids is None:
        ids = rs.GetObjects("Chọn đối tượng để xử lý né chồng lấn", filter=8+16, preselect=True)
    ids = list(ids) if ids else []
    created = {"borders": [], "inner_curves": [], "marks": []}
    if not ids:
        return created

    # Chạy ngay bằng thiết lập đã lưu, không hiển thị câu hỏi phụ.
    saved_data = sc.sticky.get(STICKY_KEY, [False, "C", "", 12.0])
    enable_mark, prefix, suffix, text_h_max = saved_data

    # --- BẮT ĐẦU XỬ LÝ (KHÔNG CẦN CHỈNH SỬA BÊN DƯỚI) ---
    rs.EnableRedraw(False)
    tol = rs.UnitAbsoluteTolerance()
    SAFE_GAP = 5.0 

    for obj_id in ids:
        parent_layer = rs.ObjectLayer(obj_id)
        raw_name = rs.ObjectName(obj_id) or ""
        full_name = "{}{}{}".format(prefix, raw_name, suffix)
        
        l_cut = rs.AddLayer("{}::CẮT".format(parent_layer), Color.FromArgb(255, 127, 0))
        l_soi = rs.AddLayer("{}::SOI".format(parent_layer), Color.FromArgb(0, 255, 0))
        l_mark = rs.AddLayer("{}::DANH MA".format(parent_layer), Color.FromArgb(255, 0, 255))

        # 1. PHÂN TÍCH HÌNH HỌC
        raw_border = rs.DuplicateSurfaceBorder(obj_id)
        if not raw_border: continue
        borders = rs.JoinCurves(raw_border, True) if len(raw_border) > 1 else raw_border
        border_list = borders if isinstance(borders, list) else [borders]
        
        for b in border_list:
            rs.ObjectLayer(b, l_cut)
            created["borders"].append(b)

        inner_curves = []
        edges = rs.DuplicateEdgeCurves(obj_id)
        if edges:
            for e in edges:
                mid = rs.CurveMidPoint(e)
                is_on = any(rs.Distance(mid, rs.EvaluateCurve(b, rs.CurveClosestPoint(b, mid))) < tol*2.5 for b in border_list)
                if is_on: rs.DeleteObject(e)
                else:
                    rs.ObjectLayer(e, l_soi); inner_curves.append(e)
                    created["inner_curves"].append(e)

        # 2. THUẬT TOÁN SĂN VÙNG TRỐNG
        if enable_mark and full_name.strip():
            plane = rs.CurvePlane(border_list[0])
            if not plane: plane = rs.WorldXYPlane()
            
            res_centroid = rs.SurfaceAreaCentroid(obj_id)
            centroid = res_centroid[0] if res_centroid else rs.CurveAreaCentroid(border_list[0])[0]
            plane.Origin = centroid
            
            bbox = rs.BoundingBox(obj_id, plane)
            lx = rs.Distance(bbox[0], bbox[1])
            ly = rs.Distance(bbox[0], bbox[3])
            
            if ly > lx:
                plane.Rotate(Rhino.RhinoMath.ToRadians(90), plane.ZAxis, centroid)
                lx, ly = ly, lx

            obs_geoms = [rs.coercecurve(o) for o in (border_list + inner_curves)]
            best_pt = centroid
            max_clearance = 0
            
            for i in range(1, 20):
                for j in range(1, 10):
                    test_pt = plane.PointAt(lx * (i/20.0 - 0.5), ly * (j/10.0 - 0.5))
                    if rs.PointInPlanarClosedCurve(test_pt, border_list[0], plane) == 1:
                        d_min = 1000000
                        for obs in obs_geoms:
                            if not obs: continue
                            cp = rs.CurveClosestPoint(obs, test_pt)
                            dist = rs.Distance(test_pt, rs.EvaluateCurve(obs, cp))
                            if dist < d_min: d_min = dist
                        if d_min > max_clearance:
                            max_clearance = d_min
                            best_pt = test_pt

            # 3. ĐẶT TEXT
            plane.Origin = best_pt
            curr_h = text_h_max
            final_txt = None

            for _ in range(15):
                if curr_h < 1.0: break
                te = Rhino.Geometry.TextEntity()
                te.Plane, te.PlainText, te.TextHeight = plane, full_name, curr_h
                te.Justification = Rhino.Geometry.TextJustification.MiddleCenter
                
                txt_bbox = te.GetBoundingBox(plane)
                current_gap = min(SAFE_GAP, max_clearance * 0.4)
                txt_bbox.Inflate(current_gap)
                
                corners = txt_bbox.GetCorners()
                buffer_crv = Rhino.Geometry.Polyline([corners[0], corners[1], corners[2], corners[3], corners[0]]).ToNurbsCurve()
                
                collision = False
                for obs in obs_geoms:
                    if obs and Rhino.Geometry.Curve.PlanarCurveCollision(buffer_crv, obs, plane, tol):
                        collision = True; break
                
                if not collision:
                    final_txt = sc.doc.Objects.AddText(te)
                    break
                else: curr_h *= 0.8

            if final_txt:
                rs.ObjectLayer(final_txt, l_mark); rs.ObjectColorSource(final_txt, 0)
                created["marks"].append(final_txt)

    rs.DeleteObjects(ids)
    rs.EnableRedraw(True)
    sc.doc.Views.Redraw()
    print("Xử lý hoàn tất!")
    return created

if __name__ == "__main__":
    integrated_smart_process()

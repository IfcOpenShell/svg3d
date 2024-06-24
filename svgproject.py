import os
import sys
import json
import numpy
import operator

import ifcopenshell, ifcopenshell.geom

import OCC.Core.gp, OCC.Core.Bnd, OCC.Core.BRepBuilderAPI, OCC.Core.BRepExtrema, OCC.Core.TopoDS, OCC.Core.TopExp, OCC.Core.TopAbs, OCC.Core.BRepMesh, OCC.Core.BRep, OCC.Core.TopLoc

ON_EDGE = OCC.Core.BRepExtrema.BRepExtrema_IsOnEdge

from xml.dom.minidom import parse

ifcfn, fn, fn2, fn3, fn4 = sys.argv[1:]

SCALE = 50.
NO_COLOURS = True

dom1 = parse(fn)
dom2 = parse(fn2)

svg1 = dom1.childNodes[0]
svg2 = dom2.childNodes[0]

# @todo the OCCT BVH tree we used doesn't do ray intersection, so we compute AABB from the box (which is only a good solution for AA rays). Upgrade to the new QBVH tree in OCCT?
ifc_file = ifcopenshell.open(ifcfn)
tree_settings = ifcopenshell.geom.settings()
tree = ifcopenshell.geom.tree(ifc_file, tree_settings)

# @todo we cannot get model bounds from the tree in Python, use something arbitrarily large without running into weird tolerance issues

# @todo add a prototype to tree that stores individual faces?

# @todo do we really need to create shapes twice?
shape_settings = ifcopenshell.geom.settings(USE_PYTHON_OPENCASCADE=True, APPLY_LAYERSETS=True)

shape_dict = {}
face_style_dict = {}
face_instance_dict = {}

def loop_to_wire(coords):
    builder = OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakePolygon()
    for p in coords:
        builder.Add(OCC.Core.gp.gp_Pnt(*p))
    builder.Close()
    return builder.Wire()
    
def loops_to_face(inner, outer):
    mf = OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeFace(loop_to_wire(inner))
    for o in outer:
        mf.Add(loop_to_wire(o))
    return mf.Face()

def mesh(shape, deflection=0.01):
    OCC.Core.BRepMesh.BRepMesh_IncrementalMesh(shape, deflection)
    bt = OCC.Core.BRep.BRep_Tool()

    exp = OCC.Core.TopExp.TopExp_Explorer(shape, OCC.Core.TopAbs.TopAbs_FACE)
    while exp.More():
        face = OCC.Core.TopoDS.topods.Face(exp.Current())
        loc = OCC.Core.TopLoc.TopLoc_Location()
        triangulation = bt.Triangulation(face, loc)
        if triangulation is not None:
            trsf = loc.Transformation()
            vs = ["1-based"] + [triangulation.Node(i + 1).Transformed(trsf) for i in range(triangulation.NbNodes())]

            tris = triangulation.Triangles()
            for i in range(triangulation.NbTriangles()):
                tri = tris.Value(i + 1)
                pnts = tuple(map(vs.__getitem__, tri.Get()))
                if face.Orientation() == OCC.Core.TopAbs.TopAbs_REVERSED:
                    pnts = pnts[::-1]
                yield pnts

        exp.Next()
    

class obj_model:
    def __init__(self, fn):
        self.F = open(fn, "w")
        self.G = open(fn[:-4] + ".mtl", "w")
        self.vs = []
        self.fs = []
        self.mtls = []
        self.style_name_dict = {}
        self.N = 1
        print("mtlib", os.path.basename(fn3)[:-4] + ".mtl", file=self.G)
        
    def write(self, id, clr, vs, idxs):
        if not idxs:
            return
        
        m = self.style_name_dict.get(clr)
        if m is None:
            n = len(self.style_name_dict)
            print("newmtl", "s%03d" % n, file=self.G)
            print("Kd", *clr, file=self.G)
            m = self.style_name_dict[clr] = "s%03d" % n
            
        print("g", id, file=self.F)
        print("usemtl", m, file=self.F)
        
        assert min(min(i) for i in idxs) >= 0
        assert max(max(i) for i in idxs) < len(vs)
        
        for v in vs:
            print("v", *v, file=self.F)
        for f in idxs:
            print("f", *(i + self.N for i in f), file=self.F)
            
        self.N += len(vs)
        
obj = obj_model(fn3)

hatches = set()

def create_pattern(name, h_or_v_spacing_pairs, rgb=None):
    
    spacing = {'h': None, 'v': None}
    
    for h_or_v, sp in h_or_v_spacing_pairs:
        if spacing[h_or_v] is None or sp > spacing[h_or_v]:
            spacing[h_or_v] = sp
            
    assert spacing['h'] is not None or spacing['v'] is not None
    
    if spacing['h'] is None: spacing['h'] = spacing['v']
    if spacing['v'] is None: spacing['v'] = spacing['h']
    
    spacing['h'] = str(spacing['h'] / SCALE)
    spacing['v'] = str(spacing['v'] / SCALE)
    
    defs = svg1.getElementsByTagName('defs')[0]
    pat = dom1.createElement('pattern')
    pat.setAttribute('id', name)
    # pat.setAttribute('viewBox', ",".join(map(str, (0,0,spacing,spacing))))
    pat.setAttribute('width', spacing['h'])
    pat.setAttribute('height', spacing['v'])
    pat.setAttribute('patternTransform', 'translate(0,0) scale(1,1)')
    pat.setAttribute('patternUnits', 'userSpaceOnUse')
    
    for h_or_v, sp in h_or_v_spacing_pairs:
    
        x2 = spacing['h'] if h_or_v == 'h' else '0'
        y2 = spacing['v'] if h_or_v == 'h' else '0'

        line = dom1.createElement('line')
        
        line.setAttribute('x1', '0')
        line.setAttribute('y1', spacing['v'])
        line.setAttribute('x2', x2)
        line.setAttribute('y2', y2)
        line.setAttribute('stroke-width', "0.25")
        
        clr = 'black'
        if rgb:
            clr = "%s(%s)" % ("rgba"[0:len(rgb)], ", ".join(str(f * 255.) for f in rgb))
            
        line.setAttribute('style', 'stroke: %s' % clr)
        
        pat.appendChild(line)

    defs.appendChild(pat)
    
to_exclude = sum(map(ifc_file.by_type, ("IfcOpeningElement", "IfcSpace")), [])
for e in ifcopenshell.geom.iterate(shape_settings, ifc_file, exclude=to_exclude):
    shape_dict[e.data.guid] = e.geometry
    it = OCC.Core.TopoDS.TopoDS_Iterator(e.geometry)
    subshapes = []
    while it.More():
        subshapes.append(it.Value())
        it.Next()
    assert len(subshapes) == len(e.styles)
    
    for ss, st, sid in zip(subshapes, e.styles, e.style_ids):
        
        ref = None

        if sid != -1:                
            style = list(ifc_file.get_inverse(ifc_file[sid]))[0].Styles
            fills = [i for i in style if i.is_a("IfcFillAreaStyle")]
            if fills:
                nm = 'hatch_%d' % fills[0].id()
                ref = 'url(#%s)' % nm
                
                if fills[0] not in hatches:
                    hatches.add(fills[0])
                    sdi = {s.is_a(): s for s in fills[0].FillStyles}
                    clr = sdi.get('IfcColourRgb')
                    rgb = None
                    if clr:
                        rgb = clr.Red, clr.Green, clr.Blue
                    _, *h_or_v_and_spacing = fills[0].Name.split("_")
                    h_or_v_spacing_pairs = [(h_or_v_and_spacing[i].lower(), float(h_or_v_and_spacing[i+1])) for i in range(0, len(h_or_v_and_spacing), 2)]
                    create_pattern(nm, h_or_v_spacing_pairs, rgb)
    
        exp = OCC.Core.TopExp.TopExp_Explorer(ss, OCC.Core.TopAbs.TopAbs_FACE)
        while exp.More():
            face_style_dict[exp.Current()] = st, ref
            face_instance_dict[exp.Current()] = ifc_file[e.data.guid]
            exp.Next()
            
def yield_groups(n):
    if n.nodeType == n.ELEMENT_NODE and n.tagName == "g":
        yield n
    for c in n.childNodes:
        yield from yield_groups(c)
        
def parse_path(d):
    vs = []
    idxs = []
    add_edge = lambda a, b: idxs.append((a, b))
    for i, s in enumerate(d.split(" ")):
        code = s[0]
        if i == 0: assert code == "M"
        if code != "Z":
            vs.append(list(map(float, s[1:].split(","))))
        if code == "L":
            add_edge(i-1, i)
        elif code.upper() == "Z":
            add_edge(i-1, 0)
    return vs, idxs

groups1 = [g for g in yield_groups(svg1) if g.getAttribute("class") == "projection"]
groups2 = list(yield_groups(svg2))

assert len(groups1) == len(groups2)

for ii, (g, gg) in enumerate(zip(groups1, groups2)):

    projection = g
    g = g.parentNode
    
    nm = g.getAttribute("ifc:name")
    m4 = numpy.array(json.loads(g.getAttribute("ifc:plane")))
    m3 = numpy.array(json.loads(g.getAttribute("ifc:matrix3")))
    m44 = numpy.eye(4)
    m44[0][0:2] = m3[0][0:2]
    m44[1][0:2] = m3[1][0:2]
    m44[0][3] = m3[0][2]
    m44[1][3] = m3[1][2]
    m44 = numpy.linalg.inv(m44)
    
    def project(xy, z=0.):
        xyzw = m44 @ numpy.array(xy + [z, 1.])
        xyzw[1] *= -1.
        return tuple(map(float, (m4 @ xyzw)[0:3]))

    for i, p in enumerate(gg.getElementsByTagName("path")):
        
        d = p.getAttribute("d")
        if "A" in d: continue
        
        assert p.hasAttribute("ifc:pointInside")
        
        xy = list(map(float, p.getAttribute("ifc:pointInside").split(",")))
        v_ray = [project(xy, 0.), project(xy, -100.)]
        i_ray = [(0, 1)]
        
        pnts = [OCC.Core.gp.gp_Pnt(*pn[0:3]) for pn in v_ray]
        E = OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeEdge(*pnts).Edge()
        
        BB = OCC.Core.Bnd.Bnd_Box()
        for pn in pnts:
            BB.Add(pn)
        coords = BB.Get()
        coords = coords[:3], coords[3:]
        candidates = tree.select_box(coords)
        
        min_u = 1e9
        closest = None
        closest_face = None
        closest_point = None
        
        p.style = "fill: white"
        
        for c in candidates:
            if c.is_a("IfcSpace") or c.is_a("IfcOpeningElement"):
                continue
            shp = shape_dict[c.GlobalId]
            dss = OCC.Core.BRepExtrema.BRepExtrema_DistShapeShape(E, shp)
            for iii in range(1, dss.NbSolution() + 1):
                if dss.SupportTypeShape1(iii) != ON_EDGE:
                    print("Not on edge")
                    #@todo pick 0?
                    continue
                
                u = dss.ParOnEdgeS1(iii)
                ff = dss.SupportOnShape2(iii)
                
                if u < min_u and ff.ShapeType() == OCC.Core.TopAbs.TopAbs_FACE:
                    min_u = u
                    closest = c
                    closest_face = OCC.Core.TopoDS.topods.Face(ff)
                    closest_point = dss.PointOnShape1(iii)
                    
        if closest_face is None:
            print("No face recovered")
            clr = (-1., -1., -1., 1.)
            id = "".join(c for c in nm if c.isalnum())
            hatch_ref = None
        else:    
            clr, hatch_ref = face_style_dict[closest_face]
            id = "".join(c for c in nm+closest.Name if c.isalnum())
            
        inst = face_instance_dict.get(closest_face)
        if inst:
            p.setAttribute('class', inst.is_a())            
            
        clr_obj = clr
        svg_fill = "rgb(%s)" % ", ".join(str(f * 255.) for f in clr[0:3])
        
        if clr[0] == -1.:
            svg_fill = "none"
            clr_obj = 0.6, 0.6, 0.6, 1.0
            
        if hatch_ref:
            svg_fill = hatch_ref
        elif NO_COLOURS:
            svg_fill = "none"
                
        p.setAttribute('style', "fill: " + svg_fill)
        
        if closest:
            v_ray[1] = (closest_point.X(), closest_point.Y(), closest_point.Z())
            obj.write(id + "ray", clr_obj, v_ray, i_ray)
        
        edge_loop_to_face = lambda edge_loop: list(map(operator.itemgetter(0), edge_loop))
                
        segments = ["M"+s.strip() for s in d.split("M")[1:]]
        vs_is = [(list(map(project, vs)), ids) for vs, ids in map(parse_path, segments)]
        
        obj.write(id + "outer", clr_obj, *vs_is[0])
        
        if len(vs_is) == 1:        
            i_face = edge_loop_to_face(vs_is[0][1])
            obj.write(id + "face", clr_obj, vs_is[0][0], [i_face])
        else:
            all_vs = []
            all_idxs = []
            for vs, idxs in vs_is:
                all_idxs.append(numpy.array(edge_loop_to_face(idxs)) + len(all_vs))
                all_vs += vs
            all_vs = numpy.array(all_vs)
            coords = sum(mesh(loops_to_face(all_vs[all_idxs[0]], [all_vs[ii] for ii in all_idxs[1:]])), ())
            coords = [(c.X(), c.Y(), c.Z()) for c in coords]
            idxs = [(3*i+0,3*i+1,3*i+2) for i in range(len(coords) // 3)]
            obj.write(id + "face", clr_obj, coords, idxs)

    # swap the XML nodes from the files
    g.removeChild(projection)
    gg.setAttribute('class', 'projection')
    g.appendChild(gg)

data = dom1.toxml()
data = data.encode('ascii', 'xmlcharrefreplace')
open(fn4, 'wb').write(data)

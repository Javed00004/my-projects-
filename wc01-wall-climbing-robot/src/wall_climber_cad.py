"""Parametric WC-01 CAD concept: DXF drawings, STEP solids and a GLB preview.

Run in Google Colab. All model coordinates and DXF drawing units are mm.
The design is a neutral, symmetric pose based on the earlier animated concept.
"""

from dataclasses import dataclass
from pathlib import Path
import math
import zipfile

import cadquery as cq
import ezdxf
import matplotlib.pyplot as plt
import numpy as np
from ezdxf.addons.drawing import Frontend, RenderContext
from ezdxf.addons.drawing.matplotlib import MatplotlibBackend
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

try:
    from IPython.display import Image, display
except ImportError:
    Image = display = None


@dataclass(frozen=True)
class Design:
    # EDIT THESE VALUES, THEN RUN THE NOTEBOOK AGAIN.
    body_w: float = 660.0
    body_h: float = 720.0
    body_depth: float = 220.0
    rear_plate_w: float = 730.0
    rear_plate_h: float = 800.0
    rear_plate_t: float = 12.0
    rear_plate_y: float = 235.0
    hole_d: float = 12.0
    hole_pitch_x: float = 610.0
    hole_pitch_z: float = 680.0
    pad_x: float = 770.0       # x distance from center to each pad
    pad_z: float = 320.0       # z distance from center to each pad
    cup_d: float = 210.0
    flange_d: float = 252.0


P = Design()
OUT = Path("WC01_CAD_Output")
OUT.mkdir(exist_ok=True)


def validate(p):
    assert p.body_w > 0 and p.body_h > 0 and p.body_depth > 0
    assert p.rear_plate_w > p.hole_pitch_x + p.hole_d + 20
    assert p.rear_plate_h > p.hole_pitch_z + p.hole_d + 20
    assert p.pad_x > p.body_w/2 + p.flange_d/2
    assert p.pad_z > p.flange_d/2
    assert p.flange_d > p.cup_d > 0
    assert p.rear_plate_y + p.rear_plate_t < 300
    assert p.body_h > 500 and p.body_w > 400


def layout(p):
    """Derived coordinates shared by the solids and the orthographic views."""
    body_back = 300.0
    body_front = body_back + p.body_depth
    shoulder_x = p.body_w/2 - 40
    shoulder_z = p.body_h/2 - 130
    elbow_x = shoulder_x + 0.52*(p.pad_x-shoulder_x)
    return body_back, body_front, shoulder_x, shoulder_z, elbow_x


def solid_box(sx, sy, sz, x=0, y=0, z=0):
    return cq.Workplane("XY").box(sx, sy, sz).val().translate((x, y, z))


def cyl_y(radius, y0, length, x=0, z=0):
    return cq.Solid.makeCylinder(radius, length, cq.Vector(x, y0, z),
                                 cq.Vector(0, 1, 0))


def rod(start, end, radius):
    a, b = cq.Vector(*start), cq.Vector(*end)
    direction = b.sub(a)
    return cq.Solid.makeCylinder(radius, direction.Length, a,
                                 direction.normalized())


def make_parts(p):
    """One exact geometric definition for both the 3D exports and dimensions."""
    body_back, body_front, shoulder_x, shoulder_z, elbow_x = layout(p)
    plate_y_center = p.rear_plate_y + p.rear_plate_t/2
    plate = solid_box(p.rear_plate_w, p.rear_plate_t, p.rear_plate_h,
                      y=plate_y_center)
    for x in (-p.hole_pitch_x/2, p.hole_pitch_x/2):
        for z in (-p.hole_pitch_z/2, p.hole_pitch_z/2):
            hole = cyl_y(p.hole_d/2, p.rear_plate_y-1,
                         p.rear_plate_t+2, x, z)
            plate = plate.cut(hole)

    # Shape, name, RGB color. The rear plate is an individual machined part.
    parts = [(plate, "rear_plate_4x_dia12_holes", (0.34, 0.52, 0.59))]
    for x in (-260, 260):
        for z in (-280, 280):
            parts.append((cyl_y(27, p.rear_plate_y+p.rear_plate_t,
                                body_back-(p.rear_plate_y+p.rear_plate_t), x, z),
                          f"standoff_{x}_{z}", (0.64, 0.75, 0.77)))

    parts += [
        (solid_box(p.body_w, p.body_depth, p.body_h,
                   y=(body_back+body_front)/2),
         "main_enclosure", (0.10, 0.28, 0.36)),
        (solid_box(p.body_w-150, 20, p.body_h-240,
                   y=body_front+10, z=-30),
         "front_access_cover", (0.87, 0.59, 0.25)),
        (solid_box(350, 45, 80, y=body_front+42.5, z=p.body_h/2-105),
         "upper_service_module", (0.15, 0.39, 0.45)),
        (cyl_y(85, body_front+20, 60, 0, 60),
         "sensor_housing", (0.75, 0.84, 0.84)),
        (cyl_y(39, body_front+80, 15, 0, 60),
         "sensor_lens", (0.17, 0.67, 0.77)),
    ]
    for x in (-190, 190):
        parts.append((cyl_y(25, body_front+20, 20, x, p.body_h/2-110),
                      f"status_light_{x}", (0.36, 0.90, 0.64)))

    for side in (-1, 1):
        for level in (-1, 1):
            x, z = side*p.pad_x, level*p.pad_z
            label = f"{'L' if side < 0 else 'R'}_{'lower' if level < 0 else 'upper'}"
            shoulder = (side*shoulder_x, body_back+50, level*shoulder_z)
            elbow = (side*elbow_x, 235, z)
            hub = (x, 150, z)
            cup = cyl_y(p.cup_d/2, 20, 70, x, z)
            recess = cyl_y(p.cup_d/2-22, 19, 35, x, z)
            parts += [
                (cyl_y(62, body_back+30, 55, shoulder[0], shoulder[2]),
                 f"shoulder_pivot_{label}", (0.76, 0.56, 0.28)),
                (rod(shoulder, elbow, 43),
                 f"primary_arm_{label}", (0.65, 0.76, 0.77)),
                (cyl_y(60, 210, 50, elbow[0], elbow[2]),
                 f"elbow_pivot_{label}", (0.85, 0.63, 0.34)),
                (rod(elbow, hub, 34),
                 f"secondary_arm_{label}", (0.74, 0.82, 0.82)),
                (cup.cut(recess),
                 f"hollow_suction_cup_{label}", (0.23, 0.47, 0.37)),
                (cyl_y(p.flange_d/2, 90, 30, x, z),
                 f"pad_flange_{label}", (0.45, 0.85, 0.64)),
                (cyl_y(41, 120, 30, x, z),
                 f"pad_hub_{label}", (0.34, 0.45, 0.49)),
            ]
    return plate, parts


def export_3d(plate, parts):
    assembly = cq.Assembly(name="WC01_Wall_Climber")
    for shape, name, rgb in parts:
        if not shape.isValid() or shape.Volume() <= 0:
            raise ValueError(f"Invalid CAD solid: {name}")
        assembly.add(shape, name=name, color=cq.Color(*rgb))
    assembly.export(str(OUT/"WC01_Assembly.step"), unit="MM")
    assembly.export(str(OUT/"WC01_Assembly.glb"))
    plate.export(str(OUT/"WC01_Rear_Plate.step"), "STEP")
    return assembly


def add_layers(doc):
    for name, color, weight in [
        ("OUTLINE", 7, 35), ("BODY", 4, 25), ("ARMS", 8, 25),
        ("PADS", 3, 25), ("CENTER", 8, 13),
        ("DIMENSIONS", 2, 18), ("NOTES", 6, 18)
    ]:
        doc.layers.new(name, dxfattribs={"color": color, "lineweight": weight})


def rect(msp, left, bottom, right, top, layer):
    msp.add_lwpolyline([(left,bottom),(right,bottom),(right,top),(left,top)],
                       close=True, dxfattribs={"layer":layer})


def segment(msp, a, b, layer):
    msp.add_line(a, b, dxfattribs={"layer":layer})


def circ(msp, x, y, radius, layer):
    msp.add_circle((x,y), radius, dxfattribs={"layer":layer})


def note(msp, x, y, message, height=28, layer="NOTES"):
    msp.add_text(message, dxfattribs={"height":height,"layer":layer}).set_placement((x,y))


def dim(msp, base, p1, p2, angle=0, height=25):
    msp.add_linear_dim(base=base, p1=p1, p2=p2, angle=angle,
                       dimstyle="Standard", dxfattribs={"layer":"DIMENSIONS"},
                       override={"dimtxt":height,"dimasz":14,"dimdec":0,
                                 "dimlfac":1,"dimexo":6,"dimexe":9,
                                 "dimgap":7,"dimclrd":2,"dimclre":2,
                                 "dimclrt":2}).render()


def make_general_dxf(p):
    body_back, body_front, shoulder_x, shoulder_z, elbow_x = layout(p)
    doc = ezdxf.new("R2013")
    doc.units = ezdxf.units.MM
    add_layers(doc)
    m = doc.modelspace()
    # Front elevation: 2D X is model X; 2D Y is model Z.
    rect(m, -p.rear_plate_w/2, -p.rear_plate_h/2,
         p.rear_plate_w/2, p.rear_plate_h/2, "OUTLINE")
    rect(m, -p.body_w/2, -p.body_h/2,
         p.body_w/2, p.body_h/2, "BODY")
    rect(m, -(p.body_w-150)/2, -30-(p.body_h-240)/2,
         (p.body_w-150)/2, -30+(p.body_h-240)/2, "BODY")
    circ(m, 0, 60, 85, "BODY")
    circ(m, 0, 60, 39, "BODY")
    for side in (-1,1):
        for level in (-1,1):
            x, z = side*p.pad_x, level*p.pad_z
            a, e = (side*shoulder_x,level*shoulder_z), (side*elbow_x,z)
            segment(m,a,e,"ARMS")
            segment(m,e,(x,z),"ARMS")
            circ(m,e[0],e[1],60,"ARMS")
            circ(m,x,z,p.flange_d/2,"PADS")
            circ(m,x,z,p.cup_d/2,"PADS")
            circ(m,x,z,5,"CENTER")
    half_w=p.pad_x+p.flange_d/2
    half_h=max(p.rear_plate_h/2,p.pad_z+p.flange_d/2)
    dim(m,(0,-670),(-half_w,-half_h),(half_w,-half_h))
    dim(m,(0,580),(-p.pad_x,p.pad_z),(p.pad_x,p.pad_z))
    dim(m,(-1100,0),(-p.pad_x,-p.pad_z),(-p.pad_x,p.pad_z),90)
    dim(m,(1160,0),(half_w,-half_h),(half_w,half_h),90)
    dim(m,(0,-485),(-p.body_w/2,-p.body_h/2),
        (p.body_w/2,-p.body_h/2),height=19)
    note(m,-960,740,"FRONT ELEVATION | NEUTRAL POSE",42)
    note(m,-960,-800,"4X SUCTION CUP DIA 210 / FLANGE DIA 252",26)

    # Right side: local horizontal coordinate is distance from the wall (Y).
    ox=2500
    rect(m,ox+p.rear_plate_y,-p.rear_plate_h/2,
         ox+p.rear_plate_y+p.rear_plate_t,p.rear_plate_h/2,"OUTLINE")
    rect(m,ox+body_back,-p.body_h/2,ox+body_front,p.body_h/2,"BODY")
    rect(m,ox+body_front,-30-(p.body_h-240)/2,
         ox+body_front+20,-30+(p.body_h-240)/2,"BODY")
    rect(m,ox+body_front+20,-25,ox+body_front+80,145,"BODY")
    rect(m,ox+body_front+80,21,ox+body_front+95,99,"BODY")
    for level in (-1,1):
        z=level*p.pad_z
        rect(m,ox+20,z-p.cup_d/2,ox+90,z+p.cup_d/2,"PADS")
        rect(m,ox+90,z-p.flange_d/2,ox+120,z+p.flange_d/2,"PADS")
        segment(m,(ox+body_back+50,level*shoulder_z),(ox+235,z),"ARMS")
        segment(m,(ox+235,z),(ox+150,z),"ARMS")
    segment(m,(ox, -500),(ox,500),"CENTER")
    note(m,ox-15,535,"WALL PLANE",23)
    dim(m,(ox+(body_front+115)/2,-670),(ox+20,-446),
        (ox+body_front+95,-446))
    note(m,ox-30,740,"RIGHT SIDE ELEVATION",42)

    # Top view, projected under front view.
    oy=-1640
    rect(m,-p.rear_plate_w/2,oy+p.rear_plate_y,
         p.rear_plate_w/2,oy+p.rear_plate_y+p.rear_plate_t,"OUTLINE")
    rect(m,-p.body_w/2,oy+body_back,p.body_w/2,oy+body_front,"BODY")
    rect(m,-(p.body_w-150)/2,oy+body_front,
         (p.body_w-150)/2,oy+body_front+20,"BODY")
    rect(m,-85,oy+body_front+20,85,oy+body_front+95,"BODY")
    for side in (-1,1):
        x=side*p.pad_x
        rect(m,x-p.cup_d/2,oy+20,x+p.cup_d/2,oy+90,"PADS")
        rect(m,x-p.flange_d/2,oy+90,x+p.flange_d/2,oy+120,"PADS")
        segment(m,(side*shoulder_x,oy+body_back+50),(side*elbow_x,oy+235),"ARMS")
        segment(m,(side*elbow_x,oy+235),(x,oy+150),"ARMS")
    note(m,-960,oy+755,"TOP VIEW",42)
    note(m,-950,-1925,"WC-01 | WALL CLIMBER CAD CONCEPT | UNITS: mm | REV A",32)
    note(m,-950,-1995,"Reference geometry only. Verify loads, adhesion, joints and clearances before fabrication.",23)
    path=OUT/"WC01_General_Arrangement.dxf"
    doc.saveas(path)
    return doc, path


def make_plate_dxf(p):
    doc=ezdxf.new("R2013")
    doc.units=ezdxf.units.MM
    add_layers(doc)
    m=doc.modelspace()
    rect(m,-p.rear_plate_w/2,-p.rear_plate_h/2,
         p.rear_plate_w/2,p.rear_plate_h/2,"OUTLINE")
    for x in (-p.hole_pitch_x/2,p.hole_pitch_x/2):
        for z in (-p.hole_pitch_z/2,p.hole_pitch_z/2):
            circ(m,x,z,p.hole_d/2,"OUTLINE")
            segment(m,(x-22,z),(x+22,z),"CENTER")
            segment(m,(x,z-22),(x,z+22),"CENTER")
    dim(m,(0,-540),(-p.rear_plate_w/2,-p.rear_plate_h/2),
        (p.rear_plate_w/2,-p.rear_plate_h/2))
    dim(m,(0,545),(-p.hole_pitch_x/2,p.hole_pitch_z/2),
        (p.hole_pitch_x/2,p.hole_pitch_z/2))
    dim(m,(-530,0),(-p.hole_pitch_x/2,-p.hole_pitch_z/2),
        (-p.hole_pitch_x/2,p.hole_pitch_z/2),90)
    dim(m,(530,0),(p.rear_plate_w/2,-p.rear_plate_h/2),
        (p.rear_plate_w/2,p.rear_plate_h/2),90)
    note(m,-370,650,"REAR MOUNTING PLATE | 4X DIA 12 THROUGH",36)
    note(m,-370,-655,f"MATERIAL/TOLERANCE: TO BE SPECIFIED | THICKNESS: {p.rear_plate_t:g} mm",24)
    note(m,-370,-710,"WC-01 CONCEPT PART | ALL DIMENSIONS IN mm | REV A",23)
    path=OUT/"WC01_Rear_Plate_Profile.dxf"
    doc.saveas(path)
    return doc, path


def render_dxf(doc, path, figsize=(16,10), dpi=160):
    fig, ax=plt.subplots(figsize=figsize,dpi=dpi)
    fig.patch.set_facecolor("#0a1720")
    ax.set_facecolor("#0a1720")
    Frontend(RenderContext(doc),MatplotlibBackend(ax)).draw_layout(doc.modelspace(),finalize=True)
    ax.set_aspect("equal",adjustable="box")
    ax.axis("off")
    fig.savefig(path,dpi=dpi,facecolor=fig.get_facecolor(),bbox_inches="tight",pad_inches=0.25)
    plt.close(fig)


def render_isometric(parts, path):
    fig=plt.figure(figsize=(11,8),dpi=140,facecolor="#0a1720")
    ax=fig.add_subplot(111,projection="3d",computed_zorder=False)
    ax.set_facecolor("#0a1720")
    # Colored surfaces come from the same exact solids as the STEP assembly.
    for shape, name, rgb in parts:
        vertices, indices=shape.tessellate(6.0,0.25)
        pts=np.array([[v.x,v.y,v.z] for v in vertices])
        tri=pts[np.array(indices,dtype=int)]
        ax.add_collection3d(Poly3DCollection(tri,facecolors=[rgb],
                                              edgecolors="none",linewidths=0,
                                              antialiased=False))
    ax.set_xlim(-1000,1000)
    ax.set_ylim(-120,730)
    ax.set_zlim(-540,540)
    ax.set_box_aspect((2000,850,1080),zoom=1.10)
    ax.view_init(elev=18,azim=68)
    ax.set_axis_off()
    fig.text(.5,.94,"WC-01 | SOLID CAD ASSEMBLY",ha="center",color="white",
             fontsize=19,weight="bold")
    fig.text(.5,.90,"Symmetric reference pose  •  exact STEP solids  •  millimeters",
             ha="center",color="#92b7bb",fontsize=10)
    fig.savefig(path,dpi=140,facecolor=fig.get_facecolor(),bbox_inches="tight")
    plt.close(fig)


def main():
    validate(P)
    plate, parts=make_parts(P)
    export_3d(plate,parts)
    general_doc,_=make_general_dxf(P)
    plate_doc,_=make_plate_dxf(P)
    render_dxf(general_doc,OUT/"WC01_General_Arrangement.png",(18,11),150)
    render_dxf(plate_doc,OUT/"WC01_Rear_Plate_Detail.png",(9,9),160)
    render_isometric(parts,OUT/"WC01_3D_Assembly_Preview.png")

    notes=OUT/"README.txt"
    notes.write_text(
        "WC-01 Wall-Climbing Robot: CAD concept\n"
        "Units: millimeters. Model pose: neutral symmetric, four pads at wall.\n"
        f"Body enclosure: {P.body_w:g} x {P.body_h:g} x {P.body_depth:g} mm.\n"
        f"Pad centers: {2*P.pad_x:g} horizontal x {2*P.pad_z:g} vertical.\n"
        f"Pad: diameter {P.cup_d:g}; flange diameter {P.flange_d:g}.\n"
        f"Rear plate: {P.rear_plate_w:g} x {P.rear_plate_h:g} x {P.rear_plate_t:g}; "
        f"4 holes diameter {P.hole_d:g} at pitches {P.hole_pitch_x:g} x {P.hole_pitch_z:g}.\n"
        "STEP: editable B-rep solids/assembly in CAD import; native feature history stays in Python.\n"
        "DXF: 2D orthographic and plate profile with dimension entities.\n"
        "GLB: colored mesh preview.\n"
        "Concept only: verify suction, friction, mass, torque, joints, sealing, materials, "
        "tolerances and collisions before fabrication.\n"
    )
    package=Path("WC01_CAD_Design_Package.zip")
    with zipfile.ZipFile(package,"w",zipfile.ZIP_DEFLATED) as zf:
        for file in sorted(OUT.iterdir()):
            zf.write(file,arcname=file.name)

    print("Created:")
    for file in sorted(OUT.iterdir()):
        print(f"  {file.name} ({file.stat().st_size:,} bytes)")
    print(f"  {package.name} ({package.stat().st_size:,} bytes)")
    if display is not None:
        display(Image(filename=str(OUT/"WC01_General_Arrangement.png")))
        display(Image(filename=str(OUT/"WC01_3D_Assembly_Preview.png")))


if __name__ == "__main__":
    main()

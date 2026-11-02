# WC-01: Wall-Climbing Robot

**Python simulation and parametric CAD concept** · Google Colab · DXF · STEP

[Open 2D simulation in Colab](https://colab.research.google.com/github/Javed00004/my-projects-/blob/main/wc01-wall-climbing-robot/notebooks/01_2d_climbing_animation.ipynb) · [Open 3D simulation in Colab](https://colab.research.google.com/github/Javed00004/my-projects-/blob/main/wc01-wall-climbing-robot/notebooks/02_3d_climbing_animation.ipynb) · [Open CAD notebook in Colab](https://colab.research.google.com/github/Javed00004/my-projects-/blob/main/wc01-wall-climbing-robot/notebooks/03_cad_design.ipynb)

This repository explores a four-pad wall-climbing robot. It shows an alternating diagonal-pad gait in 2D and 3D, then translates the nominal geometry into dimensioned drawings and solid CAD files. The model is intended for research discussion and further engineering work. **It is not a tested physical robot or a validated adhesion design.**

![WC-01 solid CAD assembly](assets/figures/cad_assembly.png)

## Project at a glance

| Workstream | What is included | Current status |
| --- | --- | --- |
| Motion | [2D](assets/videos/2d_climb.mp4) and [3D](assets/videos/3d_climb.mp4) six-second climbing videos | Kinematic animation |
| 3D design | [STEP assembly](cad/assembly.step), [colored GLB](cad/assembly.glb), [concept STL](cad/animated_pose_concept.stl) | Solid geometry and mesh previews |
| 2D design | [General arrangement DXF](cad/general_arrangement.dxf) and [rear-plate DXF](cad/rear_plate_profile.dxf) | Orthographic views and dimensions |
| Reproduction | Three [Colab notebooks](notebooks/) and their [Python sources](src/) | Parameters can be changed and files regenerated |

The animations use a **diagonal alternating gait**. Each suction pad stays fixed on the wall while marked as attached; two or four pads are attached at every sampled frame. This describes motion only. The videos do not calculate whether those pads can support the robot's weight.

## Explore the work

1. Open [the 2D notebook](notebooks/01_2d_climbing_animation.ipynb) for the gait concept.
2. Open [the 3D notebook](notebooks/02_3d_climbing_animation.ipynb) for the rendered climb and a concept STL.
3. Open [the CAD notebook](notebooks/03_cad_design.ipynb) to change dimensions and regenerate the DXF, STEP, GLB, and drawing previews. Run its setup cell first in Google Colab.

The CAD notebook holds the editable dimension block in its `Design` dataclass. The static CAD assembly represents a **symmetric reference pose**, while the animation moves the feet through a gait cycle.

### Reference geometry

All CAD dimensions are in **millimeters**. These are concept choices, not measured or optimized specifications.

| Feature | Nominal dimension |
| --- | ---: |
| Body enclosure | 660 × 720 × 220 mm |
| Suction cup diameter / flange diameter | 210 / 252 mm |
| Pad-center horizontal / vertical spacing | 1540 / 640 mm |
| Rear mounting plate | 730 × 800 × 12 mm |
| Rear-plate holes | 4 × Ø12 mm on a 610 × 680 mm pitch |
| CAD assembly bounding box | 1792 × 595 × 892 mm |

See [design and kinematics](docs/design-and-kinematics.md) for the coordinate system, gait equations, and geometric assumptions.

## Reproduce locally

Use Python 3.12 and FFmpeg. The notebooks are the simplest route in Colab. For a local run after cloning `Javed00004/my-projects-`, enter this project folder:

```bash
cd my-projects-/wc01-wall-climbing-robot
python -m pip install -r requirements.txt
python src/wall_climber_render.py
python src/wall_climber_3d.py
python src/wall_climber_cad.py
```

The scripts write generated files into the current directory; the CAD script creates `WC01_CAD_Output/`. Curated outputs are already included under `assets/` and `cad/`. See [reproducibility and checks](docs/reproducibility.md) for the tested environment and file validation.

## Engineering questions to investigate next

- How much usable adhesion remains on rough or porous wall surfaces under leakage and pressure variation?
- What are the friction and overturning margins during two-pad support phases?
- What torque, bending strength, and control response do the arm joints need for a specified mass and speed?
- How should contact sensing detect seal failure and trigger a safe stop?

These are **proposed investigations**. This repository contains no experimental results, physical prototype measurements, or peer-reviewed publication. A possible [research roadmap](docs/research-roadmap.md) separates the current work from future validation.

## Provenance

This portfolio repository was prepared with AI assistance under Muhammad Javed's direction. It is a new simulation and CAD concept; it does not contain the original bachelor's thesis report, prototype data, or independent experimental results. See [project provenance](docs/provenance.md) for the asset history and scope.

## File formats

The DXFs are editable 2D drawings with dimension entities. STEP carries CAD solids and assembly structure into software such as SolidWorks, but it does not transfer the Python feature-generation history. Edit the notebooks or `src/` scripts to regenerate geometry; the GLB and STL are mesh formats for viewing. The CAD concept needs material selection, tolerances, mechanical design, contact analysis, and prototype testing before fabrication.

**Project direction:** Muhammad Javed · Mechatronics engineering

The Colab links above load the notebooks from this project folder in `Javed00004/my-projects-`. The notebooks can also be uploaded directly to Colab. See [upload verification](docs/upload-verification.md) for the checks run while preparing this repository, and `SHA256SUMS.txt` for the complete file inventory.

**Tool references:** [CadQuery export documentation](https://cadquery.readthedocs.io/en/latest/importexport.html) · [ezdxf dimension documentation](https://ezdxf.readthedocs.io/en/stable/tutorials/linear_dimension.html)

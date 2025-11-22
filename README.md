
<img src="https://github.com/ramakarl/just_math/blob/main/gallery/img_teaser.jpg" width="900">

# just_math
Just Math - A collection of pure math demos.

The goal of Just Math is to provide both a visual example and code demonstrations of specific concepts in computer graphics, mathematics, simulation and AI. 

## News & Updates

Nov 22, 2025 - Faster and simpler builds. Clean re-commit. Reduced download from 150MB to ~12 MB.<br>
Sep 16,2023 - Obj materials sample added.<br>
Aug 12,2023 - G-coder sample added.<br>
Mar 13,2023 - Bilinear Patch intersection sample added.<br>
Feb 7, 2023 - Artificial Neural Network sample added. PS: For fun I wrote the original wikipedia article for <a href="https://en.wikipedia.org/wiki/Tensor_(machine_learning)">Tensor (machine learning)</a>.<br>

## Samples

Each sample in Just Math demonstrates a specific concept in code and visually.
The samples provided are briefly described:
- 3DDDA - 3D Differential Analyzer. March through a volume to identify voxels on a line.
- ANN - Artificial Neural Network learning the sine function.
- Basis - Orthonormal bases. Transformation from one space to another using bases.
- Bilinear Patch - Fast raytracing of a bilinear patch (curved quad defined by 4x points).
- Cells - Cellular membrane simulation. Simulated with physics using circles for cells.
- Deform - 3D spatial deformations, including bending, twisting and folding.
- Gcoder - Generation of CNC toolpath g-code from depth images.
- InvK - Inverse Kinematics using quaternions. Demo of robot and human arm IK.
- Obj Materials - Reads and renders 3D meshes (.obj) with their material (.mtl) definitions.
- QuatSquad - Quaternion Squad. A C1 continuous method for interpolating orientations.
- QuatTrajectory - Trajectory interpolation of both position and orientation,
using B-Splines, Bezier Curves, and Catmull-Rom splines for position. Slerp or Squad for orientation.
- Raycast - Rendering of a 3D volume with an opacity-based volume integral, on CPU.
- Voxelizer - Voxelization of triangle into a volume, using several methods.
- WangTiles - Sampling of spatial distribution functions with scale invariance.
- WangTiles3D - Alternative demo of Wang Tiles for 3D geometry instancing over a density map landscape.

## How to Build

**Updated Nov 2025**<br>
Build with cmake is now simpler and faster.<br>
Libmin dependent code is now directly compiled with each project (no shared or static libs).<br>
Steps:<br>
1. Clone this just_math repo<br>
2. Clone <a href="https://github.com/ramakarl/libmin">libmin</a> as a sibling folder<br>
<pre>
\codes
 ├── \just_math
 └── \libmin
</pre>
3. Run cmake or cmake-gui on specific sample folder.<br>

Let me know if you have any issues building.

## Contact
Feel free to contact me if you have any questions, comments or suggestions:<br>
**Rama Hoetzlein** <br>
Website: <a href="https://ramakarl.com">ramakarl.com</a><br>
Email: ramahoetzlein@gmail.com<br>

## License & Copyright info
MIT License.<br>
Copyright 2007-2024 (c) Quanta Sciences & Rama Hoetzlein<br>
The Just Math samples are MIT Licensed.<br>
Libmin is MIT Licensed with contributions from other BSD and MIT licensed sources.<br>




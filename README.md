# just_math

Just Math - A collection of pure math demos.

The goal of Just Math is to provide both a visual example and code demonstrations of specific concepts in computer graphics, mathematics, simulation and AI. 

## Sample Gallery

<div style="display:flex">
<img src="https://github.com/ramakarl/just_math/blob/main/gallery/img_3ddda.JPG" width="200">
<img src="https://github.com/ramakarl/just_math/blob/main/gallery/img_basis.JPG" width="200">
<img src="https://github.com/ramakarl/just_math/blob/main/gallery/img_bp.jpg" width="200">
<img src="https://github.com/ramakarl/just_math/blob/main/gallery/img_cells.jpg" width="200">
<img src="https://github.com/ramakarl/just_math/blob/main/gallery/img_deform.jpg" width="200">
<img src="https://github.com/ramakarl/just_math/blob/main/gallery/img_invk.jpg" width="200">
<img src="https://github.com/ramakarl/just_math/blob/main/gallery/img_quatsquad.jpg" width="200">
<img src="https://github.com/ramakarl/just_math/blob/main/gallery/img_raycast.jpg" width="200">
<img src="https://github.com/ramakarl/just_math/blob/main/gallery/img_trajectories.jpg" width="200">
<img src="https://github.com/ramakarl/just_math/blob/main/gallery/img_wangtiles.jpg" width="200">
<img src="https://github.com/ramakarl/just_math/blob/main/gallery/img_wangtiles3d.jpg" width="200">
</div>

## Just Math Samples

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

## News & Updates

**UPDATES**: <br>
Dec 5, 2023 - Build steps confirmed working with latest libmin repo and Visual Studio 2019 in Dec 2023 for all samples. Removed <a href="https://github.com/ramakarl/libmin">libmin</a> from this repo and put into its own separate repository.<br>
Sep 16,2023 - Obj materials sample added.<br>
Aug 12,2023 - G-coder sample added.<br>
Mar 13,2023 - Bilinear Patch intersection sample added.<br>
Feb 7, 2023 - Artificial Neural Network sample added. PS: For fun I wrote the original wikipedia article for <a href="https://en.wikipedia.org/wiki/Tensor_(machine_learning)">Tensor (machine learning)</a>.<br>

## How to Build
<br>
* Updated 2025 *
Build is now simpler and faster. Libmin dependent code is now directly compiled with each project (not static or shared linked).
Steps:<br>
1. Clone this repo
2. Clone <a href="https://github.com/ramakarl/libmin">libmin</a> as a sibling folder
`
\codes
  \just_math
  \libmin
`
3. Run cmake or cmake-gui on specific sample folder.

## Contributions
I am interested in building a community around simple, well documented, math codes, in pure C/C++ for CPU (no shaders), with interactive graphical demos (not just youtube videos) that are MIT/BSD Licensed. If you have similar interests contact me at: Rama Hoetzlein, ramahoetzlein@gmail.com

## License & Copyright info
MIT License.<br>
Copyright 2007-2024 (c) Quanta Sciences & Rama Hoetzlein: 3DDDA, ANN, Basis, Bilinear_Patch, Cells, Deform, Displace, Gcoder, Invk, Obj_Materials, QuadSquad, QuatTraj, Raycast, Voxelizer<br>
Copyright 2022-2023 (c) Abram Connelly and Rama Hoetzlein: math_belief_prop (incl. BMS, WFC, BP, LSA)<br>
The Just Math samples are MIT Licensed.<br>
Libmin is MIT Licensed with contributions from other BSD and MIT licensed sources.<br>
Contact: Rama Hoetzlein at ramahoetzlein@gmail.com




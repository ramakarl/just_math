TODO
===

###### 2023-10-03

* calculate entropy center of mass ($\mu = \sum _ {\text{cell}} E _ c(\text{cell}) p _ {\text{cell}}$)
* calculate entropy variance ($\nu = \frac{ (E_c(\text{cell}) - \mu)^2 }{ \sum _ {\text{cell}} \mathbb{1} _ {E_c({\text{cell}}) > 0} }$)
* choose soften policy less aggressively
  - allow for trying many different regions with higher entropy rather than just one before softening


###### 2023-09-22

* ~block level WFC needs randomness on restarts~
* investigate conflicts of BMS outside of block choice
  (percentage of contradictions that fall outside of
  the block)
* is map hysteresis screwing us? (rail tile has cone
  of influence above it, then gets whacked out but leaves
  constrained cone in the map)

###### 2023-09-18

* different boundary conditions
  - mirror, loop/modular, arbitrary
* Breakout Model Synthesis
  - global constraints (occupancy, connectivity, etc.)


###### 2023-09-01

* Breakout Model Synthesis
  - ~further debugging (Rama noticed some buggy behavior during visualization)~ (fixed)
  - ~choice of block~ (done)
    + ~based on block entropy~
    + ~add random component~
  - global constraints (occupancy, connectivity, etc.)
* ~statistics to calculate correlation length or cluster size (or their analogs
  in this domain)~ (mostly done)
* different boundary conditions
  - mirror, loop/modular, arbitrary



###### 2023-07-25

* ~block WFC~ (done)
* Tiled has the option for multiple layers, so we can shove 3d tiles on multiple layers
  or have multiple layers for 2d realizations that have failed (to see other options)

###### 2023-04-02

* add 'escheresque' tilset:
  - https://github.com/merrell42/model-synthesis/blob/master/samples/escheresque.txt
* ~investigate block based approach from model synthesis~
  - https://www.boristhebrave.com/2021/10/26/model-synthesis-and-modifying-in-blocks/


###### 2023-03-25

* implement 'fixing frames'
  - force certain constrained tiles/cells and make sure
    algorithm doesn't touch them afterwards.
  - will allow us to set up smaller, potentially constrained
    problems, to test the different algorithms on
* ~implement backtracking algorithm~ (done?)
  - consider 'look ahead' algorithm to try and
    cull tiles that we know will lead to contradiction
    if they're fixed (sort of done, needs some validation
    and work) (done)
* statistics for knowing where in the cluster (chain of
  implication after a tile has been chosen/forced) that
  leads to a contradiction (low priority, not sure how to progress)
  - the idea is that there might be information if the
    contradiction happens with high frequency at the end
    of a long 10 chain instead of near the start of the
    implication chain
* ~2d tileset png~

###### 2023-03-12

* ~integrate wfc into realize pre/post/step~


###### 2023-03-12

* additional statistics:
  - mean/mode/std-dev occupancy
  - mean/mode/std-dev cluster size?
    + cluster defined as how many tiles get forced should a particular
      tile get chosen?


###### 2023-03-12

* stat additions
  - average occupancy (number of tiles per cell) at each step
    possibly with variance
* ~heat map overlay of grids picked~ (done-ish)

###### 2023-03-06

* img2tile improvements:
  - avg color
  - add an optional 'base' tileset in case people just want to use
    it to parse an example file with base tiles
* add 3d stl input instead of just obj

###### 2023-03-04

* ~make sure wfc option is working~
  - think it is...
* ~other window sizes and geometries for img2tile~
  - 3x3 works
  - nixing the other geometries. Cross is adds complexity without
    giving anything valid
* 3d output
  - ~obj input, stl output~
  - needs some fixing up
* config file
* avg color for tilename
* ~figure out location and generation of js tilesets (stair)~
* ~3d pipe tileset (js?)~
  - ~oskar pipe (not checked in?)~
* ~converge eps start/end along with interpolation function~
  - we can guess how to interpolate based on how many tiles
    we've fixed so far
  - degenerate is just fixed eps, so doesn't break anything
    to put in
  - default to linear
  - can specify python notation range for the `-e` option
* getting worried about what looks to be a preference
  for one tile type over another. Some results look too
  homogeneous.
  - Regardless, needs to be explained
  - try and figure out how to test if there is a preference
    either based on location, tile order or something else

###### 2023-02-22

* extend img2tile to accomodate 3x3 and other windows
  - test base tile to make sure it's working
* consider how to visualize the different tiles and their
  neighbors
* produce an initial tiled output file from img2tile to
  confirm tileset is meaningful
* run on wfc to see results and make sure it's still working
  - create statistics of failures for various tile collections
* put back in weighting option for img2tile (name and rule)

###### 2022-11-30

* ~Implement different schedules for fixing tile choices in the collapse stage~
  - ~choose 'least' belief to remove instead of max belief to fix~
  - ~allow for other schedules~
  - ~Do min entropy min belief (removal)~
* ~Implement "residual" belief propagation updates to try and speed up convergence~
  - sort of implemented, test, not that conclusive
* ~Visualize different aspects including:~
  - ~belief~
  - ~belief of restricted tile sets~
  - ~messages~
* Add heuristic to help with convergence
  - tile correlation?
* Add extra CSV columns in `_name.csv` add 6 additional fields with plus 1 for color (DONE)
  - face direction of tile `x+,x-,y+,y-,z+,z-`
  - `0` if no connection in that direction, `1` if connection in that direction
  - "post" rotated tile (maps to world space)
  - color in hex format
* Add simplified 2d tilesets that can scale
  - 2d tileset with 1-2 colors with 4 bends, 2 straights, 4 endtiles and empty (10 per set + empty) for each color
  - create canonical examples from simplified tileset that can test different problem sizes
* ~implement svd speedup~ (used eigen3)
  - ~https://github.com/lucasmaystre/svdlibc~
* ~merge optimizations from upstream repo~
* ~implement checkerboard updating~
* ~implement automated tests~
* add other 2d tilesets for testing and demos (from oga):
  - https://opengameart.org/content/gameboy-tileset
  - https://opengameart.org/content/zelda-like-tilesets-and-sprites
  - https://opengameart.org/content/blowhard-2-blow-harder
  - https://opengameart.org/content/8x8-tileset-by-soundlust
  - https://opengameart.org/content/16x16-fantasy-tileset
  - https://opengameart.org/content/zoria-tileset
  - https://opengameart.org/content/overhead-action-rpg-forest
  - https://opengameart.org/content/micro-tileset-overworld-and-dungeon
  - https://opengameart.org/content/a-blocky-dungeon
  - https://opengameart.org/content/dungeon-tileset
  - https://opengameart.org/content/1-bit-pack
  - https://opengameart.org/content/micro-roguelike
  - https://opengameart.org/content/monochrome-rpg
* add 3d tilesets and figure out reasonable export/import format (vox?):
  - https://opengameart.org/content/marble-kit
  - https://github.com/ephtracy/voxel-model/blob/master/MagicaVoxel-file-format-vox.txt
  - goxel looks like a nice FOSS alternative



---

Path forward:

* ~tests (automated)~
* ~checkerboard~
* ~svd~
* ~code refactor/optimizations~

---

Come up with stop condition for CPU implementation.

For reference, 10x10x10x46 ~ 7m25s.

The original overworld for Zelda looks to be 256x88x(8x18-25).

So `(256*88*(8*18-25)) / (10*10*10*46) ~= 58.73`, so looking to speed it up by 60x.

Checkerboard might give 2x, SVD might give 3x for a total of 6x.

Residual BP might give 4x (speculative), with SVD at 3x gives 12x.

---

* [gamedev SO on contradictions in WFC](https://gamedev.stackexchange.com/questions/178443/resolving-contradictions-in-wfc-more-efficiently-than-naive-backtracking)
  - [tileset gif](https://imgur.com/1b8FYNG)
  - [oskar stalberg pipes (tw)](https://twitter.com/Nolithius/status/1218534693903138818)
  - imgur [link0](https://imgur.com/ZLVkuaI) [link1](https://imgur.com/XXyrpmc)
  - [reddit post](https://www.reddit.com/r/proceduralgeneration/comments/eq2vxh/resolving_error_states_in_wfc_more_efficiently/)

---

Running into errors with `svdlibc` (`example_f2.mat` hangs).

Alternatives:

* [eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page) ([mpl2 license](https://www.mozilla.org/en-US/MPL/2.0/FAQ/))
* [armadillow](https://arma.sourceforge.net/faq.html) ([apache2](https://opensource.org/licenses/Apache-2.0))

---

Potential 3d models

* https://sketchfab.com/3d-models/monument-valley-4d3880dff578465b8ae0c732da878d0d
* https://sketchfab.com/3d-models/monument-valley-level-design-995687a3ace948738a8cbe1739d74137
* https://sketchfab.com/3d-models/monument-valley-level-experiment-03d44e345dd942339f7ddd8205e6fb5f


---

References
---

* [WFC tutorial](https://medium.com/swlh/wave-function-collapse-tutorial-with-a-basic-exmaple-implementation-in-python-152d83d5cdb1)
* [procjam WFC](https://www.procjam.com/tutorials/wfc/)
* [WFC very clearly](https://robertheaton.com/2018/12/17/wavefunction-collapse-algorithm/)
* [WFC OverlappingModel.cs](https://github.com/mxgmn/WaveFunctionCollapse/blob/master/OverlappingModel.cs)
* [Tilings and Projection Set Algorithms](https://gvarnavides.com/musings/tilings-and-projection-set-algorithms/)




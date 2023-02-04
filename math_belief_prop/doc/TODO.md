TODO
===

###### 2022-11-30

* Implement different schedules for fixing tile choices in the collapse stage
  - ~choose 'least' belief to remove instead of max belief to fix~
  - ~allow for other schedules~
  - ~Do min entropy min belief (removal)~
* ~Implement "residual" belief propagation updates to try and speed up convergence~
  - sort of implemented, test, not that conclusive
* Visualize different aspects including:
  - belief
  - belief of restricted tile sets
  - messages
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
* implement svd speedup
  - https://github.com/lucasmaystre/svdlibc
* merge optimizations from upstream repo
* implement checkerboard updating
* implement automated tests


---

Path forward:

* ~tests (automated)~
* checkerboard
* svd
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

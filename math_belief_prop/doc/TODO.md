TODO
===

###### 2022-11-30

* Implement different schedules for fixing tile choices in the collapse stage
  - choose 'least' belief to remove instead of max belief to fix
  - allow for other schedules
* Implement "residual" belief propagation updates to try and speed up convergence
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

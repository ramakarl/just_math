Developer Notes
===

Buffers
---

| | |
|---|---|
| `BUF_TILE_IDX_N`    | holds current admissible count of tiles for a given cell |
| `BUF_TILE_IDX`      | holds tile at cell position and tile index. Note that order of tiles for a given cell position can get jumbled |



A basic loop through all values in the grid would go as follows:

```
  for (cellPos = 0; cellPos < getNumVerts(); cellPos++) {
    tileIndexCount = getValI( BUF_TILE_IDX_N
    for (tileIndex = 0 ; tileIndex < tileIndexCount; tileIndex++) {
      tileVal = getValI( BUF_TILE_IDX, tileIndex, cellPos );
    }
  }
```

Program Flow
---

Programs wanting to use BP with the various algorithms and options
should structure the flow roughly as follows:

```
  if (bp.start() < 0) {
    // failure
  }

  ret = -1;
  for (iteration=0; iteration < maxIterations ; iteration++) {

    ret = bp.RealizePre();
    if (ret < 0) { break; }

    while (bp.RealizeStep > 0) { }

    ret = bp.RealizePost();
    if (ret <= 0) { break; }

  }

  if (ret == 0) {
    // success
  }
  else if (ret < 0) {
    // failure
  }

```

File Formats
===

This is a description of the file formats used in this implementation
of Belief Propagation.

Name File
---

The name file is a comma separated CSV file that names tiles, associating
an integer ID to a string name for the tile.

Lines beginning with a hash (`#`) are ignored.

The format is:

```
<integer_id>,<tile_name>
```

Here are the first few lines of an example name file:

```
# tile_id,tile_name
0,.000
1,|000
2,|001
```

Rule File
---

The rule file is a comma separated CSV file that gives a pair wise probability
to tiles for each direction.

Lines beginning with a hash (`#`) are ignored.

The format is:

```
<tileid_a>,<tileid_b>,<direction_id>,<weight>
```

The `<tileid_a>` and `<tileid_b>` are tile IDs as indicated in
the name file.

The direction ID is an integer code indicating which direction
the rule is for.
The direction IDs are as follows:

| Direction ID | Interpretation |
|--------------|----------------|
| 0 | positive X direction `(+1, 0, 0)` |
| 1 | negative X direction `(-1, 0, 0)` |
| 2 | positive Y direction `( 0,+1, 0)` |
| 3 | negative Y direction `( 0,-1, 0)` |
| 4 | positive Z direction `( 0 ,0,+1)` |
| 5 | negative Z direction `( 0, 0,-1)` |

The weight should be a number from 0 to 1 (inclusive).
If no rule is given for a particular tile ID pair, it is
assumed to be 0.

So far, only weights of `0` and `1` have been assumed
indicating which tile pairings are admissible with a `1` value.
This may change in the future.

Here are the first few lines of an example rule file:

```
0,0,0,1
0,1,0,1
0,3,0,1
0,5,0,1
0,6,0,1
0,7,0,1
```

Constraints File
---

A constraints file is a comma separated CSV file that indicates what
tiles should be populated in grid realization.

The constraints file is verbose and clunky and was only created to
give the possibility of providing constraints to a particular realization.
This format will change in the future but is documented here so
that the format is recorded and the assumptions are stated.

The constraints file has the following format:

```
<x>,<y>,<z>,<tile_id>
```

Each line indicates the `x`, `y` and `z` position with
an allowed `tile_id` at that position.
Each grid position is assumed to be completely unpopulated
with any tile IDs and each line in the constraints file
is a tile ID to be added to that position.

It is an error to run `bpc` on a constraints file
whose dimensions don't match the instantiated problem
size.
It as in error to have a position be unpopulated with
at least one tile ID. 

Here are the first few lines of an example constraints file:

```
0,0,0,0
0,0,0,13
0,0,0,14
0,0,0,15
0,0,0,16
0,0,0,17
0,0,0,18
```



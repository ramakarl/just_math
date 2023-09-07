Breakout Model Synthesis
===

These are rough notes for a nascent idea of an extension to Merrell's Model Synthesis (MMS) algorithm
for more general classes of models, including some that might be constrained in particular ways.
The following discussion is still rough and only lightly tested but is provided here for future
reference.

---

---

A modification to Merrell's Model Synthesis (MMS) algorithm, called Breakout Model Synthesis (BMS).

Some terms:

* A "tolerated" state is one in which the model is arc consistent.

* A "wildcard" grid is a model which has a full tile choice for every cell.

* A "prefatory" state is one in which boundary and any other initial constraints have been propagated on a wildcard grid.

* A "realization" of a set of cells, in a block, say, is one in which every cell in question has exactly one tile
  and is arc consistent both within the block and within the model the block potential resides in.

* A grid that is an "indeterminate" state is a grid that is not fully realized.

* A "cell" is a position in the grid, potentially having many tiles.


A rough overview of the algorithm is provided:

```
/* BREAKOUT-MODEL-SYNTHESIS algorithm */
Create an initial prefatory arc consistent model $M = M_0$ (if no such model exists, fail)

Repeat $T_{mix}$ times {

  /* BLOCK-POSITION phase */
  Choose a block, $B$, of cells to modify

  /* BLOCK-REALIZE phase */
  Repeat until a valid configuration is found or a maximum $T_B$ tries has been made {

    Find a realization of tiles in cells contained in $B$, propagating constraints to the cells outside of $B$ and store in $M'$
    If $M'$ is arc consistent (aka tolerated), $M = M'$ and break

  }

  If no valid realization of block $B$ could be found from the previous step {
    /* SOFTEN phase */
    Restore block $B$ to its prefatory state and restore all block neighbors, $B_{nei}$, to their prefatory state
    propagate constraints into and out of the prefatory state
  }

}

```

Note that for simplicity, block $B$ is assumed to have the same span in each dimension and
$B_{nei}$ is also assumed to have the same span in each dimension, though the size of $B$ and $B_{nei}$ need
not be the same.
That is, $B$ is assumed to be a cube (square) as is $B_{nei}$ assumed to be a cube (square), though the
size of the cube (square) for $B$ and $B_{nei}$ need not be the same.

The above algorithm is only a slight modification from MMS but differs in a key factor that
allows the initial state of the grid to not be in a fully realized state.
Relaxing the initial configuration of the grid allows for potentially complex initial conditions
and inference on models that might have rules where an initial realization might be non trivial to discover
or labor intensive to create.

The `SOFTEN` phase potentially allows BMS to recover from a contradictory state.

One assumption is that there is a fundamental "realization size", $S_V$, for any given tile set where, given a grid or block within
a larger grid, the chance of finding a valid realization is almost surely impossible for a block or grid larger than $S_V$.
Another assumption is that there is a fundamental "constraint propagation radius", $R_C$, at play, where any given cell
with a realized tile will not have a significant effect beyond $R_C$ cells (potentially measured in Manhattan distance).
In the `SOFTEN` phase, the 
neighbor block dimensions, $\max(\dim(B_{nei}))$, should be chosen such that $\max(\dim(B_{nei})) > 2 \cdot R_C$ to allow
the neighboring blocks to take on values unaffected by potentially poisonous choices from the block, $B$, in question.
The block size, $\max(\dim(B))$, should be chosen so that $\max(\dim(B)) < S_V$ so that there is a non negligible chance
of being able to find a valid block realization in the `BLOCK-REALIZE` phase.

Note that $S_V$ and $R_C$ might have different values depending on boundary conditions, such as the choosing blocks near
the edge of the grid, or boundary conditions from a partial realization of the grid.
$S_V$ and $R_C$ might not even exist depending on the tile set or grid configuration.
Practically, $S_V$ will also be dependent on the algorithm to resolve the $B$ block.
For simplicity, $S_V$ and $R_C$ are assumed to exist and be fixed, even though this is almost surely not true in the general
case.


This independence assumption is perhaps true for many typical tile sets and configurations but can easily be violated for
intentional choices of tile sets or special configurations.
NP-Completeness should be considered the norm in this domain so the best one should hope for is solutions to typical
cases, both in choice of tile sets and configurations.

Block position choice (in the `BLOCK-POSITION` phase) can be chosen to fit the domain in question but perhaps some reasonable
choices are:

* Pick the block with the lowest entropy as measured by the tile choice and probabilities
* Pick a random block
* Pick blocks from a fixed set, say with overlapping regions as in MMSs original algorithm

Taking lowest entropy with a random power law factor to add noise might work well in practice.

By allowing indeterminate state in the initial grid, this potentially allows BMS to find realizations from
tile sets that have longer reaching or global constraints that MMS would not be able to realize.


Potential Pitfalls
---

As mentioned previously, finding a realization given an initial configuration and tile set is an NP-Complete,
problem, so a tractable general solution is not expected.
Tile sets can be created in which finding a valid configuration is almost surely intractable.
Restricted configurations, by altering initial constraints, say, can be created from a wide variety of
generic tile sets in which finding a valid configuration is almost surely intractable.

There is a question of whether BMS will fail for larger grid sizes, even if the tile set is suitably generic or
there are no novel constraints imposed on the grid configuration.
One could imagine that higher order or more subtle constraints could come into play as the grid becomes realized
and that the radius in the SOFTEN step in the BMS algorithm won't be large enough to escape from a locally consistent,
but ultimately contradictory, state.
Another way to view this is there is a non-negligible chance that $S_V$ or $R_C$ don't exist or change as grid sizes increase.

NP-Completeness and, more broadly, Turing machine equivalence, should be considered the norm, rather than the exception,
in this domain.
The fundamental assumption for tractable problems in this domain is that tractable instances are the most likely
choice when picking from the ensemble imposed by the tile set and configuration.
BMS assumes tractable instances take the form of of having only local, finite effects when choosing tiles.
Should this local constraint assumption be violated, there is the potential for BMS to get trapped in a configuration
basin that could be improbable to escape from.

Miscellaneous Notes
---

Fundamental to the functioning of this idea is that the correlation length, however it's measured, is finite
for generic tile sets and configurations.
A finite correlation length means choosing a big enough block size, and 'soften' length, wil be able to resolve a
section independent of the surrounding configuration.

The question comes up on how to measure the various quantities analogous to correlation length and cluster
size in use for percolation, the Ising model, etc.

Here are some suggestions:

* Create a large grid and do constraint propagation so that it's in an arc consistent state (AC3)
  - Choose corners, boundaries and the center, then fix one 'center' tile to see how far the constraint
    propagation filters out
  - Choose larger bundles of cells to fix the tiles therein and measure how far the constraints propagate
* Generate a large set of resolved configurations and for each configuration:
  - Choose corners, boundaries and the center
  - Remove a block, potentially with non linear/planar boundaries, and make each cell wildcard
  - Run constraint propagation on the fuzzed wildcard area
  - Collect statistics on feasability of finding a solution for every tile value in the center of the removed area

---

There's a question on how to pick the block to process.
I think a reasonable heuristic is:

* Of all the possible blocks available, what is the block that will yield the minimum
  entropy after it's been fuzzed and constraints have been propagated (grid is in
  arc consistent state)

This is still a heuristic but it gets closer to the spirit of choosing the "minimum
entropy".
It's, unfortunately, a bit computationally expensive to compute, so, as a compromise,
maybe finding what a "canonical" entropy is (for edge, corner, middle) and picking
a block that has entropy closest to it might be good enough.

A block that that is fully realized will be completely fuzzed out and is an undesirable
pick, especially considering other areas that might have unresolved blocks.
A block that is fully wildcard is also undesirable because this is maximum entropy.
The "ideal" case is when there's a (arc consisten, constraint propagated) block that
is completely surrounded by a realized grid.
Fuzzing a block has essentially no effect as it should be identical after fuzzing and
constraing propagation, so it's not introducing any more entropy from the fuzzing state.

There could be cases when fuzzing and constraint propagation (without wfc) could yield
a lower entropy state but this should be rare?






---

All content, unless specifically stated otherwise, is licensed under CC0 license.

LICENSE: CC0




References
---

* [arc consistency](https://www.sciencedirect.com/topics/computer-science/arc-consistency#:~:text=Definition%2013.2,variable%20pairs%20are%20arc%20consistent.)
* [AC-3 on Wikipedia](https://en.wikipedia.org/wiki/AC-3_algorithm)
* [Example-Based Model Synthesis by Paul Merrell](https://paulmerrell.org/wp-content/uploads/2022/03/model_synthesis.pdf) ([model synthesis](https://paulmerrell.org/model-synthesis/))
* [infinite wfc](https://marian42.de/article/infinite-wfc/)

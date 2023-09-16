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

Repeat until a realization is found or a maximum $T_{max}$ tries has been made {

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
A finite correlation length means choosing a big enough block size, and 'soften' length, will be able to resolve a
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
  - Collect statistics on feasibility of finding a solution for every tile value in the center of the removed area

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
The "ideal" case is when there's a (arc consistent, constraint propagated) block that
is completely surrounded by a realized grid.
Fuzzing a block has essentially no effect as it should be identical after fuzzing and
constraint propagation, so it's not introducing any more entropy from the fuzzing state.

There could be cases when fuzzing and constraint propagation (without WFC) could yield
a lower entropy state but this should be rare?


Radius of Influence
---

Alternate names:

* implication radius
* tangle radius
* influence radius
* hook radius

The underlying assumption is that there is something like a 'radius of influence' that
is finite for these tile sets.
Some tile sets that have large influence radius still work, so this is not a clear characterization.

One attempt at defining the influence radius is:

$$
\begin{array}{ll}
\forall s, t \in M :& |t - s| > R, \\
\forall d_s \in D_s, \forall d_t \in D_t, &
|\Pr\{ u_t = d_t | u_s = d_s\} - \Pr\{ u_t = d_t \}| > \epsilon
\end{array}
$$

Where the probability is assumed to be over all valid configurations.

Besides being clunky to define, the above is, in general, intractable to compute as it
requires a full enumeration of states.
One can hope to try and get at this idea by measuring an arc consistent influence
by starting in a prefatory state, fixing a tile value in the middle and seeing what the farthest
cell that's affected after propagating constraints.

---

In relation to BMS, we're concerned with finding realizations.
WFC is a "one-shot" algorithm, meaning that it will stop when it reaches a contradiction.
Without backtracking, WFC will most likely fail for constrained tile sets past a certain size,
so some other algorithm needs to be used.

MMS has the advantage of always producing valid states but at the cost of finding an initial
valid state and the disadvantage of potentially being locked in a basin of solutions without
the ability to be able to break out to sample other basins of solutions.

Breakout model synthesis (BMS) draws from the advantage of WFC and MMS by running the constraint
propagation of WFC but restricted to a block, like MMS.
Unlike MMS, the initial state need not be specified, potentially allowing a broader set of solutions.

Whereas MMS can get trapped into a basin of solutions, like in the 'river runs through it' example,
BMS defers the choice of the state by allowing cell positions to have more tile values available.
BMS also offers the advantage of not requiring the explicit setup of a ground state, as in MMS,
which can be labor intensive or non-trivial to find.

The deferred choice that BMS offers comes at the cost of space and run time. BMS requires the grid to
store tile value choices still available inflating space considerations from an efficient implementation
of MMS.
BMS further runs constraint propagation on a potentially partially realized grid surrounding a chosen
block which could increase the run time of an efficient implementation of MMS, which only requires local
constraint propagation to within the block being considered.

For problems that are amenable to MMS, MMS is probably the better choice.
BMS is probably a better choice when initial ground state is difficult to create, for example when
a ground state is not obvious or is labor intesive to create, or when the choice of ground state for
MMS would lock it into a basin of solutions that do not explore the full solution space.

One example of MMS getting locked into a strict subset solution space is if there are structural
features embedded in the tile set whose size is larger than a block.
MMS will never be able to find these structures unless they exist in the ground state, as there
is no way to discover a valid grid state that only has part of the structure embedded within a block
sized region.

This can be a tricky balance as BMS will also have problems finding a realization if the tile set
is too constrained, all of which have effects on the probability of finding a valid realization restricted
to a block, the block size to begin with and how large the SOFTEN region should be.

For BMS, choosing the block size too small can lead to getting trapped in a local minima.
Choosing the block size too large can lead to the inability to find any realization, as the
WFC portion will not be able to find, with reasonable probability, a valid realization for
the large block region.

We are concerned with BMS's ability to actually find solutions.
To that end, we can test to see what the probability is of finding a solution given
a block size.

One option is to vary the block size and potentially the placement region to estimate how probable
a solution can be found.

One idea is to try and tie the arc consistent influence radius, $R_{ac}$, to the block size but this is
tricky as we're really more concerned with the 'constraidedness' of the tile set which may or
may not have a direct relation to $R_{ac}$.
For example, a structure that is completley or near completely specified might have a large $R_{ac}$
but is not constrained in the usual sense as it's easy to resolve, even with a small block size.

There are competing ideas:

* constraidednes - how much freedom does each tile have to either be next to each other or be
                   chosen at any given location
* arc consistent influence radius - what the average or maximum grid cell length is that is affected
  by fixing a particular tile at a grid location 
* block size - how big to choose a sub region to work on

---

* too overly constrained, wfc will work because the implication is deterministic or near deterministic
* too underconstrained and wfc or mms will work
* a mix of underconstrained and over constrained and nothing will work as it's optimally hard


---

* multi-colored ouroborous.
* left-to-right and right-to-left train


---

All content, unless specifically stated otherwise, is licensed under CC0 license.

LICENSE: CC0




References
---

* [arc consistency](https://www.sciencedirect.com/topics/computer-science/arc-consistency#:~:text=Definition%2013.2,variable%20pairs%20are%20arc%20consistent.)
* [AC-3 on Wikipedia](https://en.wikipedia.org/wiki/AC-3_algorithm)
* [Example-Based Model Synthesis by Paul Merrell](https://paulmerrell.org/wp-content/uploads/2022/03/model_synthesis.pdf) ([model synthesis](https://paulmerrell.org/model-synthesis/))
* [infinite wfc](https://marian42.de/article/infinite-wfc/)

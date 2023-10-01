Notes
===


Some notes to record some issues when running BP.

SVD fails on inputs it shouldn't
---

SVD should be almost completely equivalent.

There's at least one run that converges for vanilla BP
but fails to converge for SVD.

```
./bpc -R ./assets/stair_rule.csv -N ./assets/stair_name.csv -I 100000000 -D 10 -G 4 -V 1 -S 124  -e 0.00025
./bpc -R ./assets/stair_rule.csv -N ./assets/stair_name.csv -I 100000000 -D 10 -G 4 -V 1 -S 124  -e 0.00025 -E
```

todo:

* do a side by side comparison of the underlying matrix multiplication to see if it diverges

Oskar tilesets have weird boundary issues
---

When running vanilla BP, the Oskar tilesets have boundary issues
where they tend to prefer long pipes.
Contrast this with a successful BP run, where it looks much more
random and what I would think it should look like.

todo:

* check to see if setting the rate lower fixes the issue

See below for a potential reason of this issue.


Residual BP is not as fast as I would like
---

Interestingly, reducing the rate seems to help?


Residual BP
---

For certain combinations of rate, convergence epsilon,
it looks like it fails to converge.

For example, an epsilon of 0.000005 and a rate of .99999
gets stuck at around 0.00007.

Setting the rate to 1 has significant problems and, at
least for residual BP, it quickly has problems converging.


Oskar Pipes get trivial answer
---

Running vanilla BP on the uniform probability tileset, with start
pipes appropriately filtered on initialization, results in
a 'trivial' answer of small connecting pipes on the bottom layer.

I suspect this is not so much a bug as a feature as this potentially
*is* the maximum likelyhood state.
One should do a quick calculation to confirm but the number of states
of this form might grossly outweigh other 'non-trivial' states, so
that it is finding a very likely state.

This might also explain the other states that have a random distribution
on tiles as it's choosing the maximum likelyhood of a straight vertical
pipe, say, resulting in the 'sheets' that show up.

Residual BP might be travelling down the gradient curve much more randomly,
resulting in more non-uniform realizations.

To me, this is reminiscent of varying the temperature, where vanilla BP
looks a lot like what I would expect a "zero temperature" solution to look
like whereas the residual BP looks much more like a higher temperature solution
would look like.

Some potential solutions (without a deep understanding of what's going on):

* Fix some tiles on the boundary or center to try and encourage non-trivial
  solutions
* Fiddle with the rate to see if that helps break some of the uniformity,
  potentially increasing/decreasing the rate as it gets closer to the end
* Vary the individual tile probability or the pairwise probability depending
  on location in the grid

For reference:

A bottom layer of 'half-pipes':

```
time ../bpc \
  -N ./example_tile_collection/oskar-pipe_name.csv \
  -R ./example_tile_collection/oskar-pipe_rule.csv \
  -L ./example_tile_collection/oskar-pipe_objloc.csv \
  -D 8 -I 100000000 -e 0.00001 -G 2 -w 0.96 \
  -S 123 -J 'd[][1:][] 1' \
  -M ./oskar_d8.stl  \
  -V 2 | tee oskar_d8.log
```

---


Knoll et all suggest looking back in an $L$ window to see if the messages are
repeating when doing residual belief propagation.
If so, they suggest injecting Gaussian noise (noise injection BP or NIBP) to help alleviate the situation.

Alternatively, Knoll et all suggest introducing a weighting factor that dampens the considered
residual to schedule updates that have occurred more often less (weight damped belief propagation or WDBP).
That is, instead of sorting by residual, sort by residual divided by number
times updates ($\frac{ r _ { i, j } }{  \\# \text{ times chosen } }$).

Note that the convergence rate for RBP, NIBP or WDBP is higher than the "vanilla" BP case ("asynchronous BP" (ABP) or
"round robin BP"), the quality might suffer.
That is, RBP and others might converge, incorrectly, for many more graphs than for ABP, giving substandard results.

As a quick spot check, the results for the Ising glass model Knoll et all were using look to give about a 50% worst
MSE for RBP, NIBP and WDBP, even though the convergence rate was both higher and quicker.

Discussion
---

* noticing quadratic or cubic run time
  - cubic run might be because edge culling (on pm, say)
    make the average tile occupancy low that after increasing
    the grid dize, the number of tiles, $T$, goes from $O(T)$
    to $O(T^2)$
  - quadratic because we sweep every cell during the main BP run,
    fix a cell with a tile, then repeat

At the end, we expect $O(N^2 T^2 S)$, with:

* $N$ - number of cells ($X \cdot Y \cdot Z$)
* $T$ - number of tiles
* $S$ - average number of steps until convergence (dependent on epsilon, rate, etc.)

* WFC gets worse as the system size gets larger because it's a one-shot algorithm
  - 90x90 for pm gives many failures
  - 20x18x18 for stair gives many failures
* In principle, BP should be better at these cases, and WFC should have an exponentially
  decreasing chance of finding a solution
  - unclear whether WFC can be fixed up with some backtracking that would allow it to
    overcome this exponentially decreasing probability of finding an answer

---

```
eps_converge
spatial dimension
# tilecount
constraidedness
```


BEM
---

```
pct_solved = 1 - #{ tile count == 1} / (X*Y*Z)
block_entropy = \sum_{p \in B} cell_entropy(p)
cell_entropy(p) = \sum_{j \in D_p} g_p(j) \lg g_p(j)
g_p(p) = \frac{ G_p(j) }{ \sum_{j \in D_p} G_p(j) }
block_mass(B) = \frac{ \sum_{ p \in B | |D_p| > 1 } p }{ #\{ p | |D_p| > 1 \} }
```

Where $g_p(j)$  is the normalized probability, dependent on
what's left at the cell $p$ ($G_p(\cdot)$ is the original,
non-normalized probability).


Now:

```
Pre:
  # choose block B from some schedule (max entropy + noise)
  if we're in a fresh run:
    if (pct_solved <= 0.95):
      pick max entropy block (+noise)
    else:
      pick max entropy block outside of radius
      if entropy in outside radius is 0, decrease radius size

  # jitter block B
  move the block choice by a small random amount from currently chosen position
  

Step:
  ...
  if (pct_solved > 0.95) and
     (cur_block_mass < prev_block_mass) and
     (rand() < 0.5):
    accept (partial) block
  ...
```

Some notes:

* Proceeds as normal until it gets to 'frustrated and entangled' (FaE) state
  (pct_solved > 0.95)
* In the FaE state, we want to corral/herd frustrated regions together
  - We pick the center as the destination of the corralling
  - To focus attention on frustrated elements on the periphery, we
    reject all block choices inside a 'punch-out' radius
  - We selectively and probabilistically accept partial block solutions so long as the
    frustrated regions within them are closer to the destination point
    (e.g. the center)
  - The selective biasing of choice of partially completed blocks is
    independent of the underlying solver (as is most of the rest of these algorithms)
* As blocks on the periphery get solved or as the small frustrated regions get
  corralled to the destination point (e.g. the center), the punch-out radius
  shrinks to capture regions still on the periphery but closer to the center
* During block choice, including after every retry, the block is potentially
  jittered by some amount. That is, block choice is shifted by a small random amount
  - The jitter is done relative to the previous block choice so a block choice that
    has undergone multiple retries might be far from the initially chosen block position
  - One attempt was to bias the jitter towards the destination point but this functionality
    is currently removed
    
There are two main regions, the block choice and the block acceptance.

* Block choice is done to:
  - heavily bias outer blocks during FaE
  - allow some small randomization of block choice when
    retrying to give the attempt a better chance at overcoming
    local contradictions
* Block acceptance is done to:
  - allow for entropy corralling during FaE by biasing selections
    that favor the small frustrated regions within a block to move
    towards a destination point (e.g. the center)
    
---


Entangled elastic constraints.

Elastic constraints being contraints on configurations that are related to two or more locally frustrated
regions with each frustrated region allowed to wander very far from each other.
A canonical example of this is a self avoiding loop or path in 2d or 3d.
The number of paths entering a region must be even.
If a region gets into an arc consistent state but somehow could only resolve to an odd parity
boundary, it's only way to resolve that contradiction is to pair up with another region that has
the same issue.
Presumably regions can wander and this is what is meant by elastic, as they might be satisfied
locally or even start of local but then have the possibility of moving away from each other.

They're entangled in a sense because the map will remain frustrated until they eventually wander
into each other with the hope of resolution.

The current snapshot of the code has three ideas to try and take advantage of this observation:

* Instead of failing outright, allow for partially resolved blocks, biasing the center of
  mass in a certain direction
* Weight considered blocks heavier the further away from the rally point the blocks are
  as the rally point will tend towards a high entropy state
* Do the above process based on percentage of map completed, preferring it to do it when
  the map has settled into its cooled entangled phase.



Glossary
---

| Term | Description |
|---|---|
| PGM  | probabilistic graph model |
| BP   | belief propagation |
| ABP | asynchronous belief propagation (round robin update) |
| LBP   | loopy belief propagation |
| RBP   | residual belief propagation |
| NIBP | noise injected belief propagation |
| WDBP | weight decay belief propagation |
| MRF | Markov random field |
| MSE | mean square error |




References
---

* [pacman mazegen](https://shaunlebron.github.io/pacman-mazegen/) ([gh](https://github.com/shaunlebron/pacman-mazegen))
* [Texture Synthesis by A. Opara]( by https://youtu.be/fMbK7PYQux4) ([gh](https://github.com/EmbarkStudios/texture-synthesis))
* [Boris the Brave](boristhebrave.com)
  - [Tessera](https://www.boristhebrave.com/permanent/21/08/Tessera_A_Practical_System_for_WFC.pdf)
  - [Constraint-Based Tile Generators](https://www.boristhebrave.com/2021/10/31/constraint-based-tile-generators/)
  - [WFC Explained](https://www.boristhebrave.com/2020/04/13/wave-function-collapse-explained/)
* ["Message Scheduling Methods for Belief Propagation" by Knoll et all](https://github.com/abetusk/papers/blob/release/ComputerScience/BeliefPropagation/message-sched-for-bp_sknoll-rath-tschiatschek-pernkopf.pdf)
* [Constraint-Based 2D Tile GAme Blending with Sturgeon](https://www.youtube.com/watch?v=4abT8yKh-AQ)
* [WFC tutorial](https://medium.com/swlh/wave-function-collapse-tutorial-with-a-basic-exmaple-implementation-in-python-152d83d5cdb1)
* [procjam WFC](https://www.procjam.com/tutorials/wfc/)
* [WFC very clearly](https://robertheaton.com/2018/12/17/wavefunction-collapse-algorithm/)
* [WFC OverlappingModel.cs](https://github.com/mxgmn/WaveFunctionCollapse/blob/master/OverlappingModel.cs)
* [Tilings and Projection Set Algorithms](https://gvarnavides.com/musings/tilings-and-projection-set-algorithms/)
* [Quantogram](https://zaratustra.itch.io/quantogram?width=32&height=32&image=demo_pacman.png)
* [Paul Merrell](https://paulmerrell.org/model-synthesis/)
  - [Model Synthesis](https://paulmerrell.org/model-synthesis/)
  - [Model Synthesis on GitHub](https://github.com/merrell42/model-synthesis)
  - [Comparison Model Synthesis and WFC](https://paulmerrell.org/wp-content/uploads/2021/07/comparison.pdf)
  - [Boris the Brave on Model Synthesis](https://www.boristhebrave.com/2021/10/26/model-synthesis-and-modifying-in-blocks/)
* [Boris the Brave on 'backjumping'](https://twitter.com/boris_brave/status/1485006264119799811)
* [Maxim Gumin on twitter](https://twitter.com/ExUtumno/status/895684431477747715)
* [Creating Infinite Road for my Drifting Game](https://www.youtube.com/watch?v=n44DgwqnjxQ)
* [wfc zelda](https://observablehq.com/@makio135/zelda-wfc)
* [Infinite WFC for Clomper](https://www.youtube.com/watch?v=DrTYmUtWWw4) (infinite wfc but destroys saved collapsed state outside of field of view)
* [roads and platforms, wfc having trouble](https://twitter.com/0x21376B00/status/1544803545718804480) ([link](https://twitter.com/0x21376B00/status/1546776783663628288))


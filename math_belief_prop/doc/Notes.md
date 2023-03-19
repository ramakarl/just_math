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
times updates ($\frac{ r _ { i, j } }{ \text{ # times chosen } }$).

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

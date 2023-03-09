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





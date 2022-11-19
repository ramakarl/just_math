Belief Propagation Notes
===


The Belief Propagation (BP) algorithm in this repository is a specialized version that is specifically:

* Loopy Belief Propagation
* Regular square grids in 3 dimensions
* Each grid cell effectively holds a finite Markov Random Field (MRF) random variable (r.v.)
* Each MRF r.v. is homogeneous and can hold one of $D$ values


### Markov Random Field

Finite Markov Random Fields (MRF) will be considered.

$$ G(V,E), \ \ \  |V| = n, \ \ \ x_i \in D = \{ d_0, d_1, \cdots, d_{m-1} \}  $$

$$
i \in V \to g_i(\cdot) \in \mathbb{R}
$$

$$
(i,j) \in E \to f_{i,j}( \cdot, \cdot ) \in \mathbb{R}
$$

* $x_i$ represents the value of vertex $i$.
* $g_i(x_i)$ is the mapping of vertex $i$ with value $x_i$
* $f_{i,j}(x_i,x_j)$ is the mapping of the connected vertex $i$ and $j$
  with values $x_i$ and $x_j$, respectively

For example, $f_{i,j}(x_i,x_j)$ could be an indicator function that
vertex $i$ and $j$ could have values $x_i$ and $x_j$.

### Belief Propagation on a (discrete) Markov Random Field

Each vertex, $i$, can be associated with a random variable, $X_i$, taking
on (discrete) values chosen from some domain $D = \{ d_0, d_1, \cdots, d_{m-1} \}$ with
a probability distribution function $g_i(\cdot)$.

$$
\mu_{i,j}^{t+1}(b) = \sum_{a \in D} f_{i,j}(a,b) \cdot g_i(a) \cdot \prod_{k \in N(i) \text{ \\\\ } j} \mu_{k,i}^{t}(a)
$$

$$
P(X_i = a) \approx b^t_i(a) \propto g_i(a) \cdot \prod_{k \in N(i)} \mu^t_{k,i}(a)
$$

$$
\sum_{b \in D} \mu_{i,j}^{t}(b) = 1,  \ \ \ \ \sum_{a \in D} b^t_i(a) = 1
$$

The product can be more compactly represented by a function $h^t_{i,j}(\cdot)$:

$$
h^t_{i,j}(a) = g_i(a) \cdot \prod_{k \in N(i) \text{\\\\} j } \mu^t_{k,i}(a)
$$

$$
\mu_{i,j}^{t+1}(b) = \sum_{a \in D} f_{i,j}(a,b) \cdot h^t_{i,j}(a)
$$

One can recast this as a matrix multiplication:

$$ \begin{bmatrix} f_{i,j}(d_0,d_0) & f_{i,j}(d_1,d_0) &  \cdots & f_{i,j}(d_{m-1},d_0) \\\\ f_{i,j}(d_0,d_1) & f_{i,j}(d_1,d_1) &  \cdots & f_{i,j}(d_{m-1},d_1) \\\\ \vdots  & \vdots & \ddots & \vdots & \\\\ f_{i,j}(d_0,d_{m-1}) & f_{i,j}(d_1,d_{m-1}) &  \cdots & f_{i,j}(d_{m-1},d_{m-1}) \end{bmatrix} \begin{bmatrix} h_{i,j}^{t}(d_0) \\\\ h_{i,j}^{t}(d_1) \\\\ \vdots \\\\ h_{i,j}^{t}(d_{m-1}) \end{bmatrix} = \begin{bmatrix} \mu_{i,j}^{t+1}(d_0) \\\\ \mu_{i,j}^{t+1}(d_1) \\\\ \vdots \\\\ \mu_{i,j}^{t+1}(d_{m-1}) \end{bmatrix}
$$

$$
\to F_{i,j} \cdot \vec{h}^t_{i,j} = \vec{\mu}^{t+1}_{i,j}
$$


Since $h^t_{i,j}(\cdot)$ has no dependence on $b$, this speeds up a naive calculation by re-using the product results instead of re-calculating them.

If the $F_{i,j}$ matrix has low rank, $r < m$, it can be factored into a singular value decomposition (SVD) for performance:

$$U \cdot S \cdot V = \begin{bmatrix} \vec u_0 & \vec u_1 & \cdots & \vec u_{r-1} \end{bmatrix} \begin{bmatrix} s_0 & 0 &  \cdots & 0 \\\\ 0 & s_1 & \cdots & 0 \\\\ \vdots & \vdots  & \ddots & \vdots \\\\ 0 & 0 &  \cdots & s_{r-1} \end{bmatrix} \begin{bmatrix} \vec{v}^\dagger_0 \\\\ \vec{v}^\dagger_1 \\\\ \vdots \\\\ \vec{v}_{r-1}^\dagger  \end{bmatrix}
$$

Where $F_{i,j} = U \cdot S \cdot V$.

The matrix multiplication that was $O(m^2)$ now becomes two matrix multiplications of order $O(r \cdot m)$ for a potential speedup of $\sim \frac{m}{r}$.


### A Case Study In Failure

Consider a 2x2x1 grid that can have one of two realizations:


```
 2   3       2   3
|---|---|   |---|---|
| o | o |   | / | \ |
|---|---|   |---|---|
| o | o |   | \ | / |
|---|---|   |---|---|
 0   1       0   1
```

That is, either it's all "blank" (the empty tile `o`) or it's a small
looped road.
Each of the cell positions has been labelled with a number above or below it.

Call the empty tile (`o`) $d_0$ and each of the 'bend' tiles in the right realization
$d_1$, $d_2$, $d_3$ and $d_4$ from cell position `0` to `3` respectively.
That is:

* $d_1$ = `\` at position 0
* $d_2$ = `/` at position 1
* $d_3$ = `/` at position 2
* $d_4$ = `\` at position 3

We further assume the adjacency matrix $f_{i,j}(\cdot,\cdot)$ is `0` for the blank tile
next to each of the 'bend' tiles in the cell positions.

For the moment, let's focus on updating the $\mu$ values for cell position `0`.
We can write down what the updated $\mu$ should be:

$$ \mu^{t+1}_ {1,0}(d_0) = f_ {1,0}(d_0,d_0) \cdot \mu^{t}_ {3,1}(d_0) + f_ {1,0}(d_1,d_0) \cdot \mu^{t}_ {3,1}(d_1) $$

$$ \mu^{t+1}_ {1,0}(d_1) = f_ {1,0}(d_2,d_1) \cdot \mu^{t}_ {3,1}(d_2) + f_ {1,0}(d_0,d_1) \cdot \mu^{t}_ {3,1}(d_0) $$

$$ \mu^{t+1}_ {2,0}(d_0) = f_ {2,0}(d_0,d_0) \cdot \mu^{t}_ {3,2}(d_0) + f_ {2,0}(d_3,d_0) \cdot \mu^{t}_ {3,2}(d_3) $$

$$ \mu^{t+1}_ {2,0}(d_1) = f_ {2,0}(d_3,d_1) \cdot \mu^{t}_ {3,2}(d_3) + f_ {2,0}(d_0,d_1) \cdot \mu^{t}_ {3,2}(d_0) $$

Since we know a blank tile can't connect to a 'bend' tile, each of the terms on the right is zero because
the $f$ function is zero in that case.
In addition, in this case, the $f$ function only takes on two values, $0$ or $1$, which allows us to simplify
even further:

$$\mu^{t+1}_ {1,0}(d_0) = \mu^{t}_ {3,1}(d_0) $$

$$\mu^{t+1}_ {1,0}(d_1) = \mu^{t}_ {3,1}(d_2) $$

$$\mu^{t+1}_ {2,0}(d_0) = \mu^{t}_ {3,2}(d_0) $$

$$\mu^{t+1}_ {2,0}(d_1) = \mu^{t}_ {3,2}(d_3) $$

Here we see the problem.
This means that at every time step, $\mu_{j,0}(a)$ gets swapped with the corresponding
$\mu_{3,j}(a')$.

The values will cycle around without every converging, and this is what we see when we inspect
the $\mu$ values when running BP in this pathological case.

One possible solution is to set the 'rate' of update, providing a sort of momentum term
when updating the $\mu$ values.
In this pathological case, at least, the $\mu$'s converge, with a preference for one configuration
over another based on initial conditions of the random $\mu$ values.




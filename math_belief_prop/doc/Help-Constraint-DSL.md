Constraint DSL
===

To help facilitate easy generation of levels, a small DSL
has been been created to do basic operations on a grid.

The `-J` option is given a string to parse.

The tokens ` `, `\n` and `\t` (space, new line and tab) are
separators

The following are alist of commands:

| Operation | Description | Example |
|---|---|---|
| `d` | Delete | `d[][1:][] 0:2` |
| `a` | Add (unimplemented) | |
| `f` | Force | `f[][][0] 3` |


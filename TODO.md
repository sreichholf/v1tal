# TODO

## `v1seg` allocation checks

Static analysis (GCC `-fanalyzer`) found `malloc`/`calloc` results in `src/v1seg.cpp` that are
used without a NULL check. On the V1, an allocation failure therefore becomes a null
dereference instead of a clean producer failure.

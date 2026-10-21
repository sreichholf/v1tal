# TODO

## `v1seg` allocation checks — done (2026-10-21)

`src/v1seg.cpp` routes every `malloc`/`calloc` through `xmalloc`/`xcalloc`,
which log `[v1seg] allocation failed: ...` and `_exit(1)`.  A genuine
allocation failure is therefore a clean producer failure, and `v1appd` retries
with backoff instead of dereferencing NULL.

The producer is short-lived; measured peak RSS on a 348x402 map is ~7.5 MB.
It runs with `oom_score_adj=1000` (first OOM victim) and `v1appd` retries it
with backoff.

Verified with `g++ -fanalyzer`: no `-Wanalyzer-possible-null-dereference`.
The remaining analyzer categories (`malloc-leak`, `fd-leak`, `out-of-bounds`,
`use-of-uninitialized-value`) are false positives around the checked wrappers,
the out-parameter file reader, and the fixed-size byte readers (which are only
called after a length check); they are tracked in the private notes.

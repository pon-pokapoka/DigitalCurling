# DigitalCurling (pon-pokapoka fork) — notes for Claude Code

A fork of digitalcurling/DigitalCurling. Its own work, on `main`, is
`fcv1light` (`src/simulator/src/fcv1light/`): a faster approximation of
the FCV1 simulator for search, used by DigitalCurling4-CNN. Build
instructions are in README.md / README-en.md.

## Working in this repo

- **Branches.** Branch from this fork's `main` and open a PR against it.
  Never push straight to `main`. Commits are signed; never bypass a
  signing failure.
- **Upstream stays clean.** This file and the fork-only changes are not
  for digitalcurling/DigitalCurling. Anything proposed upstream starts
  from a branch off upstream's `main`, not this fork's.
- **`fcv1` is upstream's and stays unchanged.** Servers run it; a change
  there would make a client's search disagree with the server. Speed
  work goes into `fcv1light`.
- **`fcv1light` trades accuracy for speed on purpose.** It takes coarse
  steps and refines to fine steps only around a collision; stones resting
  against each other don't count as one (refining on every resting
  contact made a shot about 180× slower). Judge a change by how far its
  results move from `fcv1`'s and by its time per shot, and keep
  `src/simulator/test/test_fcv1light.cpp` (save/load, resting contact,
  hits) passing.
- **Tests:** configure with `-DDIGITALCURLING_BUILD_TEST=ON` (the default
  when this is the top-level project), build, and run `ctest`.
- **Public repo:** nothing specific to one deployment. Write new text in
  English; leave the existing Japanese docs alone unless asked.

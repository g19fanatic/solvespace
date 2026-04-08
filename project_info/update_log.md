# project_info Update Log

## Refresh: Apr 6, 2026 (git f854b536)

### Trigger
`/init --refresh` after 4 commits landed on top of the initial `/init` (3e5873e6):
- `019e0a4b` Add chamfer and fillet group operations
- `204b7cea` fix: Normalize chamfer/fillet orientation for face order
- `fd804775` fix: Register face entity for chamfer/fillet surface
- `257045cc` fix: Remove stale vertices from chamfer/fillet curves
- `f9104eb8` fix: Hide source vertices after chamfer/fillet
- `a133ce22` fix: Hide LINE_SEGMENT edges after chamfer/fillet
- `fb75a694` test: Add TDD tests for chained chamfer origin-line bug
- `f854b536` fix: Repair open trim polygons in chamfer/fillet caps

### Files Updated

#### `project_info/overview.md`
- Updated git hash from `3e5873e6` → `f854b536`

#### `project_info/context-strategy.md`
- Chamfer/Fillet Development section: updated `src/srf/chamfer.cpp` line reference from `~960` to `1366 lines`
- Updated test.cpp entry: `12+` → `63 tests (2863 lines)`
- Updated key subsystem doc description to include 7 helper functions + 63-test coverage
- Removed stale CHAMFER_DEBUG strip command (all lines already stripped)

#### `project_info/subsystems/chamfer-fillet.md` (major update)
- `chamfer.cpp` size: `~960 lines` → `1366 lines`
- Added Post-15 step: `BridgeTrimGapIfOpen` sweep to algorithm table
- Updated Step 15 key locations to match new line numbers (739, 790 for chamfer; 1270 for fillet)
- Updated fillet section start from `~600` → `849` (verified)
- Expanded Helper Functions section:
  - Added `TruncateCurveAtVertex` (new function at line 71)
  - Added `RemoveTrimLast` (line 114)
  - Added `UpdateAllSurfaceTrimEndpoints` (line 129) with detailed description
  - Added `FindTrimGap` (line 146)
  - Added `InsertTrimAt` (line 164) with implementation explanation
  - Added `BridgeTrimGapIfOpen` (line 187) with full algorithm description
- Updated File References table with new line ranges
- Updated test.cpp entry: `12+ tests` → `63 tests (2863 lines)`
- Removed stale CHAMFER_DEBUG strip command; noted strips complete
- Updated build note: `ctest` result now shows `63 tests, 0 failures`
- Added new "Test Coverage Summary" section:
  - 7 helper function table with line numbers
  - Test category breakdown table (63 tests across 16 categories)
  - "Origin-line regression" section with detection method + test locations
  - "Backface regression" section with detection method + test locations

#### `project_info/todos.md`
- Updated chamfer-fillet.md status description to reflect expanded content

### Preserved (no changes needed)
- `project_info/architecture.md` — no structural changes in these commits
- `project_info/tech-stack.md` — no new dependencies
- `project_info/code-patterns.md` — no new patterns
- `project_info/build-notes.md` — no build system changes
- `project_info/subsystems/solver.md` — no solver changes
- `project_info/subsystems/sketch.md` — no sketch data model changes
- `project_info/subsystems/ui.md` — textscreens.cpp chamfer UI already documented
- `project_info/subsystems/platform.md` — no platform changes

## Refresh: Apr 7, 2026 (git 96df040d)

### Trigger
`/init --refresh` after 2 commits landed on top of the last doc update (f854b536):
- `4594b7c4` docs: (doc-only update, already integrated into project_info at last refresh)
- `96df040d` fix(WIP!): Correct cap selection and trim repair for DIFF/ASSEMBLE fillets

### Files Updated

#### `project_info/overview.md`
- Updated git hash from `f854b536` → `96df040d`
- Updated description: "fix: Repair open trim polygons in chamfer/fillet caps" → "WIP: Correct cap selection and trim repair for DIFF/ASSEMBLE fillets"

#### `project_info/context-strategy.md`
- Updated `src/srf/chamfer.cpp` size from `1366 lines` → `1636 lines`
- Updated test.cpp entry from `63 tests (2863 lines)` → `77 tests`
- Updated key subsystem doc description to mention Step 16, DIFF geometry support, and cap scoring

#### `project_info/subsystems/chamfer-fillet.md` (major update)
- `chamfer.cpp` size: `1366 lines` → `1636 lines`
- Added Step 16: Handle surfaces with intermediate vertices on the shared edge V1V2 (`chamfer.cpp:1433`)
- Added RECON path description to algorithm: `trim.n==0` path for DIFF internal surface endcap reconstruction via graph traversal (`chamfer.cpp:~1388`)
- Added cap candidate scoring description to Step 10: selection by `|normal.Dot(t)|` instead of first-match (`chamfer.cpp:~1062`)
- Added note: ASSEMBLE stitch for third-surface trim edges ending at V1
- Added note: Removed broad `BridgeTrimGapIfOpen` fallback over ALL surfaces (replaced by targeted per-surface calls)
- Updated test count: `63 tests` → `77 tests` (+14 new fillet_diff_* tests)
- Added new test categories:
  - `fillet_diff_*` no-crash tests (12 tests via `CreateBoxWithCutout`/`CreateBoxWithCutoutFromBottom`)
  - `fillet_diff_endcap_has_triangles` (endcap triangle assertion)
  - `fillet_assemble_cap_no_backface` (ASSEMBLE backface geometry check)
- Added new test helpers to helper function table:
  - `FindPointNear` (`test.cpp:1588`)
  - `CreateBoxWithCutout` (`test.cpp:1600`)
  - `CreateBoxWithCutoutFromBottom` (`test.cpp:1684`)
- Added WIP state note: 2 debug `fprintf` statements remain (`RECON:`, `RECON_DONE:` — WIP state)
- Updated build note: `63 tests, 0 failures` → `77 tests, 0 failures (test count as of 96df040d)`

#### `project_info/todos.md`
- Updated chamfer-fillet.md status description to reflect 77 tests, Step 16, and DIFF geometry support

### Preserved (no changes needed)
- `project_info/architecture.md` — no structural changes
- `project_info/tech-stack.md` — no new dependencies
- `project_info/code-patterns.md` — no new patterns
- `project_info/build-notes.md` — no build system changes
- `project_info/subsystems/solver.md` — no solver changes
- `project_info/subsystems/sketch.md` — no sketch data model changes
- `project_info/subsystems/ui.md` — no UI changes
- `project_info/subsystems/platform.md` — no platform changes

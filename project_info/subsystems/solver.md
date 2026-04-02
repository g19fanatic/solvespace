# Constraint Solver Subsystem

> **Purpose**: Documents the SolveSpace constraint solver architecture — the `System` class, expression algebra, Jacobian construction, DOF calculation, parameter/equation lifecycle, and `SolveResult` semantics.

## Related Files

| File | Role |
|------|------|
| `src/system.cpp` | Solver implementation (Newton's method, Jacobian, substitution) |
| `src/solvespace.h:74-155` | `System` class declaration, `SolveResult` enum |
| `src/expr.h` | Symbolic algebra: `Expr`, `ExprVector`, `ExprQuaternion` |
| `src/param.h` | `Param` and `hParam` structures |
| `src/sketch.h:117-125` | `Equation` class definition |
| `src/generate.cpp:541-594` | `SolveGroup`, `WriteEqSystemForGroup`, `SolveGroupAndReport` |
| `src/constrainteq.cpp` | Constraint equation generation |

## Overview

SolveSpace solves geometric constraints via a **modified Newton's method** over a symbolic algebra system. Constraints, entities, and groups each contribute equations and parameters. The solver assembles these into a sparse Jacobian matrix `J·Δx = -F(x)` and iterates until convergence or failure.

The solver is group-scoped: each `Group` in sketch order is solved independently per `GenerateAll` pass (`src/solvespace.h:689`).

---

## SolveResult Enum

**Location**: `src/solvespace.h:44-50`

```cpp
enum class SolveResult : uint32_t {
    OKAY                     = 0,   // fully constrained, converged
    DIDNT_CONVERGE           = 10,  // Newton failed to converge
    REDUNDANT_OKAY           = 11,  // redundant constraints but converged
    REDUNDANT_DIDNT_CONVERGE = 12,  // redundant + failed to converge
    TOO_MANY_UNKNOWNS        = 20   // exceeded MAX_UNKNOWNS (2048)
};
```

Result stored in `Group::solved.how` (`src/sketch.h:213-219`) after each solve.

---

## Key Data Structures

### System Class (`src/solvespace.h:75-155`)

```
class System {
    enum { MAX_UNKNOWNS = 2048 };

    EntityList                    entity;   // shadow entities for solver
    ParamList                     param;    // parameters (unknowns)
    IdList<Equation, hEquation>   eq;       // symbolic equations
    ParamSet                      dragged;  // params being dragged (keep close to initial)

    struct { ... } mat;  // Jacobian matrix (sparse, Eigen)
};
```

The `System` object lives as `SS.sys` (global singleton, `src/solvespace.h` / `src/solvespace.cpp`).

### Equation (`src/sketch.h:117-125`)

```cpp
class Equation {
    int       tag;    // subsystem assignment tag
    hEquation h;      // handle (encodes origin: constraint vs entity vs group)
    Expr     *e;      // symbolic expression that equals zero when satisfied
};
```

An equation represents the constraint `e = 0`. The expression tree (`Expr*`) is built symbolically and then differentiated analytically for the Jacobian.

### Param (`src/param.h`)

```cpp
class Param {
    int    tag;    // subsystem tag or special values (VAR_SUBSTITUTED, VAR_DOF_TEST)
    hParam h;      // handle (bits 31:16 = request index, 15:0 = param index)
    double val;    // current value (updated by Newton iterations)
    bool   known;  // true if already solved (not an unknown)
    bool   free;   // true if under-constrained (DOF > 0 for this param)
};
```

### Jacobian Matrix (`src/solvespace.h:101-120`)

The matrix `mat` is an `m × n` sparse system where:
- `m` = number of active equations
- `n` = number of active parameters (unknowns)
- `mat.A.sym` = symbolic Jacobian (`Eigen::SparseMatrix<Expr*>`)
- `mat.A.num` = numeric Jacobian (`Eigen::SparseMatrix<double>`)
- `mat.B.sym` / `mat.B.num` = RHS (function values)
- `mat.X` = solution step vector

---

## Expr: Symbolic Algebra System (`src/expr.h`)

The `Expr` class implements a symbolic expression tree used to represent constraint equations and their partial derivatives.

**Operations**: `PLUS`, `MINUS`, `TIMES`, `DIV`, `NEGATE`, `SQRT`, `SQUARE`, `SIN`, `COS`, `ASIN`, `ACOS`

**Key methods**:
| Method | Description |
|--------|-------------|
| `Eval()` | Numerically evaluate the expression |
| `PartialWrt(hParam p)` | Symbolically differentiate w.r.t. parameter `p` |
| `FoldConstants()` | Constant-fold the expression tree |
| `Substitute(subMap)` | Apply parameter substitution map |
| `DeepCopyWithParamsAsPointers(...)` | Copy and resolve param handles to pointers (faster eval) |

**Composite types**: `ExprVector` (3 exprs), `ExprQuaternion` (4 exprs) for geometric expressions.

---

## Solve Lifecycle

### 1. System Population (`src/generate.cpp:481-551`)

`SolveSpaceUI::WriteEqSystemForGroup(hGroup hg)`:
1. Clear `sys.entity`, `sys.param`, `sys.eq`
2. For each `Request` in the group: call `r->Generate(&sys.entity, &sys.param)`
3. For each `Constraint` in the group: call `c->Generate(&sys.param)`
4. Call `g->Generate(&sys.entity, &sys.param)` for group-level equations
5. Initialize param values from previous solve (warm start)
6. Call `MarkDraggedParams()` — inserts dragged params into `sys.dragged`

### 2. Invoke Solver (`src/generate.cpp:554-570`)

`SolveSpaceUI::SolveGroup(hGroup hg, bool andFindFree)`:
```cpp
WriteEqSystemForGroup(hg);
SolveResult how = sys.Solve(g, &g->solved.dof, &g->solved.remove,
                            /*andFindBad=*/!g->allowRedundant,
                            /*andFindFree=*/andFindFree,
                            /*forceDofCheck=*/!g->dofCheckOk);
g->solved.how = how;
```

### 3. `System::Solve()` (`src/system.cpp`)

Main solver entry point flow:

```
WriteEquationsExceptFor(NO_CONSTRAINT, g)
    ├── Collect constraint equations via c->GenerateEquations(&eq)
    ├── Collect entity equations via e->GenerateEquations(&eq)
    └── Collect group equations via g->GenerateEquations(&eq)

SolveBySubstitution()        ← fast pre-pass: solve a=b type equations
    └── Tags substituted params/equations; returns SubstitutionMap

For each single-param equation:   ← alone=1,2,3,...
    WriteJacobian(alone)
    NewtonSolve()            ← solve in isolation (speedup)

WriteJacobian(0)             ← build Jacobian for remaining system
TestRank(dof)                ← DOF = n - rank(J)
NewtonSolve()                ← iterate up to 50 steps
    └── EvalJacobian() + SolveLeastSquares() + update params

Write results back to SK.param
Return SolveResult
```

### 4. DOF Calculation (`src/system.cpp`)

```cpp
bool System::TestRank(int *dof, int *rank) {
    EvalJacobian();
    int jacobianRank = CalculateRank();  // Eigen SparseQR
    *dof = mat.n - jacobianRank;         // DOF = unknowns - rank
    return jacobianRank == mat.m;        // true if fully constrained
}
```

DOF > 0 → under-constrained; rank < m → redundant constraints.

---

## Newton Solver Details

**Convergence tolerance**: `CONVERGE_TOLERANCE = LENGTH_EPS / 1e2` (`src/system.cpp:17`)

**Max iterations**: 50 per `NewtonSolve()` call

**Algorithm**:
1. Evaluate `F(x)` — current equation residuals
2. Evaluate Jacobian `J = ∂F/∂x`
3. Solve least-squares system: `J·Jᵀ·z = F`, `Δx = Jᵀ·z`
4. Update: `x_{n+1} = x_n - Δx`
5. Check convergence: `max(|F_i|) < CONVERGE_TOLERANCE`

**Dragged parameters** get a smaller scale factor (`1/20`) in least-squares to keep them close to their starting position during interactive drag.

---

## Substitution Optimization (`src/system.cpp`)

`SolveBySubstitution()` detects equations of the form `(a - b) = 0` and eliminates the parameter pair algebraically before Newton iteration. This significantly reduces system size for coincident-point constraints.

Tags used:
| Tag | Value | Meaning |
|-----|-------|---------|
| `VAR_SUBSTITUTED` | 10000 | Parameter was substituted away |
| `VAR_DOF_TEST` | 10001 | Temporarily tagged for DOF testing |
| `EQ_SUBSTITUTED` | 20000 | Equation handled by substitution |

---

## Error Diagnostics

When Newton fails to converge (`goto didnt_converge`):
- The solver scans `mat.eq` for residuals exceeding tolerance
- Identifies which `hConstraint` handles generated failing equations
- Adds them to `g->solved.remove` (the "bad" list)
- These are highlighted in red in the UI via `TextWindow::ReportHowGroupSolved`

`FindWhichToRemoveToFixJacobian()`: For redundant systems, exhaustively tries removing each constraint to find which one(s) cause the rank defect.

---

## Global Solver Instance

The solver lives as a field in `SolveSpaceUI`:
- `SS.sys` — the `System` object (`src/solvespace.h`)
- `SS.sys.Solve(g, ...)` — called from `SolveGroup` at `src/generate.cpp:560`
- `SS.sys.SolveRank(g, ...)` — rank-only check at `src/generate.cpp:578`

---

## Cross-References

- See [`project_info/code-patterns.md`](../code-patterns.md) — "Solve Lifecycle" pattern section
- See [`project_info/subsystems/sketch.md`](sketch.md) — handle system and equation generation
- See [`project_info/architecture.md`](../architecture.md) — where solver fits in the overall data flow
- Constraint equation generation: `src/constrainteq.cpp` (one function per constraint type)
- Entity equation generation: `Entity::GenerateEquations()` in `src/entity.cpp`

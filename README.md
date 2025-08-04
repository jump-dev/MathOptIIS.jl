# MathOptIIS.jl

[![Build Status](https://github.com/jump-dev/MathOptIIS.jl/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/jump-dev/MathOptIIS.jl/actions?query=workflow%3ACI)
[![codecov](https://codecov.io/gh/jump-dev/MathOptIIS.jl/branch/main/graph/badge.svg)](https://codecov.io/gh/jump-dev/MathOptIIS.jl)

[MathOptIIS.jl](https://github.com/jump-dev/MathOptIIS.jl) is a basic IIS solver
for MathOptInterface.jl.

## License

`MathOptIIS.jl` is licensed under the [MIT License](https://github.com/jump-dev/MultiObjectiveAlgorithms.jl/blob/main/LICENSE.md).

## Getting help

If you need help, please ask a question on the [JuMP community forum](https://jump.dev/forum).

If you have a reproducible example of a bug, please [open a GitHub issue](https://github.com/jump-dev/MathOptIIS.jl/issues/new).

## Installation

Install `MathOptIIS` using `Pkg.add`:

```julia
import Pkg
Pkg.add("MathOptIIS")
```

## Usage

This package is not intended to be called directly from user-code. Instead, it
should be added as a dependency to solver wrappers that want to provide an IIS.

To add to an existing wrapper, add a new field:
```julia
conflict_solver::Union{Nothing,MathOptIIS.Optimizer}
```

Then, add the following methods:
```julia
function MOI.compute_conflict!(model::Optimizer)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), model)
    MOI.set(solver, MathOptIIS.InnerOptimizer(), Optimizer)
    MOI.compute_conflict!(solver)
    model.conflict_solver = solver
    return
end

function MOI.get(optimizer::Optimizer, attr::MOI.ConflictStatus)
    if optimizer.conflict_solver === nothing
        return MOI.COMPUTE_CONFLICT_NOT_CALLED
    end
    return MOI.get(optimizer.conflict_solver, attr)
end

function MOI.get(
    optimizer::Optimizer,
    attr::MOI.ConstraintConflictStatus,
    con::MOI.ConstraintIndex,
)
    return MOI.get(optimizer.conflict_solver, attr, con)
end
```

## The name

The optimization community consistently uses "IIS", but they have not
standardized on what the acronym stands for. We have seen:

1. Irreducible Infeasible Set
2. Irreducibly Inconsistent Set
3. Irreducible Infeasible Subsystem
4. Infeasible Irreducible System
5. Irreducible Inconsistent Subsystem
6. Irreducibly Inconsistent System

So we choose the name MathOptIIS, and you can decide what the acronym stands for.

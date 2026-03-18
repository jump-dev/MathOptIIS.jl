# Copyright (c) 2025: Joaquim Dias Garcia, Oscar Dowson and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

module MathOptIIS

import MathOptInterface as MOI

include("interval.jl")

# ==============================================================================
# User-facing API
# ==============================================================================

"""
    struct Metadata{T,S}
        lower_bound::T
        upper_bound::T
        set::S
    end

A struct that provides additional information to help interpret an IIS. See the
docstring of `IIS` for details.
"""
struct Metadata{T,S}
    lower_bound::T
    upper_bound::T
    set::S
end

"""
    IIS{M<:Union{Nothing,Metadata}}

A struct that stores information describing an IIS. It has the following fields:

 * `constraints::Vector{MOI.ConstraintIndex}`: a list of constraint indices for
   which the status is `MOI.IN_CONFLICT`.

 * `maybe_constraints::Vector{MOI.ConstraintIndex}`: a list of constraint
   indices for which the status is `MOI.MAYBE_IN_CONFLICT`.

 * `metadata::M`: additional metadata that may be useful. There are four cases:

    1. `metadata::Nothing`: a variable contains conflicting bounds. `constraints`
       contains the offending bound constraints.

    2. `metadata::Metadata{T,MOI.Integer}`: a variable that is integer has bounds
       that conflict with the constraint that it is integer. `constraints`
       contains the offending bound constraints and the integrality constraint

    3. `metadata::Metadata{T,MOI.ZeroOne}`: a variable that is binary has bounds
       that conflict with the constraint that it is binary. `constraints`
       contains the offending bound constraints and the zero-one constraint

    4. `metadata::Metadata{T,S<:MOI.AbstractSet}`: a constraint cannot be
       satisfied based on the variable bounds. `constraints` contains the
       offending constraint and all associated variable bounds.
"""
struct IIS{M}
    constraints::Vector{MOI.ConstraintIndex}
    maybe_constraints::Vector{MOI.ConstraintIndex}
    metadata::M

    function IIS(
        constraints::Vector{MOI.ConstraintIndex};
        maybe_constraints::Vector{MOI.ConstraintIndex} = MOI.ConstraintIndex[],
        metadata::M = nothing,
    ) where {M<:Union{Nothing,Metadata}}
        return new{M}(constraints, maybe_constraints, metadata)
    end
end

"""
    Optimizer()

Create a new optimizer object that supports `MOI.compute_conflict!`.

## Attributes

Before calling `MOI.compute_conflict!`, you must set the following two
attributes:

 * `MathOptIIS.InfeasibleModel`
 * `MathOptIIS.InnerOptimizer`

There are also two optional attributes that control the behavior of the
algorithm:

 * `MOI.TimeLimitSec`
 * `MOI.Silent`

After calling `MOI.compute_conflict!`, access the IIS (if present) using the
following attributes:

 * `MOI.ConflictCount`
 * `MOI.ConflictStatus`
 * `MOI.ConstraintConflictStatus`
 * `MathOptIIS.ListOfConstraintIndicesInConflict`

## Advanced

Some applications may need to directly interact with the result objects that are
stored in the `.results::Vector{IIS}` field. There is one `IIS` for each
conflict. See the `IIS` docstring for more details.

## Embedding the Optimizer in another package

MathOptIIS is not intended to be called directly from user-code. Instead, it
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
    # Optional: set the solver's silent to true or false. In this example,
    # follow the outer attribute.
    MOI.set(solver, MOI.Silent(), MOI.get(model, MOI.Silent()))
    # Optional: set a time limit. In this case, 60 seconds.
    MOI.set(solver, MOI.TimeLimitSec(), 60.0)
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

function MOI.get(optimizer::Optimizer, attr::MOI.ConflictCount)
    if optimizer.conflict_solver === nothing
        return 0
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
"""
mutable struct Optimizer <: MOI.AbstractOptimizer
    infeasible_model::Union{MOI.ModelLike,Nothing}
    inner_optimizer::Any
    # iis attributes
    time_limit::Float64
    verbose::Bool
    # result data
    start_time::Float64
    status::MOI.ConflictStatusCode
    results::Vector{IIS}

    function Optimizer()
        return new(
            nothing,
            nothing,
            Inf,
            false,
            NaN,
            MOI.COMPUTE_CONFLICT_NOT_CALLED,
            IIS[],
        )
    end
end

function MOI.empty!(model::Optimizer)
    model.infeasible_model = nothing
    model.start_time = NaN
    model.status = MOI.COMPUTE_CONFLICT_NOT_CALLED
    empty!(model.results)
    return
end

# MathOptIIS.InfeasibleModel

"""
    InfeasibleModel() <: MOI.AbstractModelAttribute

A model attribute for passing the infeasible model to the IIS solver.

## Example

```julia
import MathOptIIS
solver = MathOptIIS.Optimizer()
MOI.set(solver, MathOptIIS.InfeasibleModel(), model)
MOI.set(solver, MathOptIIS.InnerOptimizer(), Optimizer)
MOI.compute_conflict!(solver)
```
"""
struct InfeasibleModel <: MOI.AbstractModelAttribute end

function MOI.set(optimizer::Optimizer, ::InfeasibleModel, model::MOI.ModelLike)
    optimizer.infeasible_model = model
    empty!(optimizer.results)
    optimizer.status = MOI.COMPUTE_CONFLICT_NOT_CALLED
    return
end

# MathOptIIS.InnerOptimizer

"""
    InnerOptimizer() <: MOI.AbstractOptimizerAttribute

A optimizer attribute for passing the inner optimizer to the IIS solver.

## Example

```julia
import MathOptIIS
solver = MathOptIIS.Optimizer()
MOI.set(solver, MathOptIIS.InfeasibleModel(), model)
MOI.set(solver, MathOptIIS.InnerOptimizer(), Optimizer)
MOI.compute_conflict!(solver)
```
"""
struct InnerOptimizer <: MOI.AbstractOptimizerAttribute end

function MOI.set(optimizer::Optimizer, ::InnerOptimizer, inner_optimizer::Any)
    optimizer.inner_optimizer = inner_optimizer
    return
end

# MOI.TimeLimitSec

function MOI.set(optimizer::Optimizer, ::MOI.TimeLimitSec, value::Float64)
    optimizer.time_limit = value
    return
end

MOI.get(optimizer::Optimizer, ::MOI.TimeLimitSec) = optimizer.time_limit

# MOI.Silent

function MOI.set(optimizer::Optimizer, ::MOI.Silent, value::Bool)
    optimizer.verbose = !value
    return
end

MOI.get(optimizer::Optimizer, ::MOI.Silent) = !optimizer.verbose

# MOI.ConflictStatus

MOI.get(optimizer::Optimizer, ::MOI.ConflictStatus) = optimizer.status

# MOI.ConflictCount

MOI.get(optimizer::Optimizer, ::MOI.ConflictCount) = length(optimizer.results)

# MOI.ConstraintConflictStatus

function MOI.get(
    optimizer::Optimizer,
    attr::MOI.ConstraintConflictStatus,
    con::MOI.ConstraintIndex,
)
    if !(1 <= attr.conflict_index <= length(optimizer.results))
        return MOI.NOT_IN_CONFLICT
    elseif con in optimizer.results[attr.conflict_index].constraints
        return MOI.IN_CONFLICT
    elseif con in optimizer.results[attr.conflict_index].maybe_constraints
        return MOI.MAYBE_IN_CONFLICT
    end
    return MOI.NOT_IN_CONFLICT
end

# MathOptIIS.ListOfConstraintIndicesInConflict

"""
    ListOfConstraintIndicesInConflict(conflict_index = 1)

An `MOI.AbstractModelAttribute` for querying the list of constraints that appear
in the IIS at index `conflict_index`.

The return value is a `Vector{MOI.ConstraintIndex}`. Note how this is not
type-stable.
"""
struct ListOfConstraintIndicesInConflict <: MOI.AbstractModelAttribute
    conflict_index::Int

    function ListOfConstraintIndicesInConflict(conflict_index::Integer = 1)
        return new(conflict_index)
    end
end

function MOI.get(optimizer::Optimizer, attr::ListOfConstraintIndicesInConflict)
    if !(1 <= attr.conflict_index <= length(optimizer.results))
        return MOI.ConstraintIndex[]
    end
    return vcat(
        optimizer.results[attr.conflict_index].constraints,
        optimizer.results[attr.conflict_index].maybe_constraints,
    )
end

# MOI.compute_conflict!

function _check_interrupt(f::F) where {F}
    try
        return reenable_sigint(f)
    catch ex
        if !(ex isa InterruptException)
            rethrow(ex)
        end
        return true
    end
end

function _check_premature_termination(optimizer::Optimizer)
    return _check_interrupt() do
        return time() >= optimizer.start_time + optimizer.time_limit
    end
end

function _update_time_limit(optimizer::Optimizer, model::MOI.ModelLike)
    time_remaining = optimizer.start_time + optimizer.time_limit - time()
    if isfinite(time_remaining) && MOI.supports(model, MOI.TimeLimitSec())
        MOI.set(model, MOI.TimeLimitSec(), time_remaining)
    end
    return
end

function _optimize!(optimizer::Optimizer, model::MOI.ModelLike)
    _update_time_limit(optimizer, model)
    MOI.optimize!(model)
    return
end

function MOI.compute_conflict!(optimizer::Optimizer)
    disable_sigint() do
        return _compute_conflict!(
            optimizer,
            optimizer.inner_optimizer,
            optimizer.infeasible_model,
        )
    end
    return
end

function _compute_conflict!(
    optimizer::Optimizer,
    inner_optimizer::Any,
    infeasible_model::MOI.ModelLike,
)
    optimizer.status = MOI.NO_CONFLICT_FOUND
    empty!(optimizer.results)
    optimizer.start_time = time()
    if optimizer.verbose
        println("[MathOptIIS] starting compute_conflict!")
    end
    if _feasibility_check(optimizer, infeasible_model)
        optimizer.status = MOI.NO_CONFLICT_EXISTS
        return optimizer.results
    end
    # Step 1: check for inconsistent variable bounds
    variable_info = _bound_infeasibility!(optimizer, infeasible_model, Float64)
    if !isempty(optimizer.results)
        optimizer.status = MOI.CONFLICT_FOUND
    end
    if _check_premature_termination(optimizer)
        return
    end
    # Step 2: check for inconsistent constraints based on variable bounds
    _range_infeasibility!(optimizer, infeasible_model, variable_info)
    if !isempty(optimizer.results)
        optimizer.status = MOI.CONFLICT_FOUND
        # Now it's safe to return: there are some trivial things for the user to
        # fix before we attempt the elastic filter
        return
    end
    if _check_premature_termination(optimizer)
        return
    end
    # Step 3: elastic filter
    _elastic_filter(optimizer, inner_optimizer, infeasible_model, variable_info)
    if !isempty(optimizer.results)
        optimizer.status = MOI.CONFLICT_FOUND
    end
    if optimizer.verbose
        println(
            "[MathOptIIS]   elastic filter found $(length(optimizer.results)) infeasible subsets",
        )
    end
    return
end

# ==============================================================================
# Step 0: feasibility check
# ==============================================================================

function _feasibility_check(
    optimizer::Optimizer,
    infeasible_model::MOI.ModelLike,
)
    termination_status = MOI.get(infeasible_model, MOI.TerminationStatus())
    if optimizer.verbose
        println(
            "[MathOptIIS]   model termination status: $(termination_status)",
        )
    end
    if termination_status in
       (MOI.OTHER_ERROR, MOI.INVALID_MODEL, MOI.OPTIMIZE_NOT_CALLED)
        return false # because we can't assert it is feasible
    end
    primal_status = MOI.get(infeasible_model, MOI.PrimalStatus())
    if optimizer.verbose
        println("[MathOptIIS]   model primal status: $(primal_status)")
    end
    if primal_status in (MOI.FEASIBLE_POINT, MOI.NEARLY_FEASIBLE_POINT) && !(
        termination_status in
        (MOI.INFEASIBLE, MOI.ALMOST_INFEASIBLE, MOI.LOCALLY_INFEASIBLE)
    )
        return true
    end
    return false
end

# ==============================================================================
# Step 1: check variable bounds and integrality restrictions for violations
# ==============================================================================

mutable struct _VariableInfo{T}
    lower::Union{Nothing,MOI.AbstractScalarSet}
    upper::Union{Nothing,MOI.AbstractScalarSet}
    integer::Bool
    zero_one::Bool

    _VariableInfo{T}() where {T} = new{T}(nothing, nothing, false, false)
end

_update_info!(info::_VariableInfo, s::MOI.LessThan) = (info.upper = s)

_update_info!(info::_VariableInfo, s::MOI.GreaterThan) = (info.lower = s)

function _update_info!(info::_VariableInfo, s::Union{MOI.EqualTo,MOI.Interval})
    info.lower = info.upper = s
    return
end

_update_info!(info::_VariableInfo, ::MOI.Integer) = (info.integer = true)

_update_info!(info::_VariableInfo, ::MOI.ZeroOne) = (info.zero_one = true)

function _update_info!(
    info::Dict{MOI.VariableIndex,<:_VariableInfo},
    model::MOI.ModelLike,
    ::Type{S},
) where {S<:MOI.AbstractScalarSet}
    for ci in MOI.get(model, MOI.ListOfConstraintIndices{MOI.VariableIndex,S}())
        f = MOI.get(model, MOI.ConstraintFunction(), ci)
        s = MOI.get(model, MOI.ConstraintSet(), ci)
        _update_info!(info[f], s)
    end
    return
end

function _ci(x::MOI.VariableIndex, ::S) where {S<:MOI.AbstractScalarSet}
    return MOI.ConstraintIndex{MOI.VariableIndex,S}(x.value)
end

_lower(::Type{T}, ::Nothing) where {T} = typemin(T)
_lower(::Type{T}, s::Union{MOI.GreaterThan,MOI.Interval}) where {T} = s.lower
_lower(::Type{T}, s::MOI.EqualTo) where {T} = s.value

_upper(::Type{T}, ::Nothing) where {T} = typemax(T)
_upper(::Type{T}, s::Union{MOI.LessThan,MOI.Interval}) where {T} = s.upper
_upper(::Type{T}, s::MOI.EqualTo) where {T} = s.value

function _check_conflict(x::MOI.VariableIndex, info::_VariableInfo{T}) where {T}
    lb, ub = _lower(T, info.lower), _upper(T, info.upper)
    if ub < lb
        return IIS(
            MOI.ConstraintIndex[_ci(x, info.lower), _ci(x, info.upper)];
            metadata = Metadata(lb, ub, nothing),
        )
    elseif info.integer
        if abs(ub - lb) < 1 && ceil(Int, lb) > floor(Int, ub)
            con = MOI.ConstraintIndex{MOI.VariableIndex,MOI.Integer}(x.value)
            c = MOI.ConstraintIndex[con, _ci(x, info.lower), _ci(x, info.upper)]
            return IIS(c; metadata = Metadata(lb, ub, MOI.Integer()))
        end
    elseif info.zero_one
        con = MOI.ConstraintIndex{MOI.VariableIndex,MOI.ZeroOne}(x.value)
        if 0 < lb && ub < 1
            c = MOI.ConstraintIndex[con, _ci(x, info.lower), _ci(x, info.upper)]
            return IIS(c; metadata = Metadata(lb, ub, MOI.ZeroOne()))
        elseif 1 < lb
            c = MOI.ConstraintIndex[con, _ci(x, info.lower)]
            return IIS(c; metadata = Metadata(lb, typemax(T), MOI.ZeroOne()))
        elseif ub < 0
            c = MOI.ConstraintIndex[con, _ci(x, info.upper)]
            return IIS(c; metadata = Metadata(typemin(T), ub, MOI.ZeroOne()))
        end
    end
    return
end

function _bound_infeasibility!(
    optimizer::Optimizer,
    infeasible_model::MOI.ModelLike,
    ::Type{T},
) where {T}
    if optimizer.verbose
        println("[MathOptIIS] starting bound analysis")
    end
    variable_info = Dict(
        x => _VariableInfo{T}() for
        x in MOI.get(infeasible_model, MOI.ListOfVariableIndices())
    )
    _update_info!(variable_info, infeasible_model, MOI.LessThan{T})
    _update_info!(variable_info, infeasible_model, MOI.GreaterThan{T})
    _update_info!(variable_info, infeasible_model, MOI.EqualTo{T})
    _update_info!(variable_info, infeasible_model, MOI.Interval{T})
    _update_info!(variable_info, infeasible_model, MOI.Integer)
    _update_info!(variable_info, infeasible_model, MOI.ZeroOne)
    results = IIS[]
    for (x, info) in variable_info
        if (conflict = _check_conflict(x, info)) !== nothing
            push!(results, conflict)
        end
    end
    append!(optimizer.results, results)
    if optimizer.verbose
        println(
            "[MathOptIIS]   bound analysis found $(length(results)) infeasible subsets",
        )
    end
    return variable_info
end

# ==============================================================================
# Step 2: propagate variable bounds through functions to detect obvious errors
# ==============================================================================

_supports_interval(::Type{T}) where {T} = false

function _range_infeasibility!(
    optimizer::Optimizer,
    infeasible_model::MOI.ModelLike,
    variable_info::Dict{MOI.VariableIndex,_VariableInfo{T}},
) where {T}
    if optimizer.verbose
        println("[MathOptIIS] starting range analysis")
    end
    variables = Dict{MOI.VariableIndex,_Interval{T}}()
    for (x, info) in variable_info
        # It may be the case that lo > hi. In which case, the bound analysis
        # will have already flagged the issue, but we might learn something from
        # the range propagation.
        lo, hi = _lower(T, info.lower), _upper(T, info.upper)
        variables[x] = _Interval(lo, max(lo, hi))
    end
    results = IIS[]
    for (F, S) in MOI.get(infeasible_model, MOI.ListOfConstraintTypesPresent())
        if _supports_interval(F) && _supports_interval(S)
            _range_infeasibility!(
                optimizer,
                infeasible_model,
                variable_info,
                variables,
                F,
                S,
                results,
            )
        end
    end
    append!(optimizer.results, results)
    if optimizer.verbose
        println(
            "[MathOptIIS]   range analysis found $(length(results)) infeasible subsets",
        )
    end
    return
end

function _range_infeasibility!(
    optimizer::Optimizer,
    infeasible_model::MOI.ModelLike,
    variable_info::Dict{MOI.VariableIndex,_VariableInfo{T}},
    variables::Dict{MOI.VariableIndex,_Interval{T}},
    ::Type{F},
    ::Type{S},
    results::Vector{IIS},
) where {T,F,S}
    if optimizer.verbose
        println(
            "[MathOptIIS]   analyzing ",
            sprint(MOI.Utilities.print_with_acronym, "$F -in- $S"),
        )
    end
    for con in MOI.get(infeasible_model, MOI.ListOfConstraintIndices{F,S}())
        if _check_premature_termination(optimizer)
            return
        end
        func = MOI.get(infeasible_model, MOI.ConstraintFunction(), con)
        cons = Set{MOI.ConstraintIndex}()
        interval = _compute_interval(variables, func, variable_info, cons)
        set = MOI.get(infeasible_model, MOI.ConstraintSet(), con)::S
        if !_valid_range(set, interval)
            push!(cons, con)
            metadata = Metadata(interval.lo, interval.hi, set)
            push!(results, IIS(collect(cons); metadata))
        end
    end
    return
end

_supports_interval(::Type{MOI.ScalarAffineFunction{T}}) where {T} = true

function _compute_interval(
    variables::Dict{MOI.VariableIndex,_Interval{T}},
    f::MOI.ScalarAffineFunction,
    variable_info::Dict{MOI.VariableIndex,_VariableInfo{T}},
    cons::Set{MOI.ConstraintIndex},
) where {T}
    out = _Interval(f.constant, f.constant)
    for t in f.terms
        out += t.coefficient * variables[t.variable]
        if (s = variable_info[t.variable].lower) !== nothing
            push!(cons, _ci(t.variable, s))
        end
        if (s = variable_info[t.variable].upper) !== nothing
            push!(cons, _ci(t.variable, s))
        end
    end
    return out
end

_supports_interval(::Type{MOI.EqualTo{T}}) where {T} = true

_valid_range(set::MOI.EqualTo, x::_Interval) = x.lo <= set.value <= x.hi

_supports_interval(::Type{MOI.LessThan{T}}) where {T} = true

_valid_range(set::MOI.LessThan, x::_Interval) = set.upper >= x.lo

_supports_interval(::Type{MOI.GreaterThan{T}}) where {T} = true

_valid_range(set::MOI.GreaterThan, x::_Interval) = x.hi >= set.lower

_supports_interval(::Type{MOI.Interval{T}}) where {T} = true

function _valid_range(set::MOI.Interval, x::_Interval)
    return set.upper >= x.lo && set.lower <= x.hi
end

# ==============================================================================
# Step 3: compute a proper IIS using an elastic filter
# ==============================================================================

function _fix_slack(
    model::MOI.ModelLike,
    func::MOI.ScalarAffineFunction{T},
) where {T}
    for term in func.terms
        MOI.add_constraint(model, term.variable, MOI.LessThan(zero(T)))
    end
    return
end

function _unfix_slack(
    model::MOI.ModelLike,
    func::MOI.ScalarAffineFunction{T},
) where {T}
    for term in func.terms
        x = term.variable
        ci = MOI.ConstraintIndex{MOI.VariableIndex,MOI.LessThan{T}}(x.value)
        @assert MOI.is_valid(model, ci)
        MOI.delete(model, ci)
    end
    return
end

function _instantiate(optimizer::F, ::Type{T}) where {F,T}
    cache = MOI.Utilities.CachingOptimizer(
        MOI.Utilities.UniversalFallback(MOI.Utilities.Model{T}()),
        MOI.instantiate(optimizer),
    )
    return MOI.Bridges.full_bridge_optimizer(cache, T)
end

function _elastic_filter(
    optimizer::Optimizer,
    inner_optimizer::Any,
    infeasible_model::MOI.ModelLike,
    variable_info::Dict{MOI.VariableIndex,_VariableInfo{T}},
) where {T}
    if optimizer.verbose
        println("[MathOptIIS] starting elastic filter")
    end
    if inner_optimizer === nothing
        println(
            "[MathOptIIS] elastic filter cannot continue because no optimizer was provided",
        )
        return
    end
    # We instantiate with a cache, even if the solver supports the incremental
    # interface, because we don't know if the `inner_optimizer` supports all of
    # the modifications we're going to do here.
    model = _instantiate(inner_optimizer, T)
    if MOI.supports(model, MOI.Silent())
        MOI.set(model, MOI.Silent(), true)
    end
    index_map = MOI.copy_to(model, infeasible_model)
    # ==========================================================================
    # Step 1: relax integrality and solve.
    # ==========================================================================
    if optimizer.verbose
        println("[MathOptIIS]   testing if we can relax integrality")
    end
    relax_info = _relax_integrality(model, variable_info)
    # It's a bit wasteful to re-solve this problem when we know it is
    # infeasible, but:
    #  * we might have relaxed the integrality
    #  * we're going to be modifying and re-solving anyway, so it doesn't hurt
    #    to cache some work. It's only one extra solve.
    _optimize!(optimizer, model)
    if _is_feasible(model)
        if optimizer.verbose
            println("[MathOptIIS]     integrality is required")
        end
        # The relaxed problem is feasible. This is significantly more difficult
        # to deal with because we need to add back in the integrality
        # restrictions.
        for x in relax_info.integer
            MOI.add_constraint(model, x, MOI.Integer())
        end
        for (ci, x) in relax_info.binary
            MOI.delete(model, ci)
            MOI.add_constraint(model, x, MOI.ZeroOne())
        end
    elseif optimizer.verbose
        println("[MathOptIIS]     integrality is not required")
    end
    # ==========================================================================
    # Step 2: additive filter. Construct a superset of the IIS
    # ==========================================================================
    dual_certificate = MOI.ConstraintIndex[]
    if MOI.get(model, MOI.DualStatus()) == MOI.INFEASIBILITY_CERTIFICATE
        # If there is an infeasibility certificate, the non-zero rows are a
        # superset of the IIS
        _dual_certificate!(dual_certificate, model, relax_info)
    end
    # ==========================================================================
    # Step 3: relax constraints to build the penalized problem
    # ==========================================================================
    if optimizer.verbose
        println("[MathOptIIS]   constructing the penalty relaxation")
    end
    constraint_to_affine = MOI.modify(
        model,
        MOI.Utilities.PenaltyRelaxation(; default = one(T), warn = false),
    )
    slack_obj = zero(MOI.ScalarAffineFunction{T})
    for v in values(constraint_to_affine)
        MOI.Utilities.operate!(+, T, slack_obj, v)
    end
    MOI.set(model, MOI.ObjectiveSense(), MOI.MIN_SENSE)
    MOI.set(model, MOI.ObjectiveFunction{typeof(slack_obj)}(), slack_obj)
    iis_candidate = Set{MOI.ConstraintIndex}()
    # In the following solves we never actually need a proof of global
    # optimality. We care only about feasibility. I assume that solvers employ a
    # stopping heuristic here: better solutions mean a smaller candidate set,
    # but come at the expense of increased solve time.
    if MOI.supports(model, MOI.ObjectiveLimit())
        # As one attempt, set an objective limit. In practice this needs tuning.
        MOI.set(model, MOI.ObjectiveLimit(), 2.1)
    end
    # ==========================================================================
    # Step 4: if we didn't find a certificate, build a super set via the
    # additive method.
    # ==========================================================================
    if !isempty(dual_certificate)
        if optimizer.verbose
            println(
                "[MathOptIIS]   using INFEASIBILITY_CERTIFICATE to construct candidate set",
            )
        end
        for ci in dual_certificate
            if (slack = get(constraint_to_affine, ci, nothing)) !== nothing
                push!(iis_candidate, ci)
                _fix_slack(model, slack)
            end
        end
        if optimizer.verbose
            println(
                "[MathOptIIS]     size of the candidate set: $(length(iis_candidate))",
            )
        end
    else
        if optimizer.verbose
            println("[MathOptIIS]   starting the additive method")
        end
        constraints_to_check = Set(keys(constraint_to_affine))
        constraints_to_fix = Set{MOI.ConstraintIndex}()
        while !isempty(constraints_to_check)
            if _check_premature_termination(optimizer)
                if optimizer.verbose
                    println("[MathOptIIS]     early termination requested")
                end
                return nothing
            end
            _optimize!(optimizer, model)
            if !_is_feasible(model)
                break
            end
            for con in constraints_to_check
                func = constraint_to_affine[con]
                for t in func.terms
                    if MOI.get(model, MOI.VariablePrimal(), t.variable) > 0
                        push!(constraints_to_fix, con)
                        break
                    end
                end
            end
            for con in constraints_to_fix
                _fix_slack(model, constraint_to_affine[con])
                push!(iis_candidate, con)
                delete!(constraints_to_check, con)
            end
            empty!(constraints_to_fix)
            if optimizer.verbose
                println(
                    "[MathOptIIS]     size of the candidate set: $(length(iis_candidate))",
                )
            end
        end
    end
    # ==========================================================================
    # Step 5: a deletion filter
    # ==========================================================================
    if optimizer.verbose
        println("[MathOptIIS]   starting the deletion filter")
    end
    # We maintain two sets of constraints:
    #  * iis_in_conflict is the set of constraints we have proven are in the
    #    conflict
    #  * iis_maybe_in_conflict is the set of constraints that were in the
    #    candidate set and we could not rule out. This set is filled only if we
    #    terminate the deletion filter before finishing.
    iis_in_conflict = Set{MOI.ConstraintIndex}()
    iis_maybe_in_conflict = Set{MOI.ConstraintIndex}()
    candidate_size = length(iis_candidate)
    for subset in Iterators.partition(iis_candidate, 10)
        if _check_premature_termination(optimizer)
            union!(iis_maybe_in_conflict, subset)
            continue
        end
        iis_subset = _iterative_deletion_filter(
            optimizer,
            model,
            constraint_to_affine,
            subset,
        )
        union!(iis_in_conflict, iis_subset)
        if optimizer.verbose
            candidate_size -= length(subset) - length(iis_subset)
            println(
                "[MathOptIIS]     size of the candidate set: $candidate_size",
            )
        end
    end
    # ==========================================================================
    # Step 6: clean up and get out of here
    # ==========================================================================
    new_to_old_index_map = _reverse(index_map)
    # First, we need to map constraints from `model` back to the original
    # `optimizer`. There's the added complication that we may have replaced a
    # `x in {0, 1}` by `1.0 * x in [0, 1]`:
    iis = MOI.ConstraintIndex[]
    for c in iis_in_conflict
        if (x = get(relax_info.binary, c, nothing)) !== nothing
            c = _ci(x, MOI.ZeroOne())
        end
        push!(iis, new_to_old_index_map[c])
    end
    maybe_constraints = MOI.ConstraintIndex[]
    for c in iis_maybe_in_conflict
        if (x = get(relax_info.binary, c, nothing)) !== nothing
            c = _ci(x, MOI.ZeroOne())
        end
        push!(maybe_constraints, new_to_old_index_map[c])
    end
    # Next, we need to add all of the constraints that _might_ be in the IIS and
    # which we didn't test because they couldn't be relaxed. If there was a dual
    # certificate, this is easy.
    if !isempty(dual_certificate)
        for ci in dual_certificate
            # No need to check `relax_info.binary` here because it can't be part
            # of the infeasibility certificate.
            if !haskey(constraint_to_affine, ci)
                push!(maybe_constraints, new_to_old_index_map[ci])
            end
        end
        push!(optimizer.results, IIS(iis; maybe_constraints))
        return
    end
    # If there wasn't a dual certificate, we need to add all constraints that
    # weren't relaxed, and we also need to add any variable bounds of variables
    # that appear in the constraints.
    #
    # First loop: get all non-relaxed constraints that are not variable bounds.
    for (F, S) in MOI.get(infeasible_model, MOI.ListOfConstraintTypesPresent())
        if F == MOI.VariableIndex
            continue
        end
        for ci in MOI.get(infeasible_model, MOI.ListOfConstraintIndices{F,S}())
            if haskey(constraint_to_affine, index_map[ci])
                break # If this constraint is, all other F-in-S are too
            end
            push!(maybe_constraints, ci)
        end
    end
    # Now, get all variables that appear in the constraints.
    variables = Set{MOI.VariableIndex}()
    for c in [iis; maybe_constraints]
        f = MOI.get(infeasible_model, MOI.ConstraintFunction(), c)
        _get_variables!(variables, f)
    end
    # Second loop: get all variable constraints as they appear.
    for (F, S) in MOI.get(infeasible_model, MOI.ListOfConstraintTypesPresent())
        if !(F <: MOI.VariableIndex)
            continue
        end
        for ci in MOI.get(infeasible_model, MOI.ListOfConstraintIndices{F,S}())
            x = MOI.get(infeasible_model, MOI.ConstraintFunction(), ci)
            if x in variables
                push!(maybe_constraints, ci)
            end
        end
    end
    push!(optimizer.results, IIS(iis; maybe_constraints))
    return
end

function _reverse(index_map::MOI.IndexMap)
    ret = MOI.IndexMap()
    for (k, v) in index_map.var_map
        ret[v] = k
    end
    for (k, v) in index_map.con_map
        ret[v] = k
    end
    return ret
end

function _iterative_deletion_filter(
    optimizer::Optimizer,
    model::MOI.ModelLike,
    constraint_to_affine::Dict{MOI.ConstraintIndex,MOI.ScalarAffineFunction{T}},
    candidates::Vector{MOI.ConstraintIndex},
) where {T}
    # Optimization: unfix all and see if we need to iterate-through one-by-one
    for con in candidates
        _unfix_slack(model, constraint_to_affine[con])
    end
    _optimize!(optimizer, model)
    if !_is_feasible(model)
        return Set{MOI.ConstraintIndex}()
    end
    for con in candidates
        _fix_slack(model, constraint_to_affine[con])
    end
    # Here's the iterative part
    ret = Set(candidates)
    for con in candidates
        if _check_premature_termination(optimizer)
            break
        end
        _unfix_slack(model, constraint_to_affine[con])
        _optimize!(optimizer, model)
        if _is_feasible(model)
            _fix_slack(model, constraint_to_affine[con])
        else
            delete!(ret, con)
        end
    end
    return ret
end

function _is_feasible(model::MOI.ModelLike)
    return MOI.get(model, MOI.PrimalStatus()) in
           (MOI.FEASIBLE_POINT, MOI.NEARLY_FEASIBLE_POINT)
end

function _dual_certificate!(
    dual_certificate::Vector{MOI.ConstraintIndex},
    model::MOI.ModelLike,
    relax_info,
)
    for (F, S) in MOI.get(model, MOI.ListOfConstraintTypesPresent())
        for ci in MOI.get(model, MOI.ListOfConstraintIndices{F,S}())
            if !iszero(MOI.get(model, MOI.ConstraintDual(), ci))
                push!(dual_certificate, ci)
            end
        end
    end
    return
end

function _relax_integrality(
    model::MOI.ModelLike,
    variable_info::Dict{MOI.VariableIndex,_VariableInfo{T}},
) where {T}
    F = MOI.VariableIndex
    ret = (;
        integer = MOI.VariableIndex[],
        binary = Dict{
            MOI.ConstraintIndex{MOI.ScalarAffineFunction{T},MOI.Interval{T}},
            MOI.VariableIndex,
        }(),
    )
    for (x, info) in variable_info
        if info.integer
            MOI.delete(model, MOI.ConstraintIndex{F,MOI.Integer}(x.value))
            push!(ret.integer, x)
        elseif info.zero_one
            MOI.delete(model, MOI.ConstraintIndex{F,MOI.ZeroOne}(x.value))
            zero_one_set = MOI.Interval(zero(T), one(T))
            ci = MOI.add_constraint(model, one(T) * x, zero_one_set)
            ret.binary[ci] = x
        end
    end
    return ret
end

_get_variables!(::Set{MOI.VariableIndex}, ::Number) = nothing

function _get_variables!(x::Set{MOI.VariableIndex}, f::MOI.VariableIndex)
    push!(x, f)
    return
end

function _get_variables!(x::Set{MOI.VariableIndex}, f::MOI.ScalarAffineTerm)
    push!(x, f.variable)
    return
end

function _get_variables!(x::Set{MOI.VariableIndex}, f::MOI.ScalarAffineFunction)
    map(Base.Fix1(_get_variables!, x), f.terms)
    return
end

function _get_variables!(x::Set{MOI.VariableIndex}, f::MOI.ScalarQuadraticTerm)
    push!(x, f.variable_1)
    push!(x, f.variable_2)
    return
end

function _get_variables!(
    x::Set{MOI.VariableIndex},
    f::MOI.ScalarQuadraticFunction,
)
    map(Base.Fix1(_get_variables!, x), f.affine_terms)
    map(Base.Fix1(_get_variables!, x), f.quadratic_terms)
    return
end

function _get_variables!(
    x::Set{MOI.VariableIndex},
    f::MOI.ScalarNonlinearFunction,
)
    stack = Any[f]
    while !isempty(stack)
        arg = pop!(stack)
        if arg isa MOI.ScalarNonlinearFunction
            for a in arg.args
                push!(stack, a)
            end
        else
            _get_variables!(x, arg)
        end
    end
    return
end

# This function currently doesn't get called directly because MOI doesn't
# support relaxing vector functions. However, @odow plans to, so this is a
# future safeguard. For now, it gets tested explicitly in the tests.
function _get_variables!(
    x::Set{MOI.VariableIndex},
    f::MOI.AbstractVectorFunction,
)
    for g in MOI.Utilities.eachscalar(f)
        _get_variables!(x, g)
    end
    return
end

end # module MathOptIIS

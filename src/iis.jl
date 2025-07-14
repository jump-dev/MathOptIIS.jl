# Copyright (c) 2025: Joaquim Dias Garcia, Oscar Dowson and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

abstract type AbstractAdditionalData end

struct InfeasibilityData
    # IIS constraints set
    constraints::Vector{MOI.ConstraintIndex}
    # variable-set constraints only for NoData IIS (from the iis solver)
    # this will be an empty vector for most types of IIS
    maybe_constraints::Vector{MOI.ConstraintIndex}
    # indicates if the IIS is irreducible
    irreducible::Bool
    # additional data
    metadata::AbstractAdditionalData
    function InfeasibilityData(
        constraints::Vector{<:MOI.ConstraintIndex},
        irreducible::Bool,
        metadata::AbstractAdditionalData;
        maybe_constraints::Vector{MOI.ConstraintIndex} = MOI.ConstraintIndex[],
    )
        return new(constraints, maybe_constraints, irreducible, metadata)
    end
end

struct BoundsData <: AbstractAdditionalData
    lower_bound::Float64
    upper_bound::Float64
end

struct IntegralityData <: AbstractAdditionalData
    lower_bound::Float64
    upper_bound::Float64
    set::Union{MOI.Integer,MOI.ZeroOne}#, MOI.Semicontinuous{T}, MOI.Semiinteger{T}}
end

struct RangeData <: AbstractAdditionalData
    lower_bound::Float64
    upper_bound::Float64
    set::Union{<:MOI.EqualTo,<:MOI.LessThan,<:MOI.GreaterThan}
end

struct NoData <: AbstractAdditionalData end

Base.@kwdef mutable struct Optimizer
    original_model::Union{MOI.ModelLike,Nothing} = nothing
    #
    # iterative solver data
    optimizer::Any = nothing # MOI.ModelLike
    optimizer_attributes::Dict{Any,Any} = Dict{Any,Any}()
    #
    # iis attributes
    time_limit::Float64 = Inf
    verbose::Bool = false
    skip_feasibility_check::Bool = false
    stop_if_infeasible_bounds::Bool = true
    stop_if_infeasible_ranges::Bool = true
    deletion_filter::Bool = true
    elastic_filter_tolerance::Float64 = 1e-5
    ignore_integrality::Bool = false
    #
    # result data
    start_time::Float64 = NaN
    status::MOI.ConflictStatusCode = MOI.COMPUTE_CONFLICT_NOT_CALLED
    results::Vector{InfeasibilityData} = InfeasibilityData[]
end

struct InfeasibleModel end

function MOI.set(
    optimizer::Optimizer,
    attr::InfeasibleModel,
    model::MOI.ModelLike,
)
    optimizer.original_model = model
    # this also resets the results
    results = InfeasibilityData[]
    status = MOI.COMPUTE_CONFLICT_NOT_CALLED
    return
end

# function MOI.get(optimizer::Optimizer, attr::InfeasibleModel)
#     return optimizer.original_model
# end

struct InnerOptimizer end

function MOI.set(optimizer::Optimizer, attr::InnerOptimizer, solver)
    optimizer.optimizer = solver
    return
end

# function MOI.get(optimizer::Optimizer, attr::InnerOptimizer)
#     return optimizer.optimizer
# end

struct InnerOptimizerAttribute
    attr::MOI.AbstractOptimizerAttribute
end

function MOI.set(optimizer::Optimizer, attr::InnerOptimizerAttribute, value)
    optimizer.optimizer_attributes[attr.attr] = value
    return
end

# function MOI.get(
#     optimizer::Optimizer,
#     attr::InnerOptimizerAttribute,
# )
#     return optimizer.optimizer_attributes[attr.attr]
# end

function MOI.set(optimizer::Optimizer, attr::MOI.TimeLimitSec, value::Float64)
    optimizer.time_limit = value
    return
end

function MOI.get(optimizer::Optimizer, attr::MOI.TimeLimitSec)
    return optimizer.time_limit
end

function MOI.set(optimizer::Optimizer, attr::MOI.Silent, value::Bool)
    optimizer.verbose = value
    return
end

function MOI.get(optimizer::Optimizer, attr::MOI.Silent)
    return optimizer.verbose
end

struct SkipFeasibilityCheck <: MOI.AbstractOptimizerAttribute end

function MOI.set(optimizer::Optimizer, attr::SkipFeasibilityCheck, value::Bool)
    optimizer.skip_feasibility_check = value
    return
end

function MOI.get(optimizer::Optimizer, attr::SkipFeasibilityCheck)
    return optimizer.skip_feasibility_check
end

struct StopIfInfeasibleBounds end

function MOI.set(
    optimizer::Optimizer,
    attr::StopIfInfeasibleBounds,
    value::Bool,
)
    optimizer.stop_if_infeasible_bounds = value
    return
end

function MOI.get(optimizer::Optimizer, attr::StopIfInfeasibleBounds)
    return optimizer.stop_if_infeasible_bounds
end

struct StopIfInfeasibleRanges end

function MOI.set(
    optimizer::Optimizer,
    attr::StopIfInfeasibleRanges,
    value::Bool,
)
    optimizer.stop_if_infeasible_ranges = value
    return
end

function MOI.get(optimizer::Optimizer, attr::StopIfInfeasibleRanges)
    return optimizer.stop_if_infeasible_ranges
end

struct DeletionFilter end

function MOI.set(optimizer::Optimizer, attr::DeletionFilter, value::Bool)
    optimizer.deletion_filter = value
    return
end

function MOI.get(optimizer::Optimizer, attr::DeletionFilter)
    return optimizer.deletion_filter
end

struct ElasticFilterTolerance end

function MOI.set(optimizer::Optimizer, ::ElasticFilterTolerance, value::Float64)
    optimizer.elastic_filter_tolerance = value
    return
end

function MOI.get(optimizer::Optimizer, ::ElasticFilterTolerance)
    return optimizer.elastic_filter_tolerance
end

struct ElasticFilterIgnoreIntegrality end

function MOI.set(
    optimizer::Optimizer,
    ::ElasticFilterIgnoreIntegrality,
    value::Bool,
)
    optimizer.ignore_integrality = value
    return
end

function MOI.get(optimizer::Optimizer, ::ElasticFilterIgnoreIntegrality)
    return optimizer.ignore_integrality
end

function MOI.get(optimizer::Optimizer, attr::MOI.ConflictStatus)
    return optimizer.status
end

function MOI.get(
    optimizer::Optimizer,
    attr::MOI.ConstraintConflictStatus,
    con::MOI.ConstraintIndex,
)
    return MOI.get(optimizer, ConstraintConflictStatus(1), con)
end

# this should be moved to MOI
struct ConflictCount <: MOI.AbstractModelAttribute end

function MOI.get(optimizer::Optimizer, attr::ConflictCount)
    return length(optimizer.results)
end

# the MOI version must be generalized
struct ConstraintConflictStatus <: MOI.AbstractModelAttribute
    conflict_index::Int
    ConstraintConflictStatus(conflict_index = 1) = new(conflict_index)
end

function MOI.get(
    optimizer::Optimizer,
    attr::ConstraintConflictStatus,
    con::MOI.ConstraintIndex,
)
    if attr.conflict_index > length(optimizer.results)
        return MOI.NOT_IN_CONFLICT # or error
    end
    if con in optimizer.results[attr.conflict_index].constraints
        return MOI.IN_CONFLICT
    elseif con in optimizer.results[attr.conflict_index].maybe_constraints
        return MOI.MAYBE_IN_CONFLICT
    end
    return MOI.NOT_IN_CONFLICT
end

struct ListOfConstraintIndicesInConflict <: MOI.AbstractModelAttribute
    conflict_index::Int
    ListOfConstraintIndicesInConflict(conflict_index = 1) = new(conflict_index)
end

function MOI.get(optimizer::Optimizer, attr::ListOfConstraintIndicesInConflict)
    if attr.conflict_index > length(optimizer.results)
        return MOI.ConstraintIndex[]
    end
    return optimizer.results[attr.conflict_index].constraints
end

function _in_time(optimizer::Optimizer)
    @assert optimizer.start_time != NaN
    return time() - optimizer.start_time < optimizer.time_limit
end

function MOI.compute_conflict!(optimizer::Optimizer)
    optimizer.status = MOI.NO_CONFLICT_FOUND
    optimizer.results = InfeasibilityData[]
    optimizer.start_time = time()

    if optimizer.verbose
        println("Starting MathOptConflictSolver IIS search.")
    end

    T = Float64

    is_feasible = _feasibility_check(optimizer)
    if is_feasible && !optimizer.skip_feasibility_check
        optimizer.status = MOI.NO_CONFLICT_EXISTS
        return optimizer.results
    end

    if optimizer.verbose
        println("Starting bound analysis.")
    end
    bounds_consistent, variables, lb_con, ub_con =
        _bound_infeasibility!(optimizer, T)
    bound_infeasibilities = length(optimizer.results)
    if optimizer.verbose
        println(
            "Complete bound analysis found $bound_infeasibilities infeasibilities.",
        )
    end
    if length(optimizer.results) > 0
        optimizer.status = MOI.CONFLICT_FOUND
    end

    # check PSD diagonal >= 0 ?
    # other cones?
    if (!bounds_consistent && optimizer.stop_if_infeasible_bounds) ||
       !_in_time(optimizer)
        return
    end

    # second layer of infeasibility analysis is constraint range analysis
    if optimizer.verbose
        println("Starting range analysis.")
    end
    range_consistent =
        _range_infeasibility!(optimizer, T, variables, lb_con, ub_con)
    range_infeasibilities = length(optimizer.results) - bound_infeasibilities
    if optimizer.verbose
        println(
            "Complete range analysis found $range_infeasibilities infeasibilities.",
        )
    end
    if length(optimizer.results) > 0
        optimizer.status = MOI.CONFLICT_FOUND
    end

    if (!range_consistent && optimizer.stop_if_infeasible_ranges) ||
       !_in_time(optimizer)
        return
    end

    # check if there is a optimizer
    # third layer is an IIS resolver
    if optimizer.optimizer === nothing
        println(
            "IIS resolver cannot continue because no optimizer was provided",
        )
        return
    end
    if optimizer.verbose
        println("Starting elastic filter solver.")
    end
    iis = _elastic_filter(optimizer)
    # for now, only one iis is computed
    if iis !== nothing
        maybe_constraints = _get_variables_in_constraints(optimizer, iis)
        push!(
            optimizer.results,
            InfeasibilityData(
                iis,
                true,
                NoData(),
                maybe_constraints = maybe_constraints,
            ),
        )
        optimizer.status = MOI.CONFLICT_FOUND
    end
    iis_infeasibilities = ifelse(iis === nothing, 0, 1)
    if optimizer.verbose
        println(
            "Complete elastic filter solver found $iis_infeasibilities infeasibilities.",
        )
    end

    return
end

function _get_variables_in_constraints(
    optimizer::Optimizer,
    con::Vector{MOI.ConstraintIndex},
)
    variables = Set{MOI.VariableIndex}()
    for c in con
        _get_variables_in_constraints!(optimizer, c, variables)
    end
    con_types =
        MOI.get(optimizer.original_model, MOI.ListOfConstraintTypesPresent())
    variable_constraints = MOI.ConstraintIndex[]
    for (F, S) in con_types
        if F <: MOI.VariableIndex
            _variable_constraints = MOI.get(
                optimizer.original_model,
                MOI.ListOfConstraintIndices{F,S}(),
            )
            for con in _variable_constraints
                var = MOI.get(
                    optimizer.original_model,
                    MOI.ConstraintFunction(),
                    con,
                )
                if var in variables
                    push!(variable_constraints, con)
                end
            end
        end
    end
    return variable_constraints
end

function _get_variables_in_constraints!(
    optimizer::Optimizer,
    con::MOI.ConstraintIndex{F},
    variables::Set{MOI.VariableIndex},
) where {F<:MOI.ScalarAffineFunction}
    func = MOI.get(optimizer.original_model, MOI.ConstraintFunction(), con)
    for term in func.terms
        push!(variables, term.variable)
    end
    return
end

function _get_variables_in_constraints!(
    optimizer::Optimizer,
    con::MOI.ConstraintIndex{F},
    variables::Set{MOI.VariableIndex},
) where {F}
    # skip
    return
end

function _feasibility_check(optimizer::Optimizer)
    termination_status =
        MOI.get(optimizer.original_model, MOI.TerminationStatus())
    if optimizer.verbose
        println("Original model termination status: $(termination_status)")
    end
    if termination_status in
       (MOI.OTHER_ERROR, MOI.INVALID_MODEL, MOI.OPTIMIZE_NOT_CALLED)
        return false # because we can assert it is feasible
    end
    primal_status = MOI.get(optimizer.original_model, MOI.PrimalStatus())
    if optimizer.verbose
        println("Original model primal status: $(primal_status)")
    end
    if primal_status in (MOI.FEASIBLE_POINT, MOI.NEARLY_FEASIBLE_POINT) && !(
        termination_status in
        (MOI.INFEASIBLE, MOI.ALMOST_INFEASIBLE, MOI.LOCALLY_INFEASIBLE)
    )
        return true
    end
    return false
end

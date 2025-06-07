# Copyright (c) 2025: Joaquim Dias Garcia, Oscar Dowson and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

abstract type AbstractAdditionalData end

struct InfeasibilityData
    constraints::Vector{MOI.ConstraintIndex}
    irreducible::Bool
    metadata::AbstractAdditionalData
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

function compute_conflicts(model::MOI.ModelLike; optimizer = nothing)
    out = InfeasibilityData[]

    T = Float64

    variables = Dict{MOI.VariableIndex,Interval{T}}()

    variable_indices = MOI.get(model, MOI.ListOfVariableIndices())

    lb = Dict{MOI.VariableIndex,T}()
    lb_con = Dict{MOI.VariableIndex,MOI.ConstraintIndex}()
    ub = Dict{MOI.VariableIndex,T}()
    ub_con = Dict{MOI.VariableIndex,MOI.ConstraintIndex}()

    for con in MOI.get(
        model,
        MOI.ListOfConstraintIndices{MOI.VariableIndex,MOI.EqualTo{T}}(),
    )
        set = MOI.get(model, MOI.ConstraintSet(), con)
        func = MOI.get(model, MOI.ConstraintFunction(), con)
        lb[func] = set.value
        lb_con[func] = con
        ub[func] = set.value
        ub_con[func] = con
    end

    for con in MOI.get(
        model,
        MOI.ListOfConstraintIndices{MOI.VariableIndex,MOI.LessThan{T}}(),
    )
        set = MOI.get(model, MOI.ConstraintSet(), con)
        func = MOI.get(model, MOI.ConstraintFunction(), con)
        # lb[func] = -Inf
        ub[func] = set.upper
        ub_con[func] = con
    end

    for con in MOI.get(
        model,
        MOI.ListOfConstraintIndices{MOI.VariableIndex,MOI.GreaterThan{T}}(),
    )
        set = MOI.get(model, MOI.ConstraintSet(), con)
        func = MOI.get(model, MOI.ConstraintFunction(), con)
        lb[func] = set.lower
        lb_con[func] = con
        # ub[func] = Inf
    end

    for con in MOI.get(
        model,
        MOI.ListOfConstraintIndices{MOI.VariableIndex,MOI.Interval{T}}(),
    )
        set = MOI.get(model, MOI.ConstraintSet(), con)
        func = MOI.get(model, MOI.ConstraintFunction(), con)
        lb[func] = set.lower
        lb_con[func] = con
        ub[func] = set.upper
        ub_con[func] = con
    end

    # for con in MOI.get(model, MOI.ListOfConstraintIndices{MOI.VariableIndex,MOI.SemiContinuous{T}}())
    #     set = MOI.get(model, MOI.ConstraintSet(), con)
    #     func = MOI.get(model, MOI.ConstraintFunction(), con)
    #     lb[func] = 0 # set.lower
    #     ub[func] = set.upper
    # end

    # for con in MOI.get(model, MOI.ListOfConstraintIndices{MOI.VariableIndex,MOI.SemiInteger{T}}())
    #     set = MOI.get(model, MOI.ConstraintSet(), con)
    #     func = MOI.get(model, MOI.ConstraintFunction(), con)
    #     lb[func] = 0 #set.lower
    #     ub[func] = set.upper
    # end

    bounds_consistent = true

    for con in MOI.get(
        model,
        MOI.ListOfConstraintIndices{MOI.VariableIndex,MOI.Integer}(),
    )
        func = MOI.get(model, MOI.ConstraintFunction(), con)
        _lb = get(lb, func, -Inf)
        _ub = get(ub, func, Inf)
        if abs(_ub - _lb) < 1 && ceil(_ub) == ceil(_lb)
            push!(
                out,
                InfeasibilityData(
                    [con, lb_con[func], ub_con[func]],
                    true,
                    IntegralityData(_lb, _ub, MOI.Integer()),
                ),
            )
            bounds_consistent = false
        end
    end

    for con in MOI.get(
        model,
        MOI.ListOfConstraintIndices{MOI.VariableIndex,MOI.ZeroOne}(),
    )
        func = MOI.get(model, MOI.ConstraintFunction(), con)
        _lb = get(lb, func, -Inf)
        _ub = get(ub, func, Inf)
        if _lb > 0 && _ub < 1
            push!(
                out,
                InfeasibilityData(
                    [con, lb_con[func], ub_con[func]],
                    true,
                    IntegralityData(_lb, _ub, MOI.ZeroOne()),
                ),
            )
            bounds_consistent = false
        elseif _lb > 1
            push!(
                out,
                InfeasibilityData(
                    [con, lb_con[func]],
                    true,
                    IntegralityData(_lb, Inf, MOI.ZeroOne()),
                ),
            )
            bounds_consistent = false
        elseif _ub < 0
            push!(
                out,
                InfeasibilityData(
                    [con, ub_con[func]],
                    true,
                    IntegralityData(-Inf, _ub, MOI.ZeroOne()),
                ),
            )
            bounds_consistent = false
        end
    end

    for var in variable_indices
        _lb = get(lb, var, -Inf)
        _ub = get(ub, var, Inf)
        if _lb > _ub
            push!(
                out,
                InfeasibilityData(
                    [lb_con[var], ub_con[var]],
                    true,
                    BoundsData(_lb, _ub),
                ),
            )
            bounds_consistent = false
        else
            variables[var] = Interval(_lb, _ub)
        end
    end

    # check PSD diagonal >= 0 ?
    # other cones?
    if !bounds_consistent
        return out
    end

    # second layer of infeasibility analysis is constraint range analysis
    range_consistent = true

    affine_cons = vcat(
        MOI.get(
            model,
            MOI.ListOfConstraintIndices{
                MOI.ScalarAffineFunction{T},
                MOI.EqualTo{T},
            }(),
        ),
        MOI.get(
            model,
            MOI.ListOfConstraintIndices{
                MOI.ScalarAffineFunction{T},
                MOI.LessThan{T},
            }(),
        ),
        MOI.get(
            model,
            MOI.ListOfConstraintIndices{
                MOI.ScalarAffineFunction{T},
                MOI.GreaterThan{T},
            }(),
        ),
    )

    for con in affine_cons
        func = MOI.get(model, MOI.ConstraintFunction(), con)
        # failed = false
        list_of_variables = MOI.VariableIndex[]
        interval = _eval_variables(func) do var_idx
            push!(list_of_variables, var_idx)
            # this only fails if we allow continuing after bounds issues
            # if !haskey(variables, var_idx)
            #     failed = true
            #     return Interval(-Inf, Inf)
            # end
            return variables[var_idx]
        end
        # if failed
        #     continue
        # end
        set = MOI.get(model, MOI.ConstraintSet(), con)
        if _invalid_range(set, interval)
            cons = Set{MOI.ConstraintIndex}()
            push!(cons, con)
            for var in list_of_variables
                if haskey(lb_con, var)
                    push!(cons, lb_con[var])
                end
                if haskey(ub_con, var)
                    push!(cons, ub_con[var])
                end
            end
            push!(
                out,
                InfeasibilityData(
                    collect(cons),
                    true, # strictly speaking, we might need the propor "sides"
                    RangeData(interval.lo, interval.hi, set),
                ),
            )
            range_consistent = false
        end
    end

    if !range_consistent
        return out
    end

    # check if there is a optimizer
    # third layer is an IIS resolver
    if optimizer === nothing
        println("iis resolver cannot continue because no optimizer is provided")
        return out
    end
    iis = iis_elastic_filter(model, optimizer)
    # for now, only one iis is computed
    if iis !== nothing
        push!(out, InfeasibilityData(iis, true, NoData()))
    end

    return out
end

function _invalid_range(set::MOI.EqualTo, interval)
    rhs = set.value
    return interval.lo > rhs || interval.hi < rhs
end

function _invalid_range(set::MOI.LessThan, interval)
    rhs = set.upper
    return interval.lo > rhs
end

function _invalid_range(set::MOI.GreaterThan, interval)
    rhs = set.lower
    return interval.hi < rhs
end

function _fix_to_zero(model, variable::MOI.VariableIndex, ::Type{T}) where {T}
    ub_idx =
        MOI.ConstraintIndex{MOI.VariableIndex,MOI.LessThan{T}}(variable.value)
    lb_idx = MOI.ConstraintIndex{MOI.VariableIndex,MOI.GreaterThan{T}}(
        variable.value,
    )
    has_lower = false
    if MOI.is_valid(model, lb_idx)
        MOI.delete(model, lb_idx)
        has_lower = true
        # MOI.PenaltyRelaxation only creates variables with LB
        # elseif MOI.is_valid(model, ub_idx)
        #     MOI.delete(model, ub_idx)
    else
        error("Variable is not bounded")
    end
    MOI.add_constraint(model, variable, MOI.EqualTo{T}(zero(T)))
    return has_lower
end

function _set_bound_zero(
    model,
    variable::MOI.VariableIndex,
    has_lower::Bool,
    ::Type{T},
) where {T}
    eq_idx =
        MOI.ConstraintIndex{MOI.VariableIndex,MOI.EqualTo{T}}(variable.value)
    @assert MOI.is_valid(model, eq_idx)
    MOI.delete(model, eq_idx)
    if has_lower
        MOI.add_constraint(model, variable, MOI.GreaterThan{T}(zero(T)))
        # MOI.PenaltyRelaxation only creates variables with LB
        # else
        #     MOI.add_constraint(model, variable, MOI.LessThan{T}(zero(T)))
    end
    return
end

function iis_elastic_filter(original_model::MOI.ModelLike, optimizer)
    T = Float64

    # handle optimize not called
    status = MOI.get(original_model, MOI.TerminationStatus())
    if !(
        status in
        (MOI.INFEASIBLE, MOI.ALMOST_INFEASIBLE, MOI.ALMOST_INFEASIBLE)
    )
        println(
            "iis resolver cannot continue because model is found to be $(status) by the solver",
        )
        return nothing
    end

    model = MOI.instantiate(optimizer)
    reference_map = MOI.copy_to(model, original_model)
    MOI.set(model, MOI.Silent(), true)

    obj_sense = MOI.get(model, MOI.ObjectiveSense())
    base_obj_type = MOI.get(model, MOI.ObjectiveFunctionType())
    base_obj_func = MOI.get(model, MOI.ObjectiveFunction{base_obj_type}())

    constraint_to_affine =
        MOI.modify(model, MOI.Utilities.PenaltyRelaxation(default = 1.0))
    # might need to do something related to integers / binary
    relaxed_obj_type = MOI.get(model, MOI.ObjectiveFunctionType())
    relaxed_obj_func = MOI.get(model, MOI.ObjectiveFunction{relaxed_obj_type}())

    pure_relaxed_obj_func = relaxed_obj_func - base_obj_func

    max_iterations = length(constraint_to_affine)

    tolerance = 1e-5

    de_elastisized = []

    changed_obj = false

    for i in 1:max_iterations
        MOI.optimize!(model)
        status = MOI.get(model, MOI.TerminationStatus())
        if status in ( # possibily primal unbounded
            MOI.INFEASIBLE_OR_UNBOUNDED,
            MOI.DUAL_INFEASIBLE,
            MOI.ALMOST_DUAL_INFEASIBLE,
        )
            #try with a pure relaxation objective
            MOI.set(
                model,
                MOI.ObjectiveFunction{relaxed_obj_type}(),
                pure_relaxed_obj_func,
            )
            changed_obj = true
            MOI.optimize!(model)
        end
        if status in
           (MOI.INFEASIBLE, MOI.ALMOST_INFEASIBLE, MOI.ALMOST_INFEASIBLE)
            break
        end
        for (con, func) in constraint_to_affine
            if length(func.terms) == 1
                var = func.terms[1].variable
                value = MOI.get(model, MOI.VariablePrimal(), var)
                if value > tolerance
                    has_lower = _fix_to_zero(model, var, T)
                    delete!(constraint_to_affine, con)
                    push!(de_elastisized, (con, var, has_lower))
                end
            elseif length(func.terms) == 2
                var1 = func.terms[1].variable
                coef1 = func.terms[1].coefficient
                var2 = func.terms[2].variable
                coef2 = func.terms[2].coefficient
                value1 = MOI.get(model, MOI.VariablePrimal(), var1)
                value2 = MOI.get(model, MOI.VariablePrimal(), var2)
                if value1 > tolerance && value2 > tolerance
                    error("IIS failed due numerical instability")
                elseif value1 > tolerance
                    # TODO: coef is alwayas 1.0
                    has_lower = _fix_to_zero(model, var1, T)
                    delete!(constraint_to_affine, con)
                    constraint_to_affine[con] = coef2 * var2
                    push!(de_elastisized, (con, var1, has_lower))
                elseif value2 > tolerance
                    has_lower = _fix_to_zero(model, var2, T)
                    delete!(constraint_to_affine, con)
                    constraint_to_affine[con] = coef1 * var1
                    push!(de_elastisized, (con, var2, has_lower))
                end
            else
                println(
                    "$con and relaxing function with more than two terms: $func",
                )
            end
        end
    end

    if changed_obj
        MOI.set(
            model,
            MOI.ObjectiveFunction{relaxed_obj_type}(),
            relaxed_obj_func,
        )
    end

    # consider deleting all no iis constraints
    # be careful with intervals

    obj_type = MOI.get(model, MOI.ObjectiveFunctionType())
    obj_func = MOI.get(model, MOI.ObjectiveFunction{obj_type}())
    obj_sense = MOI.get(model, MOI.ObjectiveSense())

    # deletion filter
    cadidates = MOI.ConstraintIndex[]
    for (con, var, has_lower) in de_elastisized
        _set_bound_zero(model, var, has_lower, T)
        MOI.optimize!(model)
        status = MOI.get(model, MOI.TerminationStatus())
        if status in
           (MOI.INFEASIBLE, MOI.ALMOST_INFEASIBLE, MOI.ALMOST_INFEASIBLE)
            # this constraint is not in IIS
        elseif status in (
            MOI.OPTIMAL,
            MOI.ALMOST_OPTIMAL,
            MOI.LOCALLY_SOLVED,
            MOI.ALMOST_LOCALLY_SOLVED,
        )
            push!(cadidates, con)
            _fix_to_zero(model, var, T)
        elseif status in (
            MOI.INFEASIBLE_OR_UNBOUNDED,
            MOI.DUAL_INFEASIBLE,
            MOI.ALMOST_DUAL_INFEASIBLE, # possibily primal unbounded
        )
            MOI.set(model, MOI.ObjectiveSense(), MOI.FEASIBILITY_SENSE)
            MOI.optimize!(model)
            primal_status = MOI.get(model, MOI.PrimalStatus())
            if primal_status in (MOI.FEASIBLE_POINT, MOI.NEARLY_FEASIBLE_POINT)
                # this constraint is not in IIS
                push!(cadidates, con)
                _fix_to_zero(model, var, T)
                MOI.set(model, MOI.ObjectiveSense(), obj_sense)
                MOI.set(model, MOI.ObjectiveFunction{obj_type}(), obj_func)
            else
                error(
                    "IIS failed due numerical instability, got status $status,",
                    "then, for MOI.FEASIBILITY_SENSE objective, got primal status $primal_status",
                )
            end
        else
            error("IIS failed due numerical instability, got status $status")
        end
    end

    pre_iis = Set(cadidates)
    iis = MOI.ConstraintIndex[]
    for (F, S) in MOI.get(original_model, MOI.ListOfConstraintTypesPresent())
        if F == MOI.VariableIndex
            continue
        end
        for con in MOI.get(original_model, MOI.ListOfConstraintIndices{F,S}())
            new_con = reference_map[con]
            if new_con in pre_iis
                push!(iis, con)
            end
        end
    end

    return iis
end

#=
    Helpers
=#

# This type and the associated function were inspired by IntervalArithmetic.jl
# Copyright (c) 2014-2021: David P. Sanders & Luis Benet

struct Interval{T<:Real}
    lo::T
    hi::T

    function Interval(lo::T, hi::T) where {T<:Real}
        @assert lo <= hi
        return new{T}(lo, hi)
    end
end

Base.convert(::Type{Interval{T}}, x::T) where {T<:Real} = Interval(x, x)

Base.zero(::Type{Interval{T}}) where {T<:Real} = Interval(zero(T), zero(T))

Base.iszero(a::Interval) = iszero(a.hi) && iszero(a.lo)

function Base.:+(a::Interval{T}, b::Interval{T}) where {T<:Real}
    return Interval(a.lo + b.lo, a.hi + b.hi)
end

function Base.:*(x::T, a::Interval{T}) where {T<:Real}
    if iszero(a) || iszero(x)
        return Interval(zero(T), zero(T))
    elseif x >= zero(T)
        return Interval(a.lo * x, a.hi * x)
    end
    return Interval(a.hi * x, a.lo * x)
end

# This type and the associated function were inspired by JuMP.jl and
# MathOptInterface.jl

function _eval_variables(value_fn::Function, t::MOI.ScalarAffineTerm)
    return t.coefficient * value_fn(t.variable)
end

function _eval_variables(
    value_fn::Function,
    f::MOI.ScalarAffineFunction{T},
) where {T}
    # TODO: this conversion exists in JuMP, but not in MOI
    S = Base.promote_op(value_fn, MOI.VariableIndex)
    U = MOI.MA.promote_operation(*, T, S)
    out = convert(U, f.constant)
    for t in f.terms
        out += _eval_variables(value_fn, t)
    end
    return out
end

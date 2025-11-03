# Copyright (c) 2025: Joaquim Dias Garcia, Oscar Dowson and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

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

# Back to functions written for MathOptIIS.jl

function _range_infeasibility!(
    optimizer::Optimizer,
    ::Type{T},
    variables::Dict{MOI.VariableIndex,Interval{T}},
    lb_con::Dict{MOI.VariableIndex,MOI.ConstraintIndex},
    ub_con::Dict{MOI.VariableIndex,MOI.ConstraintIndex},
) where {T}
    range_consistent = _range_infeasibility!(
        optimizer,
        optimizer.original_model,
        T,
        variables,
        lb_con,
        ub_con,
        MOI.EqualTo{T},
    )
    range_consistent &= _range_infeasibility!(
        optimizer,
        optimizer.original_model,
        T,
        variables,
        lb_con,
        ub_con,
        MOI.LessThan{T},
    )
    return _range_infeasibility!(
        optimizer,
        optimizer.original_model,
        T,
        variables,
        lb_con,
        ub_con,
        MOI.GreaterThan{T},
    )
end

function _range_infeasibility!(
    optimizer::Optimizer,
    original_model::MOI.ModelLike,
    ::Type{T},
    variables::Dict{MOI.VariableIndex,Interval{T}},
    lb_con::Dict{MOI.VariableIndex,MOI.ConstraintIndex},
    ub_con::Dict{MOI.VariableIndex,MOI.ConstraintIndex},
    ::Type{S},
) where {T,S}
    range_consistent = true
    for con in MOI.get(
        original_model,
        MOI.ListOfConstraintIndices{MOI.ScalarAffineFunction{T},S}(),
    )
        if !_in_time(optimizer)
            return range_consistent
        end
        func = MOI.get(original_model, MOI.ConstraintFunction(), con)
        interval = _eval_variables(variables, func)
        if interval === nothing
            continue
        end
        set = MOI.get(original_model, MOI.ConstraintSet(), con)::S
        if !_invalid_range(set, interval)
            continue
        end
        cons = Set{MOI.ConstraintIndex}()
        push!(cons, con)
        for t in func.terms
            if (c = get(lb_con, t.variable, nothing)) !== nothing
                push!(cons, c)
            end
            if (c = get(ub_con, t.variable, nothing)) !== nothing
                push!(cons, c)
            end
        end
        push!(
            optimizer.results,
            InfeasibilityData(
                collect(cons),
                true, # strictly speaking, we might need the proper "sides"
                RangeData(interval.lo, interval.hi, set),
            ),
        )
        range_consistent = false
    end
    return range_consistent
end

function _eval_variables(
    map::AbstractDict{MOI.VariableIndex,U},
    f::MOI.ScalarAffineFunction,
) where {U}
    out = convert(U, f.constant)
    for t in f.terms
        v = get(map, t.variable, nothing)
        if v === nothing
            return
        end
        out += t.coefficient * v
    end
    return out
end

function _invalid_range(set::MOI.EqualTo, interval)
    return !(interval.lo <= set.value <= interval.hi)
end

_invalid_range(set::MOI.LessThan, interval) = set.upper < interval.lo

_invalid_range(set::MOI.GreaterThan, interval) = interval.hi < set.lower

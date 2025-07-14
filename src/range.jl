# Copyright (c) 2025: Joaquim Dias Garcia, Oscar Dowson and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

function _range_infeasibility!(
    optimizer::Optimizer,
    ::Type{T},
    variables,
    lb_con::Dict{MOI.VariableIndex,MOI.ConstraintIndex},
    ub_con::Dict{MOI.VariableIndex,MOI.ConstraintIndex},
) where {T}
    range_consistent = true

    affine_cons = vcat(
        MOI.get(
            optimizer.original_model,
            MOI.ListOfConstraintIndices{
                MOI.ScalarAffineFunction{T},
                MOI.EqualTo{T},
            }(),
        ),
        MOI.get(
            optimizer.original_model,
            MOI.ListOfConstraintIndices{
                MOI.ScalarAffineFunction{T},
                MOI.LessThan{T},
            }(),
        ),
        MOI.get(
            optimizer.original_model,
            MOI.ListOfConstraintIndices{
                MOI.ScalarAffineFunction{T},
                MOI.GreaterThan{T},
            }(),
        ),
    )

    for con in affine_cons
        if !_in_time(optimizer)
            return range_consistent
        end
        func = MOI.get(optimizer.original_model, MOI.ConstraintFunction(), con)
        failed = false
        list_of_variables = MOI.VariableIndex[]
        interval = _eval_variables(func) do var_idx
            push!(list_of_variables, var_idx)
            # this only fails if we allow continuing after bounds issues
            if !haskey(variables, var_idx)
                failed = true
                return Interval(-Inf, Inf)
            end
            return variables[var_idx]
        end
        if failed
            continue
        end
        set = MOI.get(optimizer.original_model, MOI.ConstraintSet(), con)
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
                optimizer.results,
                InfeasibilityData(
                    collect(cons),
                    true, # strictly speaking, we might need the propor "sides"
                    RangeData(interval.lo, interval.hi, set),
                ),
            )
            range_consistent = false
        end
    end
    return range_consistent
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

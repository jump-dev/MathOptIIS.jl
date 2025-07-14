# Copyright (c) 2025: Joaquim Dias Garcia, Oscar Dowson and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

function _bound_infeasibility!(optimizer::Optimizer, ::Type{T}) where {T}
    variables = Dict{MOI.VariableIndex,Interval{T}}()

    variable_indices =
        MOI.get(optimizer.original_model, MOI.ListOfVariableIndices())

    lb = Dict{MOI.VariableIndex,T}()
    lb_con = Dict{MOI.VariableIndex,MOI.ConstraintIndex}()
    ub = Dict{MOI.VariableIndex,T}()
    ub_con = Dict{MOI.VariableIndex,MOI.ConstraintIndex}()

    for con in MOI.get(
        optimizer.original_model,
        MOI.ListOfConstraintIndices{MOI.VariableIndex,MOI.EqualTo{T}}(),
    )
        set = MOI.get(optimizer.original_model, MOI.ConstraintSet(), con)
        func = MOI.get(optimizer.original_model, MOI.ConstraintFunction(), con)
        lb[func] = set.value
        lb_con[func] = con
        ub[func] = set.value
        ub_con[func] = con
    end

    for con in MOI.get(
        optimizer.original_model,
        MOI.ListOfConstraintIndices{MOI.VariableIndex,MOI.LessThan{T}}(),
    )
        set = MOI.get(optimizer.original_model, MOI.ConstraintSet(), con)
        func = MOI.get(optimizer.original_model, MOI.ConstraintFunction(), con)
        # lb[func] = -Inf
        ub[func] = set.upper
        ub_con[func] = con
    end

    for con in MOI.get(
        optimizer.original_model,
        MOI.ListOfConstraintIndices{MOI.VariableIndex,MOI.GreaterThan{T}}(),
    )
        set = MOI.get(optimizer.original_model, MOI.ConstraintSet(), con)
        func = MOI.get(optimizer.original_model, MOI.ConstraintFunction(), con)
        lb[func] = set.lower
        lb_con[func] = con
        # ub[func] = Inf
    end

    for con in MOI.get(
        optimizer.original_model,
        MOI.ListOfConstraintIndices{MOI.VariableIndex,MOI.Interval{T}}(),
    )
        set = MOI.get(optimizer.original_model, MOI.ConstraintSet(), con)
        func = MOI.get(optimizer.original_model, MOI.ConstraintFunction(), con)
        lb[func] = set.lower
        lb_con[func] = con
        ub[func] = set.upper
        ub_con[func] = con
    end

    # for con in MOI.get(optimizer.original_model, MOI.ListOfConstraintIndices{MOI.VariableIndex,MOI.SemiContinuous{T}}())
    #     set = MOI.get(optimizer.original_model, MOI.ConstraintSet(), con)
    #     func = MOI.get(optimizer.original_model, MOI.ConstraintFunction(), con)
    #     lb[func] = 0 # set.lower
    #     ub[func] = set.upper
    # end

    # for con in MOI.get(optimizer.original_model, MOI.ListOfConstraintIndices{MOI.VariableIndex,MOI.SemiInteger{T}}())
    #     set = MOI.get(optimizer.original_model, MOI.ConstraintSet(), con)
    #     func = MOI.get(optimizer.original_model, MOI.ConstraintFunction(), con)
    #     lb[func] = 0 #set.lower
    #     ub[func] = set.upper
    # end

    bounds_consistent = true

    for con in MOI.get(
        optimizer.original_model,
        MOI.ListOfConstraintIndices{MOI.VariableIndex,MOI.Integer}(),
    )
        func = MOI.get(optimizer.original_model, MOI.ConstraintFunction(), con)
        _lb = get(lb, func, -Inf)
        _ub = get(ub, func, Inf)
        if abs(_ub - _lb) < 1 && ceil(_ub) == ceil(_lb)
            push!(
                optimizer.results,
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
        optimizer.original_model,
        MOI.ListOfConstraintIndices{MOI.VariableIndex,MOI.ZeroOne}(),
    )
        func = MOI.get(optimizer.original_model, MOI.ConstraintFunction(), con)
        _lb = get(lb, func, -Inf)
        _ub = get(ub, func, Inf)
        if _lb > 0 && _ub < 1
            push!(
                optimizer.results,
                InfeasibilityData(
                    [con, lb_con[func], ub_con[func]],
                    true,
                    IntegralityData(_lb, _ub, MOI.ZeroOne()),
                ),
            )
            bounds_consistent = false
        elseif _lb > 1
            push!(
                optimizer.results,
                InfeasibilityData(
                    [con, lb_con[func]],
                    true,
                    IntegralityData(_lb, Inf, MOI.ZeroOne()),
                ),
            )
            bounds_consistent = false
        elseif _ub < 0
            push!(
                optimizer.results,
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
                optimizer.results,
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
    return bounds_consistent, variables, lb_con, ub_con
end

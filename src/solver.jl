# Copyright (c) 2025: Joaquim Dias Garcia, Oscar Dowson and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

function _fix_slack(model, variable::MOI.VariableIndex, ::Type{T}) where {T}
    lb_idx = MOI.ConstraintIndex{MOI.VariableIndex,MOI.GreaterThan{T}}(
        variable.value,
    )
    @assert MOI.is_valid(model, lb_idx)
    MOI.delete(model, lb_idx)
    MOI.add_constraint(model, variable, MOI.EqualTo{T}(zero(T)))
    return
end

function _unfix_slack(model, variable::MOI.VariableIndex, ::Type{T}) where {T}
    eq_idx =
        MOI.ConstraintIndex{MOI.VariableIndex,MOI.EqualTo{T}}(variable.value)
    @assert MOI.is_valid(model, eq_idx)
    MOI.delete(model, eq_idx)
    MOI.add_constraint(model, variable, MOI.GreaterThan{T}(zero(T)))
    return
end

function _elastic_filter(optimizer::Optimizer)
    T = Float64

    model = MOI.instantiate(optimizer.optimizer)
    MOI.set(model, MOI.Silent(), true)
    for (k, v) in optimizer.optimizer_attributes
        MOI.set(model, k, v)
    end
    reference_map = MOI.copy_to(model, optimizer.original_model)

    if optimizer.ignore_integrality
        _cp_constraint_types =
            MOI.get(model, MOI.ListOfConstraintTypesPresent())
        # _removed_constraints = Tuple[]
        for (F, S) in _cp_constraint_types
            con_list = []
            if S in (
                MOI.ZeroOne,
                MOI.Integer,
                MOI.Semicontinuous{T},
                MOI.Semiinteger{T},
                MOI.SOS1{T},
                MOI.SOS2{T},
            ) || S <: MOI.Indicator
                con_list = MOI.get(model, MOI.ListOfConstraintIndices{F,S}())
            end
            for con in con_list
                MOI.delete(model, con)
                # # save removed constraints
                # func = MOI.get(model, MOI.ConstraintFunction(), con)
                # set = MOI.get(model, MOI.ConstraintSet(), con)
                # push!(_removed_constraints, (func, set))
            end
        end
    end

    obj_sense = MOI.get(model, MOI.ObjectiveSense())
    base_obj_type = MOI.get(model, MOI.ObjectiveFunctionType())
    base_obj_func = MOI.get(model, MOI.ObjectiveFunction{base_obj_type}())

    constraint_to_affine =
        MOI.modify(model, MOI.Utilities.PenaltyRelaxation(default = one(T)))
    # all slack variables added are of type ">= 0"
    # might need to do something related to integers / binary
    relaxed_obj_type = MOI.get(model, MOI.ObjectiveFunctionType())
    relaxed_obj_func = MOI.get(model, MOI.ObjectiveFunction{relaxed_obj_type}())
    pure_relaxed_obj_func = relaxed_obj_func - base_obj_func

    MOI.set(
        model,
        MOI.ObjectiveFunction{relaxed_obj_type}(),
        pure_relaxed_obj_func,
    )

    max_iterations = length(constraint_to_affine)

    tolerance = optimizer.elastic_filter_tolerance

    de_elastisized = []

    changed_obj = false

    # all (affine, non-bound) constraints are relaxed at this point
    # we will try to set positive slacks to zero until the model infeasible
    # the constraints of the fixed slacks are a IIS candidate

    for i in 1:max_iterations
        if !_in_time(optimizer)
            return nothing
        end
        MOI.optimize!(model)
        status = MOI.get(model, MOI.TerminationStatus())
        if status in ( # possibily primal unbounded statuses
            MOI.INFEASIBLE_OR_UNBOUNDED,
            MOI.DUAL_INFEASIBLE,
            MOI.ALMOST_DUAL_INFEASIBLE,
        )
            #
        end
        if status in
           (MOI.INFEASIBLE, MOI.ALMOST_INFEASIBLE, MOI.LOCALLY_INFEASIBLE)
            break
        end
        for (con, func) in constraint_to_affine
            @assert length(func.terms) <= 2
            if length(func.terms) == 1
                var = func.terms[1].variable
                value = MOI.get(model, MOI.VariablePrimal(), var)
                if value > tolerance
                    _fix_slack(model, var, T)
                    delete!(constraint_to_affine, con)
                    push!(de_elastisized, (con, var))
                end
            elseif length(func.terms) == 2
                var1 = func.terms[1].variable
                var2 = func.terms[2].variable
                value1 = MOI.get(model, MOI.VariablePrimal(), var1)
                value2 = MOI.get(model, MOI.VariablePrimal(), var2)
                if value1 > tolerance && value2 > tolerance
                    error("IIS failed due numerical instability")
                elseif value1 > tolerance
                    # TODO: coef is always 1.0
                    _fix_slack(model, var1, T)
                    delete!(constraint_to_affine, con)
                    constraint_to_affine[con] = one(T) * var2
                    push!(de_elastisized, (con, var1))
                elseif value2 > tolerance
                    _fix_slack(model, var2, T)
                    delete!(constraint_to_affine, con)
                    constraint_to_affine[con] = one(T) * var1
                    push!(de_elastisized, (con, var2))
                end
            end
        end
    end

    # consider deleting all no iis constraints
    # be careful with intervals

    obj_type = MOI.get(model, MOI.ObjectiveFunctionType())
    obj_func = MOI.get(model, MOI.ObjectiveFunction{obj_type}())
    obj_sense = MOI.get(model, MOI.ObjectiveSense())

    candidates = MOI.ConstraintIndex[]
    if !optimizer.deletion_filter
        for (con, var) in de_elastisized
            push!(candidates, con)
        end
        empty!(de_elastisized)
    end

    # deletion filter
    for (con, var) in de_elastisized
        if !_in_time(optimizer)
            return nothing
        end
        _unfix_slack(model, var, T)
        MOI.optimize!(model)
        status = MOI.get(model, MOI.TerminationStatus())
        if status in
           (MOI.INFEASIBLE, MOI.ALMOST_INFEASIBLE, MOI.LOCALLY_INFEASIBLE)
            # this constraint is not in IIS
            # hence it remains unfixed
        elseif status in (
            MOI.OPTIMAL,
            MOI.ALMOST_OPTIMAL,
            MOI.LOCALLY_SOLVED,
            MOI.ALMOST_LOCALLY_SOLVED,
        )
            push!(candidates, con)
            _fix_slack(model, var, T)
        elseif status in ( # possibily primal unbounded statuses
            MOI.INFEASIBLE_OR_UNBOUNDED,
            MOI.DUAL_INFEASIBLE,
            MOI.ALMOST_DUAL_INFEASIBLE,
        )
            MOI.set(model, MOI.ObjectiveSense(), MOI.FEASIBILITY_SENSE)
            MOI.optimize!(model)
            primal_status = MOI.get(model, MOI.PrimalStatus())
            # the unbounded case:
            if primal_status in (MOI.FEASIBLE_POINT, MOI.NEARLY_FEASIBLE_POINT)
                # this constraint is not in IIS
                push!(candidates, con)
                _fix_slack(model, var, T)
                MOI.set(model, MOI.ObjectiveSense(), obj_sense)
                MOI.set(
                    model,
                    MOI.ObjectiveFunction{relaxed_obj_type}(),
                    pure_relaxed_obj_func,
                )
                # the both primal and dual infeasible case:
                # else
                #     nothing
            end
        else
            error("IIS failed due numerical instability, got status $status")
        end
    end

    if isempty(candidates)
        return nothing
    end

    pre_iis = Set(candidates)
    iis = MOI.ConstraintIndex[]
    for (F, S) in
        MOI.get(optimizer.original_model, MOI.ListOfConstraintTypesPresent())
        if F == MOI.VariableIndex
            continue
        end
        for con in MOI.get(
            optimizer.original_model,
            MOI.ListOfConstraintIndices{F,S}(),
        )
            new_con = reference_map[con]
            if new_con in pre_iis
                push!(iis, con)
            end
        end
    end

    return iis
end

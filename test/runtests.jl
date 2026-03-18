# Copyright (c) 2025: Joaquim Dias Garcia, Oscar Dowson and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

module TestMathOptIIS

using JuMP
using Test

import HiGHS
import Ipopt
import MathOptIIS
import SCS

function runtests()
    for name in names(@__MODULE__; all = true)
        if startswith("$name", "test_")
            @testset "$(name)" begin
                getfield(@__MODULE__, name)()
            end
        end
    end
    return
end

function test_bounds()
    model = Model()
    @variable(model, 0 <= x <= 1)
    @variable(model, 2 <= y <= 1)
    @objective(model, Max, x + y)
    solver = MathOptIIS.Optimizer()
    @test MOI.get(solver, MOI.ConflictStatus()) ==
          MOI.COMPUTE_CONFLICT_NOT_CALLED
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].constraints ==
          [index(LowerBoundRef(y)), index(UpperBoundRef(y))]
    @test data[].metadata == MathOptIIS.Metadata(2.0, 1.0, nothing)
    @test MOI.get(solver, MOI.ConflictStatus()) == MOI.CONFLICT_FOUND
    @test MOI.get(
        solver,
        MOI.ConstraintConflictStatus(),
        index(LowerBoundRef(y)),
    ) == MOI.IN_CONFLICT
    @test MOI.get(
        solver,
        MOI.ConstraintConflictStatus(),
        index(UpperBoundRef(y)),
    ) == MOI.IN_CONFLICT
    @test MOI.get(solver, MOI.ConflictCount()) == 1
    @test MOI.get(
        solver,
        MOI.ConstraintConflictStatus(1),
        index(LowerBoundRef(y)),
    ) == MOI.IN_CONFLICT
    @test MOI.get(
        solver,
        MOI.ConstraintConflictStatus(1),
        index(UpperBoundRef(y)),
    ) == MOI.IN_CONFLICT
    # the next two could be errors
    @test MOI.get(
        solver,
        MOI.ConstraintConflictStatus(2),
        index(LowerBoundRef(y)),
    ) == MOI.NOT_IN_CONFLICT
    @test MOI.get(
        solver,
        MOI.ConstraintConflictStatus(2),
        index(UpperBoundRef(y)),
    ) == MOI.NOT_IN_CONFLICT
    #
    @test MOI.get(solver, MathOptIIS.ListOfConstraintIndicesInConflict(1)) ==
          [index(LowerBoundRef(y)), index(UpperBoundRef(y))]
    @test isempty(
        MOI.get(solver, MathOptIIS.ListOfConstraintIndicesInConflict(2)),
    )
    return
end

function test_integrality()
    model = Model()
    @variable(model, 0 <= x <= 1, Int)
    @variable(model, 2.2 <= y <= 2.9, Int)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].constraints == [
        index(IntegerRef(y)),
        index(LowerBoundRef(y)),
        index(UpperBoundRef(y)),
    ]
    @test data[].metadata == MathOptIIS.Metadata(2.2, 2.9, MOI.Integer())
    return
end

function test_binary_inner()
    model = Model()
    @variable(model, 0.5 <= x <= 0.8, Bin)
    @variable(model, 0 <= y <= 1, Bin)
    @constraint(model, x + y <= 1)
    @objective(model, Max, x + y)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].constraints == [
        index(BinaryRef(x)),
        index(LowerBoundRef(x)),
        index(UpperBoundRef(x)),
    ]
    @test data[].metadata == MathOptIIS.Metadata(0.5, 0.8, MOI.ZeroOne())
    return
end

function test_binary_lower()
    model = Model()
    @variable(model, 1.5 <= x <= 1.8, Bin)
    @variable(model, 0 <= y <= 1, Bin)
    @constraint(model, x + y <= 1)
    @objective(model, Max, x + y)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 2
    @test data[1].constraints == [index(BinaryRef(x)), index(LowerBoundRef(x))]
    @test data[1].metadata == MathOptIIS.Metadata(1.5, Inf, MOI.ZeroOne())
    return
end

function test_binary_upper()
    model = Model()
    @variable(model, -2.5 <= x <= -1.8, Bin)
    @variable(model, 0 <= y <= 1, Bin)
    @constraint(model, x + y <= 1)
    @objective(model, Max, x + y)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].constraints == [index(BinaryRef(x)), index(UpperBoundRef(x))]
    @test data[].metadata == MathOptIIS.Metadata(-Inf, -1.8, MOI.ZeroOne())
    return
end

function _isequal_unordered(v1::AbstractVector, v2::AbstractVector)
    return length(v1) == length(v2) && Set(v1) == Set(v2)
end

function test_range()
    model = Model()
    @variable(model, 10 <= x <= 11)
    @variable(model, 1 <= y <= 11)
    @constraint(model, c, x + y <= 1)
    @objective(model, Max, x + y)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test _isequal_unordered(
        data[].constraints,
        [
            index(c),
            index(UpperBoundRef(x)),
            index(LowerBoundRef(x)),
            index(UpperBoundRef(y)),
            index(LowerBoundRef(y)),
        ],
    )
    @test data[].metadata ==
          MathOptIIS.Metadata(11.0, 22.0, MOI.LessThan{Float64}(1.0))
    return
end

function test_range_and_bound()
    model = Model()
    @variable(model, 10 <= x <= 11)
    @variable(model, 1 <= y <= 11)
    @variable(model, 1 <= z <= 0)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test _isequal_unordered(
        data[].constraints,
        [index(LowerBoundRef(z)), index(UpperBoundRef(z))],
    )
    return
end

function test_range_and_bound_2()
    model = Model()
    @variable(model, 10 <= x <= 11)
    @variable(model, 1 <= y <= 11)
    @variable(model, 1 <= z <= 0)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test _isequal_unordered(
        data[].constraints,
        [index(LowerBoundRef(z)), index(UpperBoundRef(z))],
    )
    return
end

function test_range_neg()
    model = Model()
    @variable(model, 10 <= x <= 11)
    @variable(model, -11 <= y <= -1)
    @constraint(model, c, x - y <= 1)
    @objective(model, Max, x + y)
    #
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test _isequal_unordered(
        data[].constraints,
        [
            index(c),
            index(UpperBoundRef(x)),
            index(LowerBoundRef(x)),
            index(UpperBoundRef(y)),
            index(LowerBoundRef(y)),
        ],
    )
    @test data[].metadata ==
          MathOptIIS.Metadata(11.0, 22.0, MOI.LessThan{Float64}(1.0))
    return
end

function test_range_equalto()
    model = Model()
    @variable(model, x == 1)
    @variable(model, y == 2)
    @constraint(model, c, x + y == 1)
    @objective(model, Max, x + y)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test _isequal_unordered(
        data[].constraints,
        [index(c), index(FixRef(x)), index(FixRef(y))],
    )
    @test data[].metadata ==
          MathOptIIS.Metadata(3.0, 3.0, MOI.EqualTo{Float64}(1.0))
    return
end

function test_range_equalto_2()
    model = Model()
    @variable(model, x == 1)
    @variable(model, y == 2)
    @constraint(model, c, 3x + 2y == 1)
    @objective(model, Max, x + y)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test _isequal_unordered(
        data[].constraints,
        [index(c), index(FixRef(x)), index(FixRef(y))],
    )
    @test data[].metadata ==
          MathOptIIS.Metadata(7.0, 7.0, MOI.EqualTo{Float64}(1.0))
    return
end

function test_range_greaterthan()
    model = Model()
    @variable(model, 10 <= x <= 11)
    @variable(model, 1 <= y <= 11)
    @constraint(model, c, x + y >= 100)
    @objective(model, Max, x + y)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test _isequal_unordered(
        data[].constraints,
        [
            index(c),
            index(UpperBoundRef(x)),
            index(LowerBoundRef(x)),
            index(UpperBoundRef(y)),
            index(LowerBoundRef(y)),
        ],
    )
    @test data[].metadata ==
          MathOptIIS.Metadata(11.0, 22.0, MOI.GreaterThan{Float64}(100.0))
    return
end

function test_range_equalto_3()
    model = Model()
    @variable(model, 10 <= x <= 11)
    @variable(model, 1 <= y <= 11)
    @constraint(model, c, x + y == 100)
    @objective(model, Max, x + y)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test _isequal_unordered(
        data[].constraints,
        [
            index(c),
            index(UpperBoundRef(x)),
            index(LowerBoundRef(x)),
            index(UpperBoundRef(y)),
            index(LowerBoundRef(y)),
        ],
    )
    @test data[].metadata ==
          MathOptIIS.Metadata(11.0, 22.0, MOI.EqualTo{Float64}(100.0))
    return
end

function test_interval()
    model = Model(HiGHS.Optimizer)
    set_silent(model)
    @variable(model, x in MOI.Interval(0, 10))
    @variable(model, 0 <= y <= 20)
    @constraint(model, c1, x + y <= 1)
    @objective(model, Max, x + y)
    optimize!(model)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.set(solver, MathOptIIS.InnerOptimizer(), HiGHS.Optimizer)
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 0
    return
end

function test_pass_attribute()
    model = Model(HiGHS.Optimizer)
    set_silent(model)
    @variable(model, 0 <= x <= 10)
    @variable(model, 0 <= y <= 20)
    @constraint(model, c1, x + y <= 1)
    @objective(model, Max, x + y)
    optimize!(model)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.set(solver, MathOptIIS.InnerOptimizer(), HiGHS.Optimizer)
    MOI.set(solver, MOI.TimeLimitSec(), 5.0)
    @test MOI.get(solver, MOI.TimeLimitSec()) == 5.0
    @test MOI.get(solver, MOI.Silent()) == true
    MOI.set(solver, MOI.Silent(), false)
    @test MOI.get(solver, MOI.Silent()) == false
    data = solver.results
    @test length(data) == 0
    return
end

function test_iis_feasible()
    model = Model(HiGHS.Optimizer)
    set_silent(model)
    @variable(model, 0 <= x <= 10)
    @variable(model, 0 <= y <= 20)
    @constraint(model, c1, x + y <= 1)
    @objective(model, Max, x + y)
    optimize!(model)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 0
    return
end

function test_iis()
    model = Model(HiGHS.Optimizer)
    set_silent(model)
    @variable(model, 0 <= x <= 10)
    @variable(model, 0 <= y <= 20)
    @constraint(model, c1, x + y <= 1)
    @constraint(model, c2, x + y >= 2)
    @objective(model, Max, x + y)
    optimize!(model)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 0
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.set(solver, MathOptIIS.InnerOptimizer(), HiGHS.Optimizer)
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].metadata === nothing
    @test _isequal_unordered(data[].constraints, [index(c2), index(c1)])
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    @test isempty(solver.results)
    @test solver.status == MOI.COMPUTE_CONFLICT_NOT_CALLED
    return
end

function test_pass_attribute_inner()
    model = Model(HiGHS.Optimizer)
    set_silent(model)
    @variable(model, 0 <= x <= 10)
    @variable(model, 0 <= y <= 20)
    @constraint(model, c1, x + y <= 1)
    @constraint(model, c2, x + y >= 2)
    @objective(model, Max, x + y)
    optimize!(model)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.set(
        solver,
        MathOptIIS.InnerOptimizer(),
        MOI.OptimizerWithAttributes(
            HiGHS.Optimizer,
            MOI.TimeLimitSec() => 10.0,
        ),
    )
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].metadata === nothing
    @test _isequal_unordered(data[].constraints, [index(c2), index(c1)])
    return
end

function test_iis_free_var()
    model = Model(HiGHS.Optimizer)
    set_silent(model)
    @variable(model, x)
    @variable(model, y)
    @constraint(model, c1, x + y <= 1)
    @constraint(model, c2, x + y >= 2)
    @objective(model, Max, -2x + y)
    optimize!(model)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.set(solver, MathOptIIS.InnerOptimizer(), HiGHS.Optimizer)
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].metadata === nothing
    @test _isequal_unordered(data[].constraints, [index(c2), index(c1)])
    return
end

function test_iis_multiple()
    model = Model(HiGHS.Optimizer)
    set_silent(model)
    @variable(model, 0 <= x <= 10)
    @variable(model, 0 <= y <= 20)
    @constraint(model, c1, x + y <= 1)
    @constraint(model, c3, x + y <= 1.5)
    @constraint(model, c2, x + y >= 2)
    @objective(model, Max, x + y)
    optimize!(model)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.set(solver, MathOptIIS.InnerOptimizer(), HiGHS.Optimizer)
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].metadata === nothing
    iis = data[].constraints
    @test length(iis) == 2
    @test Set(iis) ⊆ Set([index(c3), index(c2), index(c1)])
    @test index(c2) in Set(iis)
    return
end

function test_iis_interval_right()
    model = Model(HiGHS.Optimizer)
    set_silent(model)
    @variable(model, 0 <= x <= 10)
    @variable(model, 0 <= y <= 20)
    @constraint(model, c1, 0 <= x + y <= 1)
    @constraint(model, c2, x + y >= 2)
    @objective(model, Max, x + y)
    optimize!(model)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.set(solver, MathOptIIS.InnerOptimizer(), HiGHS.Optimizer)
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].metadata === nothing
    @test _isequal_unordered(data[].constraints, [index(c2), index(c1)])
    return
end

function test_iis_interval_left()
    model = Model(HiGHS.Optimizer)
    set_silent(model)
    @variable(model, 0 <= x <= 10)
    @variable(model, 0 <= y <= 20)
    @constraint(model, c1, x + y <= 1)
    @constraint(model, c2, 2 <= x + y <= 5)
    @objective(model, Max, x + y)
    optimize!(model)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.set(solver, MathOptIIS.InnerOptimizer(), HiGHS.Optimizer)
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].metadata === nothing
    @test _isequal_unordered(data[].constraints, [index(c2), index(c1)])
    return
end

function test_iis_spare()
    model = Model(HiGHS.Optimizer)
    set_silent(model)
    @variable(model, 0 <= x <= 10)
    @variable(model, 0 <= y <= 20)
    @variable(model, 0 <= z <= 20)
    @constraint(model, c0, 2z <= 1)
    @constraint(model, c00, 3z <= 1)
    @constraint(model, c1, x + y <= 1)
    @constraint(model, c2, x + y >= 2)
    @objective(model, Max, x + y)
    optimize!(model)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.set(solver, MathOptIIS.InnerOptimizer(), HiGHS.Optimizer)
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[1].metadata === nothing
    @test _isequal_unordered(data[].constraints, [index(c2), index(c1)])
    result = Dict(c1 => MOI.IN_CONFLICT, c2 => MOI.IN_CONFLICT)
    for ci in all_constraints(model; include_variable_in_set_constraints = true)
        @test MOI.get(solver, MOI.ConstraintConflictStatus(), index(ci)) ==
              get(result, ci, MOI.NOT_IN_CONFLICT)
    end
    return
end

function test_iis_binary()
    model = Model(HiGHS.Optimizer)
    set_silent(model)
    @variable(model, x, Bin)
    @constraint(model, c1, x == 1 / 2)
    optimize!(model)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.set(solver, MathOptIIS.InnerOptimizer(), HiGHS.Optimizer)
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].metadata === nothing
    @test _isequal_unordered(data[].constraints, [index(c1)])
    @test MOI.get(solver, MOI.ConstraintConflictStatus(), index(c1)) ==
          MOI.IN_CONFLICT
    @test MOI.get(
        solver,
        MOI.ConstraintConflictStatus(),
        index(BinaryRef(x)),
    ) == MOI.MAYBE_IN_CONFLICT
    @test MOI.get(solver, MathOptIIS.ListOfConstraintIndicesInConflict()) ==
          MOI.ConstraintIndex[index(c1), index(BinaryRef(x))]
    return
end

function test_verbose()
    model = Model(HiGHS.Optimizer)
    set_silent(model)
    @variable(model, 0 <= x <= 10)
    @variable(model, 0 <= y)
    @constraint(model, 2 * y <= 40)
    @constraint(model, c, x + y >= 35)
    @objective(model, Max, x + y)
    optimize!(model)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MOI.Silent(), false)
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.set(solver, MathOptIIS.InnerOptimizer(), HiGHS.Optimizer)
    dir = mktempdir()
    open(joinpath(dir, "log.stdout"), "w") do io
        return redirect_stdout(io) do
            return MOI.compute_conflict!(solver)
        end
    end
    contents = """
    [MathOptIIS] starting compute_conflict!
    [MathOptIIS]   model termination status: INFEASIBLE
    [MathOptIIS]   model primal status: NO_SOLUTION
    [MathOptIIS] starting bound analysis
    [MathOptIIS]   bound analysis found 0 infeasible subsets
    [MathOptIIS] starting range analysis
    [MathOptIIS]   analyzing MOI.ScalarAffineFunction{Float64} -in- MOI.GreaterThan{Float64}
    [MathOptIIS]   analyzing MOI.ScalarAffineFunction{Float64} -in- MOI.LessThan{Float64}
    [MathOptIIS]   range analysis found 0 infeasible subsets
    [MathOptIIS] starting elastic filter
    [MathOptIIS]   testing if we can relax integrality
    [MathOptIIS]     integrality is not required
    [MathOptIIS]   constructing the penalty relaxation
    [MathOptIIS]   using INFEASIBILITY_CERTIFICATE to construct candidate set
    [MathOptIIS]     size of the candidate set: 2
    [MathOptIIS]   starting the deletion filter
    [MathOptIIS]     size of the candidate set: 2
    [MathOptIIS]   elastic filter found 1 infeasible subsets
    """
    @test read(joinpath(dir, "log.stdout"), String) == contents
    return
end

function _copy_conflict(model::MOI.ModelLike, solver::MathOptIIS.Optimizer)
    filter_fn(::Any) = true
    function filter_fn(cref::MOI.ConstraintIndex)
        for i in 1:MOI.get(solver, MOI.ConflictCount())
            status = MOI.get(solver, MOI.ConstraintConflictStatus(i), cref)
            if status != MOI.NOT_IN_CONFLICT
                return true
            end
        end
        return false
    end
    new_model = MOI.Utilities.Model{Float64}()
    filtered_src = MOI.Utilities.ModelFilter(filter_fn, model)
    MOI.copy_to(new_model, filtered_src)
    MOI.set(new_model, MOI.ObjectiveSense(), MOI.FEASIBILITY_SENSE)
    return new_model
end

function _print_model(model)
    return replace(sprint(print, model), "-0.0" => "0.0")
end

function _test_compute_conflict(
    input,
    output;
    silent::Bool = true,
    optimizer = HiGHS.Optimizer,
    kwargs...,
)
    model = MOI.instantiate(optimizer; kwargs...)
    MOI.set(model, MOI.Silent(), true)
    MOI.Utilities.loadfromstring!(model, input)
    MOI.optimize!(model)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), model)
    MOI.set(solver, MathOptIIS.InnerOptimizer(), optimizer)
    MOI.set(solver, MOI.Silent(), silent)
    MOI.compute_conflict!(solver)
    @test MOI.get(solver, MOI.ConflictCount()) > 0
    @test MOI.get(solver, MOI.ConflictStatus()) == MOI.CONFLICT_FOUND
    iis = _copy_conflict(model, solver)
    target = MOI.Utilities.Model{Float64}()
    MOI.Utilities.loadfromstring!(target, output)
    A, B = _print_model(iis), _print_model(target)
    if A != B
        @info "IIS"
        print(A)
        @info "Target"
        println(B)
    end
    @test A == B
    return
end

function test_relax_integrality_integer()
    _test_compute_conflict(
        """
        variables: x, y
        maxobjective: 1.0 * x + y
        2.0 * y <= 40.0
        1.0 * x + y >= 35.0
        x >= 0.0
        x <= 10.0
        x in Integer()
        y >= 0.0
        """,
        """
        variables: x, y
        2.0 * y <= 40.0
        1.0 * x + y >= 35.0
        x <= 10.0
        """,
    )
    return
end

function test_relax_integrality_integer_continuous_infeasible()
    _test_compute_conflict(
        """
        variables: x
        maxobjective: 1.0 * x
        x in Integer()
        1.1 * x == 1.0
        """,
        """
        variables: x
        x in Integer()
        1.1 * x == 1.0
        """,
    )
    return
end

function test_relax_integrality_zero_one()
    _test_compute_conflict(
        """
        variables: x, y
        x in ZeroOne()
        x == 1.0
        1.0 * y <= 0.5
        1.0 * x + -1.0 * y == 0.0
        """,
        """
        variables: x, y
        x == 1.0
        1.0 * y <= 0.5
        1.0 * x + -1.0 * y == 0.0
        """,
    )
    _test_compute_conflict(
        """
        variables: x, y
        x in ZeroOne()
        x in Interval(0.3, 1.2)
        1.0 * y <= 0.5
        1.0 * x + -1.0 * y == 0.0
        """,
        """
        variables: x, y
        x in ZeroOne()
        x in Interval(0.3, 1.2)
        1.0 * y <= 0.5
        1.0 * x + -1.0 * y == 0.0
        """,
    )
    _test_compute_conflict(
        """
        variables: x, y
        x in ZeroOne()
        x in Interval(0.8, 1.2)
        1.0 * y <= 0.5
        1.0 * x + -1.0 * y == 0.0
        """,
        """
        variables: x, y
        x in Interval(0.8, 1.2)
        1.0 * y <= 0.5
        1.0 * x + -1.0 * y == 0.0
        """,
    )
    _test_compute_conflict(
        """
        variables: x, y
        x in ZeroOne()
        x >= 0.8
        1.0 * y <= 0.5
        1.0 * x + -1.0 * y == 0.0
        """,
        """
        variables: x, y
        x >= 0.8
        1.0 * y <= 0.5
        1.0 * x + -1.0 * y == 0.0
        """,
    )
    _test_compute_conflict(
        """
        variables: x, y
        x in ZeroOne()
        x <= 0.4
        1.0 * y >= 0.5
        1.0 * x + -1.0 * y == 0.0
        """,
        """
        variables: x, y
        x <= 0.4
        1.0 * y >= 0.5
        1.0 * x + -1.0 * y == 0.0
        """,
    )
    _test_compute_conflict(
        """
        variables: x, y
        x in ZeroOne()
        1.0 * y >= 1.5
        1.0 * x + -1.0 * y == 0.0
        """,
        """
        variables: x, y
        x in ZeroOne()
        1.0 * y >= 1.5
        1.0 * x + -1.0 * y == 0.0
        """,
    )
    _test_compute_conflict(
        """
        variables: x, y
        x in ZeroOne()
        1.0 * y <= -0.1
        1.0 * x + -1.0 * y == 0.0
        """,
        """
        variables: x, y
        x in ZeroOne()
        1.0 * y <= -0.1
        1.0 * x + -1.0 * y == 0.0
        """,
    )
    return
end

function test_model_with_bound_and_range_error()
    _test_compute_conflict(
        """
        variables: x
        x in ZeroOne()
        x == 2.0
        1.0 * x <= -1.0
        1.0 * x >= 0.0
        """,
        """
        variables: x
        x in ZeroOne()
        x == 2.0
        1.0 * x <= -1.0
        """,
    )
    return
end

function test_model_with_invalid_bound_and_range_error()
    _test_compute_conflict(
        """
        variables: x
        x in Interval(2.0, 1.0)
        1.0 * x <= -1.0
        1.0 * x >= 0.0
        """,
        """
        variables: x
        x in Interval(2.0, 1.0)
        1.0 * x <= -1.0
        """,
    )
    return
end

function test_check_interrupt()
    function _test_check_interrupt(err)
        return disable_sigint() do
            return MathOptIIS._check_interrupt(() -> throw(err))
        end
    end
    @test _test_check_interrupt(InterruptException()) == true
    @test_throws ArgumentError("") _test_check_interrupt(ArgumentError(""))
    return
end

function test_with_two_non_overlapping_iis()
    _test_compute_conflict(
        """
        variables: x, y
        1.0 * x <= -1.0
        1.0 * x >= 0.0
        1.0 * y <= -1.0
        1.0 * y >= 0.0
        """,
        # We return x, but it could equally be `y`. I'm not sure if this is
        # flakey or not, because it should depend on the order that we iterate
        # through the constraints.
        """
        variables: x, y
        1.0 * x <= -1.0
        1.0 * x >= 0.0
        """,
    )
    return
end

function test_a_large_iis_with_all_constraints()
    n = 97
    vars = join(["x_$i" for i in 1:n], ", ")
    model = """
    variables: $vars
    1.0 * x_1 + -1.0 * x_$n == 1.0
    """
    for i in 2:n
        model *= "1.0 * x_$(i-1) + -1.0 * x_$i == 0.0\n"
    end
    _test_compute_conflict(model, model)
    return
end

function test_enlight4()
    model = read_from_file(joinpath(@__DIR__, "data", "enlight4.mps"))
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.set(solver, MathOptIIS.InnerOptimizer(), HiGHS.Optimizer)
    MOI.set(solver, MOI.Silent(), false)
    MOI.compute_conflict!(solver)
    iis = _copy_conflict(backend(model), solver)
    F, S = MOI.ScalarAffineFunction{Float64}, MOI.EqualTo{Float64}
    @test MOI.get(iis, MOI.NumberOfConstraints{F,S}()) == 8
    return
end

# This problem is surprisingly difficult to solve the relaxed problem of. Mark
# has the instances.
function test_enlight9()
    model = read_from_file(joinpath(@__DIR__, "data", "enlight9.mps"))
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
    MOI.set(solver, MathOptIIS.InnerOptimizer(), HiGHS.Optimizer)
    MOI.set(solver, MOI.Silent(), false)
    MOI.set(solver, MOI.TimeLimitSec(), 10.0)
    MOI.compute_conflict!(solver)
    @test MOI.get(solver, MOI.ConflictStatus()) == MOI.NO_CONFLICT_FOUND
    return
end

function test_scs_basic()
    _test_compute_conflict(
        """
        variables: x
        1.0 * x <= -1.0
        1.0 * x >= 0.0
        """,
        """
        variables: x
        1.0 * x <= -1.0
        1.0 * x >= 0.0
        """;
        optimizer = SCS.Optimizer,
        with_bridge_type = Float64,
        with_cache_type = Float64,
    )
    return
end

function test_scs_no_deletion()
    _test_compute_conflict(
        """
        variables: x, y
        [1.0 * x, 1.0 * y] in Nonnegatives(2)
        [-1.0 * x + -1.0 * y + -1.0] in Nonnegatives(1)
        """,
        """
        variables: x, y
        [1.0 * x, 1.0 * y] in Nonnegatives(2)
        [-1.0 * x + -1.0 * y + -1.0] in Nonnegatives(1)
        """;
        optimizer = SCS.Optimizer,
        with_bridge_type = Float64,
        with_cache_type = Float64,
    )
    return
end

function test_scs_with_deletion()
    _test_compute_conflict(
        """
        variables: x, y
        [1.0 * x, 1.0 * y] in Nonnegatives(2)
        [-1.0 * x + -1.0 * y + -1.0] in Nonnegatives(1)
        1.0 * x + 1.0 * y >= 0.0
        """,
        """
        variables: x, y
        [1.0 * x, 1.0 * y] in Nonnegatives(2)
        [-1.0 * x + -1.0 * y + -1.0] in Nonnegatives(1)
        """;
        optimizer = SCS.Optimizer,
        with_bridge_type = Float64,
        with_cache_type = Float64,
    )
    return
end

function test_ipopt_quadratic()
    _test_compute_conflict(
        """
        variables: x, y
        1.0 * x * x + 1.0 * y <= 1.0
        x >= 2.0
        1.0 * y >= 0.0
        1.0 * x + 1.0 * y >= 0.0
        """,
        """
        variables: x, y
        1.0 * x * x + 1.0 * y <= 1.0
        x >= 2.0
        1.0 * y >= 0.0
        """;
        optimizer = Ipopt.Optimizer,
        with_cache_type = Float64,
        silent = false,
    )
    return
end

function test_ipopt_nonlinear()
    _test_compute_conflict(
        """
        variables: x, y
        ScalarNonlinearFunction(1.0 * x * x + 1.0 * y) <= 1.0
        x >= 2.0
        1.0 * y >= 0.0
        1.0 * x + 1.0 * y >= 0.0
        """,
        """
        variables: x, y
        ScalarNonlinearFunction(1.0 * x * x + 1.0 * y) <= 1.0
        x >= 2.0
        1.0 * y >= 0.0
        """;
        optimizer = Ipopt.Optimizer,
        with_cache_type = Float64,
        silent = false,
    )
    return
end

function test_get_variables()
    op(a, args...) = MOI.ScalarNonlinearFunction(a, Any[args...])
    x, y, z = MOI.VariableIndex.(1:3)
    for f in Any[
        # ScalarAffineFunction
        1.0*x+1.0*y,
        # ScalarQuadraticFunction
        1.0*x*x+1.0*y*y,
        1.0*x*x+1.0*y,
        1.0*x*y,
        # ScalarNonlinearFunction
        op(:+, x, y),
        op(:+, x, op(:-, y, 1.0)),
        op(:+, 1.0 * x + 1.0 * y),
        op(:+, x, 1.0 * y * y),
        op(:+, op(:log, x), op(:cos, y)),
        # AbstractVectorFunction
        MOI.Utilities.vectorize([1.0 * x, 1.0 * y]),
        MOI.Utilities.vectorize([1.0 * x * x, 1.0 * y * y]),
        MOI.Utilities.vectorize([op(:+, op(:log, x), op(:cos, y))]),
    ]
        vars = Set{MOI.VariableIndex}()
        MathOptIIS._get_variables!(vars, f)
        @test length(vars) == 2
        @test x in vars
        @test y in vars
        @test !(z in vars)
    end
    return
end

function test_zero_time_limit()
    model = HiGHS.Optimizer()
    MOI.set(model, MOI.Silent(), true)
    x = MOI.add_variables(model, 100_000)
    MOI.add_constraint.(model, x, MOI.GreaterThan(0.5))
    MOI.add_constraint.(model, x, MOI.ZeroOne())
    f = MOI.ScalarAffineFunction(MOI.ScalarAffineTerm.(1.0, x), 0.0)
    MOI.add_constraint(model, f, MOI.EqualTo(0.0))
    MOI.optimize!(model)
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), model)
    MOI.set(solver, MathOptIIS.InnerOptimizer(), HiGHS.Optimizer)
    MOI.set(solver, MOI.TimeLimitSec(), 0.0)
    MOI.compute_conflict!(solver)
    @test MOI.get(solver, MOI.ConflictStatus()) == MOI.NO_CONFLICT_FOUND
    return
end

function _solve_mock(mock)
    highs = HiGHS.Optimizer()
    MOI.set(highs, MOI.Silent(), true)
    index_map = MOI.Utilities.default_copy_to(highs, mock)
    MOI.optimize!(highs)
    status = MOI.get(highs, MOI.TerminationStatus())
    if status == MOI.OPTIMAL
        x = [index_map[xi] for xi in MOI.get(mock, MOI.ListOfVariableIndices())]
        sol = MOI.get(highs, MOI.VariablePrimal(), x)
        MOI.Utilities.mock_optimize!(mock, status, sol)
        obj = MOI.get(highs, MOI.ObjectiveValue())
        MOI.set(mock, MOI.ObjectiveValue(), obj)
    else
        MOI.Utilities.mock_optimize!(mock, status)
    end
    return
end

function _mock_optimizer(solver, N)
    # We want the time limit to be hit after N solves
    fns = convert(Vector{Any}, fill(_solve_mock, N))
    push!(fns, mock -> (solver.start_time = 0.0; _solve_mock(mock)))
    return () -> begin
        mock = MOI.Utilities.MockOptimizer(MOI.Utilities.Model{Float64}())
        MOI.Utilities.set_mock_optimize!(mock, fns...)
        return mock
    end
end

function test_time_limit_interrupt()
    model = read_from_file(joinpath(@__DIR__, "data", "enlight4.mps"))
    @testset "N=$N" for N in [5, 12, 13, 14, 15, 16]
        solver = MathOptIIS.Optimizer()
        MOI.set(solver, MathOptIIS.InfeasibleModel(), backend(model))
        MOI.set(solver, MOI.TimeLimitSec(), 100.0)
        MOI.set(solver, MathOptIIS.InnerOptimizer(), _mock_optimizer(solver, N))
        MOI.compute_conflict!(solver)
        status = MOI.get(solver, MOI.ConflictStatus())
        if status != MOI.NO_CONFLICT_FOUND
            @test status == MOI.CONFLICT_FOUND
            iis = _copy_conflict(backend(model), solver)
            F, S = MOI.ScalarAffineFunction{Float64}, MOI.EqualTo{Float64}
            # We've set N such that it cannot find the full IIS in the time
            # allowed.
            @test MOI.get(iis, MOI.NumberOfConstraints{F,S}()) > 8
        end
    end
    return
end

function test_time_limit_interrupt_with_zero_one()
    model = HiGHS.Optimizer()
    # We want a model where x in {0,1} is relaxed, and [0, 1] is problematic.
    # But also it's not infeasible based on the bounds.
    MOI.Utilities.loadfromstring!(
        model,
        """
        variables: x, y
        x in ZeroOne()
        1.0 * y == 0.25
        1.0 * x + 1.0 * y == 1.5
        """,
    )
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), model)
    MOI.set(solver, MOI.TimeLimitSec(), 100.0)
    MOI.set(solver, MathOptIIS.InnerOptimizer(), _mock_optimizer(solver, 3))
    MOI.compute_conflict!(solver)
    @test MOI.get(solver, MOI.ConflictStatus()) == MOI.CONFLICT_FOUND
    iis = _copy_conflict(model, solver)
    x = MOI.get(iis, MOI.VariableIndex, "x")
    @test MOI.is_valid(
        iis,
        MOI.ConstraintIndex{MOI.VariableIndex,MOI.ZeroOne}(x.value),
    )
    return
end

function test_scs_with_primal_dual_infeasibility()
    # The [x, y] in Nonnegatives(2) is redundant, but we can't tell that because
    # we can't easily relax it.
    _test_compute_conflict(
        """
        variables: x, y
        minobjective: -1.0 * y
        [x, y] in Nonnegatives(2)
        [1.0 * x + -1.0 * y, 1.0 * x + -1.0 * y + -1.0] in Zeros(2)
        """,
        """
        variables: x, y
        [x, y] in Nonnegatives(2)
        [1.0 * x + -1.0 * y, 1.0 * x + -1.0 * y + -1.0] in Zeros(2)
        """;
        optimizer = SCS.Optimizer,
        with_bridge_type = Float64,
        with_cache_type = Float64,
    )
    return
end

function test_empty()
    model = HiGHS.Optimizer()
    MOI.Utilities.loadfromstring!(
        model,
        """
        variables: x, y
        x in ZeroOne()
        1.0 * y == 0.25
        1.0 * x + 1.0 * y == 1.5
        """,
    )
    solver = MathOptIIS.Optimizer()
    MOI.set(solver, MathOptIIS.InfeasibleModel(), model)
    MOI.set(solver, MOI.TimeLimitSec(), 100.0)
    MOI.set(solver, MathOptIIS.InnerOptimizer(), _mock_optimizer(solver, 3))
    MOI.compute_conflict!(solver)
    @test MOI.get(solver, MOI.ConflictStatus()) == MOI.CONFLICT_FOUND
    MOI.empty!(solver)
    @test solver.infeasible_model === nothing
    @test isnan(solver.start_time)
    @test MOI.get(solver, MOI.ConflictStatus()) ==
          MOI.COMPUTE_CONFLICT_NOT_CALLED
    @test MOI.get(solver, MOI.ConflictCount()) == 0
    return
end

end # module

TestMathOptIIS.runtests()

# Copyright (c) 2025: Joaquim Dias Garcia, Oscar Dowson and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

module TestMathOptIIS

using JuMP
using Test

import HiGHS
import MathOptIIS as MOIIS

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
    @constraint(model, c, x + y <= 1)
    @objective(model, Max, x + y)
    solver = MOIIS.Optimizer()
    @test MOI.get(solver, MOI.ConflictStatus()) ==
          MOI.COMPUTE_CONFLICT_NOT_CALLED
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].constraints ==
          [index(LowerBoundRef(y)), index(UpperBoundRef(y))]
    @test data[].irreducible
    @test data[].metadata == MOIIS.BoundsData(2.0, 1.0)
    @test MOI.get(solver, MOI.ConflictStatus()) == MOI.CONFLICT_FOUND
    @test MOI.get(solver, MOI.ConstraintConflictStatus(), index(c)) ==
          MOI.NOT_IN_CONFLICT
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
    @test MOI.get(solver, MOIIS.ConflictCount()) == 1
    @test MOI.get(
        solver,
        MOIIS.ConstraintConflictStatus(1),
        index(LowerBoundRef(y)),
    ) == MOI.IN_CONFLICT
    @test MOI.get(
        solver,
        MOIIS.ConstraintConflictStatus(1),
        index(UpperBoundRef(y)),
    ) == MOI.IN_CONFLICT
    @test MOI.get(solver, MOIIS.ConstraintConflictStatus(1), index(c)) ==
          MOI.NOT_IN_CONFLICT
    # the next two could be errors
    @test MOI.get(
        solver,
        MOIIS.ConstraintConflictStatus(2),
        index(LowerBoundRef(y)),
    ) == MOI.NOT_IN_CONFLICT
    @test MOI.get(
        solver,
        MOIIS.ConstraintConflictStatus(2),
        index(UpperBoundRef(y)),
    ) == MOI.NOT_IN_CONFLICT
    #
    @test MOI.get(solver, MOIIS.ListOfConstraintIndicesInConflict(1)) ==
          [index(LowerBoundRef(y)), index(UpperBoundRef(y))]
    @test isempty(MOI.get(solver, MOIIS.ListOfConstraintIndicesInConflict(2)))
    return
end

function test_integrality()
    model = Model()
    @variable(model, 0 <= x <= 1, Int)
    @variable(model, 2.2 <= y <= 2.9, Int)
    @constraint(model, x + y <= 1)
    @objective(model, Max, x + y)
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].constraints == [
        index(IntegerRef(y)),
        index(LowerBoundRef(y)),
        index(UpperBoundRef(y)),
    ]
    @test data[].irreducible
    @test data[].metadata == MOIIS.IntegralityData(2.2, 2.9, MOI.Integer())
    return
end

function test_binary_inner()
    model = Model()
    @variable(model, 0.5 <= x <= 0.8, Bin)
    @variable(model, 0 <= y <= 1, Bin)
    @constraint(model, x + y <= 1)
    @objective(model, Max, x + y)
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].constraints == [
        index(BinaryRef(x)),
        index(LowerBoundRef(x)),
        index(UpperBoundRef(x)),
    ]
    @test data[].irreducible
    @test data[].metadata == MOIIS.IntegralityData(0.5, 0.8, MOI.ZeroOne())
    return
end

function test_binary_lower()
    model = Model()
    @variable(model, 1.5 <= x <= 1.8, Bin)
    @variable(model, 0 <= y <= 1, Bin)
    @constraint(model, x + y <= 1)
    @objective(model, Max, x + y)
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].constraints == [index(BinaryRef(x)), index(LowerBoundRef(x))]
    @test data[].irreducible
    @test data[].metadata == MOIIS.IntegralityData(1.5, Inf, MOI.ZeroOne())
    return
end

function test_binary_upper()
    model = Model()
    @variable(model, -2.5 <= x <= -1.8, Bin)
    @variable(model, 0 <= y <= 1, Bin)
    @constraint(model, x + y <= 1)
    @objective(model, Max, x + y)
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].constraints == [index(BinaryRef(x)), index(UpperBoundRef(x))]
    @test data[].irreducible
    @test data[].metadata == MOIIS.IntegralityData(-Inf, -1.8, MOI.ZeroOne())
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
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
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
    @test data[].irreducible
    @test data[].metadata ==
          MOIIS.RangeData(11.0, 22.0, MOI.LessThan{Float64}(1.0))
    return
end

function test_range_and_bound()
    model = Model()
    @variable(model, 10 <= x <= 11)
    @variable(model, 1 <= y <= 11)
    @variable(model, 1 <= z <= 0)
    @constraint(model, c, x + y <= 1)
    @objective(model, Max, x + y)
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test _isequal_unordered(
        data[].constraints,
        [index(LowerBoundRef(z)), index(UpperBoundRef(z))],
    )
    @test MOI.get(solver, MOIIS.StopIfInfeasibleBounds()) == true
    MOI.set(solver, MOIIS.StopIfInfeasibleBounds(), false)
    @test MOI.get(solver, MOIIS.StopIfInfeasibleBounds()) == false
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 2
    @test _isequal_unordered(
        data[1].constraints,
        [index(LowerBoundRef(z)), index(UpperBoundRef(z))],
    )
    @test _isequal_unordered(
        data[2].constraints,
        [
            index(c),
            index(UpperBoundRef(x)),
            index(LowerBoundRef(x)),
            index(UpperBoundRef(y)),
            index(LowerBoundRef(y)),
        ],
    )
    @test data[2].irreducible
    @test data[2].metadata ==
          MOIIS.RangeData(11.0, 22.0, MOI.LessThan{Float64}(1.0))
    @test MOI.get(solver, MOIIS.StopIfInfeasibleRanges()) == true
    MOI.set(solver, MOIIS.StopIfInfeasibleRanges(), false)
    @test MOI.get(solver, MOIIS.StopIfInfeasibleRanges()) == false
    MOI.set(solver, MOIIS.InnerOptimizer(), HiGHS.Optimizer)
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 2
    return
end

function test_range_and_bound_2()
    model = Model()
    @variable(model, 10 <= x <= 11)
    @variable(model, 1 <= y <= 11)
    @variable(model, 1 <= z <= 0)
    @constraint(model, c, x + y + z <= 1)
    @objective(model, Max, x + y)
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test _isequal_unordered(
        data[].constraints,
        [index(LowerBoundRef(z)), index(UpperBoundRef(z))],
    )
    @test MOI.get(solver, MOIIS.StopIfInfeasibleBounds()) == true
    MOI.set(solver, MOIIS.StopIfInfeasibleBounds(), false)
    @test MOI.get(solver, MOIIS.StopIfInfeasibleBounds()) == false
    MOI.compute_conflict!(solver)
    data = solver.results
    # the result is only one conflic again because the range fail cant be computed
    @test length(data) == 1
    @test _isequal_unordered(
        data[1].constraints,
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
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
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
    @test data[].irreducible
    @test data[].metadata ==
          MOIIS.RangeData(11.0, 22.0, MOI.LessThan{Float64}(1.0))
    return
end

function test_range_equalto()
    model = Model()
    @variable(model, x == 1)
    @variable(model, y == 2)
    @constraint(model, c, x + y == 1)
    @objective(model, Max, x + y)
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test _isequal_unordered(
        data[].constraints,
        [index(c), index(FixRef(x)), index(FixRef(y))],
    )
    @test data[].irreducible
    @test data[].metadata ==
          MOIIS.RangeData(3.0, 3.0, MOI.EqualTo{Float64}(1.0))
    return
end

function test_range_equalto_2()
    model = Model()
    @variable(model, x == 1)
    @variable(model, y == 2)
    @constraint(model, c, 3x + 2y == 1)
    @objective(model, Max, x + y)
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test _isequal_unordered(
        data[].constraints,
        [index(c), index(FixRef(x)), index(FixRef(y))],
    )
    @test data[].irreducible
    @test data[].metadata ==
          MOIIS.RangeData(7.0, 7.0, MOI.EqualTo{Float64}(1.0))
    return
end

function test_range_greaterthan()
    model = Model()
    @variable(model, 10 <= x <= 11)
    @variable(model, 1 <= y <= 11)
    @constraint(model, c, x + y >= 100)
    @objective(model, Max, x + y)
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
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
    @test data[].irreducible
    @test data[].metadata ==
          MOIIS.RangeData(11.0, 22.0, MOI.GreaterThan{Float64}(100.0))
    return
end

function test_range_equalto_3()
    model = Model()
    @variable(model, 10 <= x <= 11)
    @variable(model, 1 <= y <= 11)
    @constraint(model, c, x + y == 100)
    @objective(model, Max, x + y)
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
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
    @test data[].irreducible
    @test data[].metadata ==
          MOIIS.RangeData(11.0, 22.0, MOI.EqualTo{Float64}(100.0))
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
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    MOI.set(solver, MOIIS.InnerOptimizer(), HiGHS.Optimizer)
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 0
    @test !MOI.get(solver, MOIIS.SkipFeasibilityCheck())
    MOI.set(solver, MOIIS.SkipFeasibilityCheck(), true)
    @test MOI.get(solver, MOIIS.SkipFeasibilityCheck())
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 0
    # TODO check status
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
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    MOI.set(solver, MOIIS.InnerOptimizer(), HiGHS.Optimizer)
    MOI.set(solver, MOI.TimeLimitSec(), 5.0)
    @test MOI.get(solver, MOI.TimeLimitSec()) == 5.0
    @test MOI.get(solver, MOI.Silent()) == false
    MOI.set(solver, MOI.Silent(), true)
    @test MOI.get(solver, MOI.Silent()) == true
    @test MOI.get(solver, MOIIS.ElasticFilterTolerance()) == 1e-5
    MOI.set(solver, MOIIS.ElasticFilterTolerance(), 1e-3)
    @test MOI.get(solver, MOIIS.ElasticFilterTolerance()) == 1e-3
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 0
    MOI.set(solver, MOIIS.SkipFeasibilityCheck(), true)
    @test MOI.get(solver, MOIIS.SkipFeasibilityCheck())
    MOI.compute_conflict!(solver)
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
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
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
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 0
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    MOI.set(solver, MOIIS.InnerOptimizer(), HiGHS.Optimizer)
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].irreducible
    @test data[].metadata == MOIIS.NoData()
    @test _isequal_unordered(data[].constraints, [index(c2), index(c1)])
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    @test isempty(solver.results)
    @test solver.status == MOI.COMPUTE_CONFLICT_NOT_CALLED
    return
end

function test_iis_no_deletion_filter()
    model = Model(HiGHS.Optimizer)
    set_silent(model)
    @variable(model, 0 <= x <= 10)
    @variable(model, 0 <= y <= 20)
    @constraint(model, c1, x + y <= 1)
    @constraint(model, c2, x + y >= 2)
    @objective(model, Max, x + y)
    optimize!(model)
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 0
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    MOI.set(solver, MOIIS.InnerOptimizer(), HiGHS.Optimizer)
    @test MOI.get(solver, MOIIS.DeletionFilter()) == true
    MOI.set(solver, MOIIS.DeletionFilter(), false)
    @test MOI.get(solver, MOIIS.DeletionFilter()) == false
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].irreducible
    @test data[].metadata == MOIIS.NoData()
    @test _isequal_unordered(data[].constraints, [index(c2), index(c1)])
    return
end

function test_iis_ignore_integrality()
    model = Model(HiGHS.Optimizer)
    set_silent(model)
    @variable(model, 0 <= x <= 10)
    @variable(model, 0 <= y <= 20, Bin)
    @constraint(model, c1, x + y <= 1)
    @constraint(model, c2, x + y >= 2)
    @objective(model, Max, x + y)
    optimize!(model)
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 0
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    MOI.set(solver, MOIIS.InnerOptimizer(), HiGHS.Optimizer)
    @test MOI.get(solver, MOIIS.ElasticFilterIgnoreIntegrality()) == false
    MOI.set(solver, MOIIS.ElasticFilterIgnoreIntegrality(), true)
    @test MOI.get(solver, MOIIS.ElasticFilterIgnoreIntegrality()) == true
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].irreducible
    @test data[].metadata == MOIIS.NoData()
    @test _isequal_unordered(data[].constraints, [index(c2), index(c1)])
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
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    MOI.set(solver, MOIIS.InnerOptimizer(), HiGHS.Optimizer)
    MOI.set(solver, MOIIS.InnerOptimizerAttribute(MOI.TimeLimitSec()), 10.0)
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].irreducible
    @test data[].metadata == MOIIS.NoData()
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
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    MOI.set(solver, MOIIS.InnerOptimizer(), HiGHS.Optimizer)
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].irreducible
    @test data[].metadata == MOIIS.NoData()
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
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    MOI.set(solver, MOIIS.InnerOptimizer(), HiGHS.Optimizer)
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].irreducible
    @test data[].metadata == MOIIS.NoData()
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
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    MOI.set(solver, MOIIS.InnerOptimizer(), HiGHS.Optimizer)
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].irreducible
    @test data[].metadata == MOIIS.NoData()
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
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    MOI.set(solver, MOIIS.InnerOptimizer(), HiGHS.Optimizer)
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].irreducible
    @test data[].metadata == MOIIS.NoData()
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
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    MOI.set(solver, MOIIS.InnerOptimizer(), HiGHS.Optimizer)
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].irreducible
    @test data[].metadata == MOIIS.NoData()
    @test _isequal_unordered(data[].constraints, [index(c2), index(c1)])
    @test MOI.get(solver, MOI.ConstraintConflictStatus(), index(c0)) ==
          MOI.NOT_IN_CONFLICT
    @test MOI.get(solver, MOI.ConstraintConflictStatus(), index(c00)) ==
          MOI.NOT_IN_CONFLICT
    @test MOI.get(solver, MOI.ConstraintConflictStatus(), index(c1)) ==
          MOI.IN_CONFLICT
    @test MOI.get(solver, MOI.ConstraintConflictStatus(), index(c2)) ==
          MOI.IN_CONFLICT
    @test MOI.get(
        solver,
        MOI.ConstraintConflictStatus(),
        index(LowerBoundRef(x)),
    ) == MOI.MAYBE_IN_CONFLICT
    @test MOI.get(
        solver,
        MOI.ConstraintConflictStatus(),
        index(LowerBoundRef(y)),
    ) == MOI.MAYBE_IN_CONFLICT
    @test MOI.get(
        solver,
        MOI.ConstraintConflictStatus(),
        index(UpperBoundRef(x)),
    ) == MOI.MAYBE_IN_CONFLICT
    @test MOI.get(
        solver,
        MOI.ConstraintConflictStatus(),
        index(UpperBoundRef(y)),
    ) == MOI.MAYBE_IN_CONFLICT
    @test MOI.get(
        solver,
        MOI.ConstraintConflictStatus(),
        index(LowerBoundRef(z)),
    ) == MOI.NOT_IN_CONFLICT
    @test MOI.get(
        solver,
        MOI.ConstraintConflictStatus(),
        index(UpperBoundRef(z)),
    ) == MOI.NOT_IN_CONFLICT
    return
end

function test_iis_binary()
    model = Model(HiGHS.Optimizer)
    set_silent(model)
    @variable(model, x, Bin)
    @constraint(model, c1, x == 1 / 2)
    optimize!(model)
    @show termination_status(model)
    @show primal_status(model)
    solver = MOIIS.Optimizer()
    MOI.set(solver, MOIIS.InfeasibleModel(), backend(model))
    MOI.set(solver, MOIIS.InnerOptimizer(), HiGHS.Optimizer)
    MOI.compute_conflict!(solver)
    data = solver.results
    @test length(data) == 1
    @test data[].irreducible
    @test data[].metadata == MOIIS.NoData()
    @test _isequal_unordered(data[].constraints, [index(c1)])
    @test MOI.get(solver, MOI.ConstraintConflictStatus(), index(c1)) ==
          MOI.IN_CONFLICT
    @test MOI.get(
        solver,
        MOI.ConstraintConflictStatus(),
        index(BinaryRef(x)),
    ) == MOI.MAYBE_IN_CONFLICT
    return
end

end # module

TestMathOptIIS.runtests()

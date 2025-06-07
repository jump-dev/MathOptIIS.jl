# Copyright (c) 2025: Joaquim Dias Garcia, Oscar Dowson and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

module TestIIS

import HiGHS
using JuMP
import MathOptConflictSolver as MOCS
using Test

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
    @constraint(model, x + y <= 1)
    @objective(model, Max, x + y)
    data = MOCS.compute_conflicts(JuMP.backend(model))
    @test length(data) == 1
    @test data[].constraints ==
          [JuMP.index(LowerBoundRef(y)), JuMP.index(UpperBoundRef(y))]
    @test data[].irreducible
    @test data[].metadata == MOCS.BoundsData(2.0, 1.0)
    return
end

function test_integrality()
    model = Model()
    @variable(model, 0 <= x <= 1, Int)
    @variable(model, 2.2 <= y <= 2.9, Int)
    @constraint(model, x + y <= 1)
    @objective(model, Max, x + y)
    data = MOCS.compute_conflicts(JuMP.backend(model))
    @test length(data) == 1
    @test data[].constraints == [
        JuMP.index(IntegerRef(y)),
        JuMP.index(LowerBoundRef(y)),
        JuMP.index(UpperBoundRef(y)),
    ]
    @test data[].irreducible
    @test data[].metadata == MOCS.IntegralityData(2.2, 2.9, MOI.Integer())
    return
end

function test_binary_inner()
    model = Model()
    @variable(model, 0.5 <= x <= 0.8, Bin)
    @variable(model, 0 <= y <= 1, Bin)
    @constraint(model, x + y <= 1)
    @objective(model, Max, x + y)
    data = MOCS.compute_conflicts(JuMP.backend(model))
    @test length(data) == 1
    @test data[].constraints == [
        JuMP.index(BinaryRef(x)),
        JuMP.index(LowerBoundRef(x)),
        JuMP.index(UpperBoundRef(x)),
    ]
    @test data[].irreducible
    @test data[].metadata == MOCS.IntegralityData(0.5, 0.8, MOI.ZeroOne())
    return
end

function test_binary_lower()
    model = Model()
    @variable(model, 1.5 <= x <= 1.8, Bin)
    @variable(model, 0 <= y <= 1, Bin)
    @constraint(model, x + y <= 1)
    @objective(model, Max, x + y)
    data = MOCS.compute_conflicts(JuMP.backend(model))
    @test length(data) == 1
    @test data[].constraints ==
          [JuMP.index(BinaryRef(x)), JuMP.index(LowerBoundRef(x))]
    @test data[].irreducible
    @test data[].metadata == MOCS.IntegralityData(1.5, Inf, MOI.ZeroOne())
    return
end

function test_binary_upper()
    model = Model()
    @variable(model, -2.5 <= x <= -1.8, Bin)
    @variable(model, 0 <= y <= 1, Bin)
    @constraint(model, x + y <= 1)
    @objective(model, Max, x + y)
    data = MOCS.compute_conflicts(JuMP.backend(model))
    @test length(data) == 1
    @test data[].constraints ==
          [JuMP.index(BinaryRef(x)), JuMP.index(UpperBoundRef(x))]
    @test data[].irreducible
    @test data[].metadata == MOCS.IntegralityData(-Inf, -1.8, MOI.ZeroOne())
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
    data = MOCS.compute_conflicts(JuMP.backend(model))
    @test length(data) == 1
    @test _isequal_unordered(
        data[].constraints,
        [
            JuMP.index(c),
            JuMP.index(UpperBoundRef(x)),
            JuMP.index(LowerBoundRef(x)),
            JuMP.index(UpperBoundRef(y)),
            JuMP.index(LowerBoundRef(y)),
        ],
    )
    @test data[].irreducible
    @test data[].metadata ==
          MOCS.RangeData(11.0, 22.0, MOI.LessThan{Float64}(1.0))
    return
end

function test_range_neg()
    model = Model()
    @variable(model, 10 <= x <= 11)
    @variable(model, -11 <= y <= -1)
    @constraint(model, c, x - y <= 1)
    @objective(model, Max, x + y)
    #
    data = MOCS.compute_conflicts(JuMP.backend(model))
    @test length(data) == 1
    @test _isequal_unordered(
        data[].constraints,
        [
            JuMP.index(c),
            JuMP.index(UpperBoundRef(x)),
            JuMP.index(LowerBoundRef(x)),
            JuMP.index(UpperBoundRef(y)),
            JuMP.index(LowerBoundRef(y)),
        ],
    )
    @test data[].irreducible
    @test data[].metadata ==
          MOCS.RangeData(11.0, 22.0, MOI.LessThan{Float64}(1.0))
    return
end

function test_range_equalto()
    model = Model()
    @variable(model, x == 1)
    @variable(model, y == 2)
    @constraint(model, c, x + y == 1)
    @objective(model, Max, x + y)
    data = MOCS.compute_conflicts(JuMP.backend(model))
    @test length(data) == 1
    @test _isequal_unordered(
        data[].constraints,
        [JuMP.index(c), JuMP.index(FixRef(x)), JuMP.index(FixRef(y))],
    )
    @test data[].irreducible
    @test data[].metadata == MOCS.RangeData(3.0, 3.0, MOI.EqualTo{Float64}(1.0))
    return
end

function test_range_equalto_2()
    model = Model()
    @variable(model, x == 1)
    @variable(model, y == 2)
    @constraint(model, c, 3x + 2y == 1)
    @objective(model, Max, x + y)
    data = MOCS.compute_conflicts(JuMP.backend(model))
    @test length(data) == 1
    @test _isequal_unordered(
        data[].constraints,
        [JuMP.index(c), JuMP.index(FixRef(x)), JuMP.index(FixRef(y))],
    )
    @test data[].irreducible
    @test data[].metadata == MOCS.RangeData(7.0, 7.0, MOI.EqualTo{Float64}(1.0))
    return
end

function test_range_greaterthan()
    model = Model()
    @variable(model, 10 <= x <= 11)
    @variable(model, 1 <= y <= 11)
    @constraint(model, c, x + y >= 100)
    @objective(model, Max, x + y)
    data = MOCS.compute_conflicts(JuMP.backend(model))
    @test length(data) == 1
    @test _isequal_unordered(
        data[].constraints,
        [
            JuMP.index(c),
            JuMP.index(UpperBoundRef(x)),
            JuMP.index(LowerBoundRef(x)),
            JuMP.index(UpperBoundRef(y)),
            JuMP.index(LowerBoundRef(y)),
        ],
    )
    @test data[].irreducible
    @test data[].metadata ==
          MOCS.RangeData(11.0, 22.0, MOI.GreaterThan{Float64}(100.0))
    return
end

function test_range_equalto_3()
    model = Model()
    @variable(model, 10 <= x <= 11)
    @variable(model, 1 <= y <= 11)
    @constraint(model, c, x + y == 100)
    @objective(model, Max, x + y)
    data = MOCS.compute_conflicts(JuMP.backend(model))
    @test length(data) == 1
    @test _isequal_unordered(
        data[].constraints,
        [
            JuMP.index(c),
            JuMP.index(UpperBoundRef(x)),
            JuMP.index(LowerBoundRef(x)),
            JuMP.index(UpperBoundRef(y)),
            JuMP.index(LowerBoundRef(y)),
        ],
    )
    @test data[].irreducible
    @test data[].metadata ==
          MOCS.RangeData(11.0, 22.0, MOI.EqualTo{Float64}(100.0))
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
    data =
        MOCS.compute_conflicts(JuMP.backend(model), optimizer = HiGHS.Optimizer)
    @test length(data) == 0
    # TODO check status
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
    data = MOCS.compute_conflicts(JuMP.backend(model))
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
    data = MOCS.compute_conflicts(JuMP.backend(model))
    @test length(data) == 0
    data =
        MOCS.compute_conflicts(JuMP.backend(model), optimizer = HiGHS.Optimizer)
    @test length(data) == 1
    @test data[].irreducible
    @test data[].metadata == MOCS.NoData()
    @test _isequal_unordered(
        data[].constraints,
        [JuMP.index(c2), JuMP.index(c1)],
    )
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
    data =
        MOCS.compute_conflicts(JuMP.backend(model), optimizer = HiGHS.Optimizer)
    @test length(data) == 1
    @test data[].irreducible
    @test data[].metadata == MOCS.NoData()
    @test _isequal_unordered(
        data[].constraints,
        [JuMP.index(c2), JuMP.index(c1)],
    )
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
    data =
        MOCS.compute_conflicts(JuMP.backend(model), optimizer = HiGHS.Optimizer)
    @test length(data) == 1
    @test data[].irreducible
    @test data[].metadata == MOCS.NoData()
    iis = data[].constraints
    @test length(iis) == 2
    @test Set(iis) ⊆ Set([JuMP.index(c3), JuMP.index(c2), JuMP.index(c1)])
    @test JuMP.index(c2) in Set(iis)
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
    data =
        MOCS.compute_conflicts(JuMP.backend(model), optimizer = HiGHS.Optimizer)
    @test length(data) == 1
    @test data[].irreducible
    @test data[].metadata == MOCS.NoData()
    @test _isequal_unordered(
        data[].constraints,
        [JuMP.index(c2), JuMP.index(c1)],
    )
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
    data =
        MOCS.compute_conflicts(JuMP.backend(model), optimizer = HiGHS.Optimizer)
    @test length(data) == 1
    @test data[].irreducible
    @test data[].metadata == MOCS.NoData()
    @test _isequal_unordered(
        data[].constraints,
        [JuMP.index(c2), JuMP.index(c1)],
    )
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
    data =
        MOCS.compute_conflicts(JuMP.backend(model), optimizer = HiGHS.Optimizer)
    @test length(data) == 1
    @test data[].irreducible
    @test data[].metadata == MOCS.NoData()
    @test _isequal_unordered(
        data[].constraints,
        [JuMP.index(c2), JuMP.index(c1)],
    )
    return
end

end # module

TestIIS.runtests()

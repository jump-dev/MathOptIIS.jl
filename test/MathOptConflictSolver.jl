# Copyright (c) 2025: Joaquim Dias Garcia, Oscar Dowson and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

module TestMathOptConflictSolver

import MathOptConflictSolver
import MathOptInterface as MOI
using Test
using JuMP

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

function test_1()
    @test 1 == 1
    return
end

end # module TestMathOptConflictSolver

TestMathOptConflictSolver.runtests()
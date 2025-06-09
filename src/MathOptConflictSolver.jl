# Copyright (c) 2025: Joaquim Dias Garcia, Oscar Dowson and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

module MathOptConflictSolver

import MathOptInterface as MOI

include("iis.jl")
include("bound.jl")
include("range.jl")
include("solver.jl")

end # module MathOptConflictSolver

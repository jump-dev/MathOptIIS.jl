using Documenter, MathOptConflictSolver, JuMP

makedocs(; sitename = "MathOptConflictSolver.jl documentation")

deploydocs(;
    repo = "github.com/jump-dev/MathOptConflictSolver.jl.git",
    push_preview = true,
)
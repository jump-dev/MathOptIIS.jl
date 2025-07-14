using Documenter, MathOptIIS, JuMP

makedocs(; sitename = "MathOptIIS.jl documentation")

deploydocs(;
    repo = "github.com/jump-dev/MathOptIIS.jl.git",
    push_preview = true,
)

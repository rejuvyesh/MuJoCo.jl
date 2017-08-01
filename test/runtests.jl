using MuJoCo
using Base.Test

# write your own tests here
info("Testing General Mujoco Environment")
include("test_env.jl")
info("Testing Simple mjModel / mjData Access")
include("test_modeldata.jl")
if VERSION >= v"0.6"
   info("Testing Finite Differencing Derivatives")
   include("test_deriv.jl")
else
   warn("Derivatives supported in Julia v0.6")
end

info("Testing Name Access Method")
include("test_names.jl")

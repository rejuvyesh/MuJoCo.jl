using MuJoCo
using LinearAlgebra
using Printf
using Test

modelfile = joinpath(dirname(@__FILE__), "humanoid.xml")
#modelfile = dirname(@__FILE__)*"/swimmer.xml"

# write your own tests here
@info "Testing General Mujoco Environment"
include("test_env.jl")
@info "Testing Simple mjModel / mjData Access"
include("test_modeldata.jl")
#@info "Testing Finite Differencing Derivatives"
#include("test_deriv.jl")

#@info "Testing Name Access Method"
#include("test_names.jl")

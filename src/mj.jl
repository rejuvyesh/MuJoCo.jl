
__precompile__()
module mj
using Libdl
using StaticArrays

const depsfile = joinpath(dirname(@__FILE__), "..", "deps", "deps.jl")
if isfile(depsfile)
   include(depsfile)
else
   error("MuJoCo was not installed correctly.")
end

const VERSION_HEADER = 200

const mjtNum = Cdouble
const mjtByte = Cuchar

# mujoco header files in julia form
include("./mjmodel.jl")
include("./mjdata.jl")
include("./mjvisualize.jl")
include("./mjrender.jl")
include("./mjui.jl")

# additional structs and functionality
include("./mj_common.jl")
include("./mjextra.jl")

# mujoco functions
include("./mujoco_c.jl")
#include("./mjderiv.jl")

include("./export_all.jl") # a list of all function and struct names
export mjtNum, mjtByte
export jlData, jlModel
end

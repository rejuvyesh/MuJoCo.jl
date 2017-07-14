__precompile__()

module MuJoCo

depsfile = joinpath(dirname(@__FILE__), "..", "deps", "deps.jl")
if isfile(depsfile)
   include(depsfile)
else
   error("MuJoCo was not downloaded / installed correctly.")
end

const mjVERSION_HEADER = 150

const mjtNum = Cdouble
const mjtByte = Cuchar

# mujoco header files in julia form
include("./mjmodel.jl")
include("./mjdata.jl")
include("./mjvisualize.jl")
include("./mjrender.jl")

# additional structs and functionality
include("./mj_common.jl")
include("./mjextra.jl")

# mujoco functions
include("./mujoco.jl")

const mj = MuJoCo
export mj #export the module for faster typing
export mjData, mjModel, mjOption # types exported
export mjtNum, mjtByte

export jlData, jlModel

const keypath = findkey()
global activated = false

function teardown()
   deactivate()
   activated = false
end

function __init__()
   val = activate(keypath)
   if val == 1
      println("MuJoCo Activated")
      activated = true
   else
      warn("MuJoCo not activated. Could not find license file in MUJOCO_KEY_PATH environment variable or through system search.")
   end
   atexit(teardown)
end

# Notes on notation:
# in this module, m and d are mjModel and mjData respectively
# outside the module, the raw c struct pointer is generally pm, pd


#TODO jlModel can try to display unallocated raw pointers which causes segfault
function display(m::jlModel)
   #println("MuJoCo.jlModel(", m.m,")")
   println("MuJoCo.jlModel()")
end
export display

end

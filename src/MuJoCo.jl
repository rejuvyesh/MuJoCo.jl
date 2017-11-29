__precompile__()

module MuJoCo

using StaticArrays

const depsfile = joinpath(dirname(@__FILE__), "..", "deps", "deps.jl")
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
include("./mujoco_c.jl")
if VERSION >= v"0.6"
   include("./mjderiv.jl")
else
   warn("Derivatives supported in Julia v0.6")
end

const mj = MuJoCo
export mj #export the module for faster typing
export mjData, mjModel, mjOption # types exported
export mjtNum, mjtByte

export jlData, jlModel

global activated = false

function teardown()
   deactivate()
   global activated = false
end

function start()
   key = ""
   try
      key = ENV["MUJOCO_KEY_PATH"]
   catch e
      if is_linux()
         keys = split(readstring(run(`locate mjkey.txt`)), "\n")
         key = keys[1]
      else
         println("Set MUJOCO_KEY_PATH environment variable, please.")
      end
   end
   val = activate(key)

   if val == 1
      println("MuJoCo Activated")
      global activated = true
   else
      warn("MuJoCo not activated. Could not find license file in MUJOCO_KEY_PATH environment variable or through system search.")
   end
end

function __init__()
   if is_linux()
      Libdl.dlopen_e(libglew, Libdl.RTLD_LAZY | Libdl.RTLD_DEEPBIND | Libdl.RTLD_GLOBAL)
   end

   start()

   atexit(teardown)
end


end

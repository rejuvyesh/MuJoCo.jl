__precompile__()

module MuJoCo

using Reexport
using Libdl

const depsfile = joinpath(dirname(@__FILE__), "..", "deps", "deps.jl")
if isfile(depsfile)
   include(depsfile)
else
   error("MuJoCo was not installed correctly.")
end


@reexport module mj
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

#const mj = MuJoCo
#export mj #export the module for faster typing

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
#export mjv, mjr, mju
#export mjtNum, mjtByte
#export jlData, jlModel
end

include("./export_all.jl") # a list of all function and struct names

function teardown()
   deactivate()
end

function __init__()
   if Sys.islinux()
      Libdl.dlopen_e(libglew, Libdl.RTLD_LAZY | Libdl.RTLD_DEEPBIND | Libdl.RTLD_GLOBAL)
   end

   key = ""
   try
      key = ENV["MUJOCO_KEY_PATH"]
      cmd = "ccall((:mj_activate,libmujoco),Cint,(Cstring,),\"$(key)\")"
      eval(Meta.parse(cmd))
   catch e
      println("Set MUJOCO_KEY_PATH environment variable, please.")
   end

   atexit(teardown)
end

end

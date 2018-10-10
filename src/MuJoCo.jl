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

include("./mj.jl")
@reexport using .mj

include("./export_all.jl") # a list of all function and struct names

function teardown()
   mj_deactivate()
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

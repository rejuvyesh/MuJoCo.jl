module MuJoCo

const mjVERSION_HEADER = 141

const mjtNum = Cdouble
const mjtByte = Cuchar

# mujoco header files in julia form
include("./mjmodel.jl")
include("./mjdata.jl")
include("./mjvisualize.jl")
include("./mjrender.jl")
include("./mjoptim.jl")

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
activate(keypath)

# Notes on notation:
# in this module, m and d are mjModel and mjData respectively
# outside the module, the raw c struct pointer is generally pm, pd


Base.atexit(deactivate)

end

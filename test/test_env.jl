using MuJoCo

############################################################# mujoco startup

#val = mj.activate(ENV["MUJOCO_KEY_PATH"])
@test mj.activated == true # should be activated on module load

pm = mj.loadXML(modelfile, C_NULL)

@test pm != nothing

pd = mj.makeData(pm)

@test typeof(pd) == Ptr{mj.Data}

m = unsafe_load(pm)
d = unsafe_load(pd)

@test m.nq == 28
@test m.nv == 27
@test m.nbody == 14
@test m.opt.timestep == 0.002


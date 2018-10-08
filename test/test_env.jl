using MuJoCo

############################################################# mujoco startup

pm = mj_loadXML(modelfile, C_NULL)

@test pm != nothing

pd = mj_makeData(pm)

@test typeof(pd) == Ptr{mjData}

m = unsafe_load(pm)
d = unsafe_load(pd)

@test m.nq == 28
@test m.nv == 27
@test m.nbody == 14
@test m.opt.timestep == 0.002


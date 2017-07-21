using MuJoCo

############################################################# mujoco startup

#val = mj.activate(ENV["MUJOCO_KEY_PATH"])
@test mj.activated == true # should be activated on module load

modelfile = dirname(@__FILE__)*"/humanoid.xml"
pm = mj.loadXML(modelfile, "")

@test pm != nothing

pd = mj.makeData(pm)

@test typeof(pd) == Ptr{mjData}

m = unsafe_load(pm)
d = unsafe_load(pd)

@test m.nq == 28
@test m.nv == 27
@test m.nbody == 14
@test m.opt.timestep == 0.002


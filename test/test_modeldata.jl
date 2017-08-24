

############################################################# map to julia
m, d  = mj.mapmujoco(pm, pd)

@test mj.get(m, :nq) == 28
@test mj.get(m, :nv) == 27
@test mj.get(m, :nbody) == 14
@test mj.get(m, :opt, :timestep) == 0.002

mj.set(m, :nq, -28)
mj.set(m, :nv, -27)
mj.set(m, :nbody, -14)
mj.set(m, :opt, :timestep, -0.002)

@test mj.get(m, :nq) == -28
@test mj.get(m, :nv) == -27
@test mj.get(m, :nbody) == -14
@test mj.get(m, :opt, :timestep) == -0.002

mj.set(m, :nq, 28)
mj.set(m, :nv, 27)
mj.set(m, :nbody, 14)
mj.set(m, :opt, :timestep, 0.002)

@test mj.get(m, :nq) == 28
@test mj.get(m, :nv) == 27
@test mj.get(m, :nbody) == 14
@test mj.get(m, :opt, :timestep) == 0.002

############################################################# mujoco basics

qpos0 = copy(d.qpos)

mj.step(m, d)

@test d.qpos != qpos0
@test mj.get(d, :time) == 0.002

mj.set(m, :opt, :timestep, -0.002) # backwards step
mj.step(m, d)

@test isapprox(qpos0, d.qpos; atol = 1e-3)
@test mj.get(d, :time) == 0.0

mj.set(d, :time, -100.0)
@test mj.get(d, :time) == -100.0

mj.resetData(m, d)

@test qpos0 == d.qpos
@test mj.get(d, :time) == 0.00


mj.deleteModel(m.m)
mj.deleteData(d.d)




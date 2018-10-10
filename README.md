# MuJoCo.jl
Julia wrapper for the MuJoCo Physics Engine. This wrapper tries to both keep full access to all MuJoCo functionality through C pointers while also allow faster development through higher level interface.

MuJoCo functions and structs retain their C-library names (e.g. mj_step, mju_copy), while enums are scoped to the module name of mj (e.g. mj.NGROUP, etc). MuJoCo C-library functions can be passed a pointer to the mjModel or mjData structs, or can be passed a convenience wrapper struct of jlModel, jlData.

```
using MuJoCo

act = mj_activate(ENV["MUJOCO_KEY_PATH"])

modelfile = "humanoid.xml"
pm = mj_loadXML(modelfile)  # Raw C pointer to mjModel
pd = mj_makeData(pm)            # Raw C pointer to mjData

mj_step(pm, pd) # At this point you can pass the pointers to mujoco functions

# Access static Model and Data fields
m = unsafe_load(pm)
println(m.nq, " position elements")
d = unsafe_load(pd)
println(d.qpos, " is a raw pointer still")
```

Higher level interfacing tries to make it easier to manipulate data in the mjModel and mjData structures through Julia types. We do this by wrapping mjModel & mjData with jlModel and jlData types that expose ```Vector{Float64}``` instead of MuJoCo's raw arrays.

```
m, d = mj.mapmujoco(pm, pd) # wrap with our jlModel, jlData types

# we can manipulate data in the raw C structs now
nq = mj.get(m, :nq)
mj.set(m, :nq, -28)
@assert mj.get(m, :nq) == -28

mj.set(m, :opt, :timestep, -0.002) # we can traverse structs-within-structs
@assert mj.get(m, :opt, :timestep) == -0.002

d.qpos[:] = rand(nq) # d.qpos is a jlData Vector; free to access and maps to raw pointer

# some functions work on the jlModel and jlData types
mj_step(m, d)
mj_resetData(m, d)

mj_step(m.m, d.d) # our wrapped functions can take in the convenience struct or the raw pointers
```

# Installation

```
julia> Pkg.clone("git://www.github.com/klowrey/MuJoCo.jl.git")
julia> Pkg.build("MuJoCo")
```
MuJoCo v2.00 should be installed automatically through Julia Pkg. You will need a mjkey.txt license file, and your system should set the environment variable "MUJOCO_KEY_PATH" to be the path to your mjkey.txt file.

Currently, this package is untested in Windows, or OSX.

# Examples
Temporary examples can be found in test suite.


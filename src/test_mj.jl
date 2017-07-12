#include("./mujoco_common.jl")
#include("./mj_common.jl")
include("./mujoco.jl")

using MuJoCo

############################################################# mujoco startup

val = mj.activate("/home/klowrey/.mujoco/mjkey.txt")
println("Acivation: $val")

modelfile = "/home/klowrey/Dropbox/robotics/mjulia/models/humanoid.xml"
pm = mj.loadXML(modelfile, "") #, "", 0)

if pm == nothing 
   println("Bad load")
   quit()
end

pd = mj.makeData(pm)


m = unsafe_load(pm)
d = unsafe_load(pd)


#for i=1:100
#   mj.step(pm, pd)
#end

function data(pd, field::Symbol, value)
   d = unsafe_load(pd) # memory copy, without pointers
   setfield!(d, field, value)
end

function data(pd, field::Symbol)
   d = unsafe_load(pd) # memory copy, without pointers
   return getfield(d, field)
end


# function for struct infos
#structinfo(T) = [(fieldoffset(T,i), fieldname(T,i), fieldtype(T,i)) for i = 1:nfields(T)]
structinfo(T) = Dict(fieldname(T,i)=>(fieldoffset(T,i),  fieldtype(T,i)) for i = 1:nfields(T))

dinfo = structinfo(mjData)
minfo = structinfo(mjModel)
oinfo = structinfo(mjOption)

optionoffset = minfo[:opt][1] # bit offset
viscosityoffset = oinfo[:viscosity][1] # bit offset

# sets viscosity in model.options
#unsafe_store!(convert(Ptr{mjtNum}, (pm + optionoffset + viscosityoffset)), 1234.5678)

mtypes = [fieldtype(mjModel, i) for i=1:nfields(mjModel)]
dtypes = [fieldtype(mjData, i) for i=1:nfields(mjData)]



# BROKEN::
function update_model(m::mjModel, field::Symbol, val::mjtNum)
   setfield!(m, field, val)
   return m
end
function update_model(m::mjModel, field::Symbol, val::Integer)
   setfield!(m, field, convert(Cint, val))
   return m
end

# convert to macro?
#function update_model(pm::Ptr{mjModel}, field::Symbol, val::Integer)
#   f_off, f_type = minfo[field] # get offset & type
#   #@assert f_type == typeof(val)
#   # pointer math in julia is bytes, not of type offsets
#   unsafe_store!(convert(Ptr{f_type}, (pm + f_off)), convert(Cint, val))
#end

# pointer math in julia is bytes, not of type offsets
function update_model(pm::Ptr{mjModel}, offset::Integer, val::Integer)
   unsafe_store!(convert(Ptr{Cint}, (pm + offset)), convert(Cint, val))
end
function update_model(pm::Ptr{mjtNum}, offset::Integer, val::mjtNum)
   unsafe_store!(convert(Ptr{mjtNum}, (pm + offset)), val)
end

function update_model(pm::Ptr{mjModel}, field::Symbol, val::Union{Integer, mjtNum})
   f_off, f_type = minfo[field] # get offset & type
   @assert f_type == typeof(val) || f_type == typeof(convert(Cint, val))

   update_model(pm, f_off, val)

end

function mutatemodel!(pm) 
   m = unsafe_load(pm)
   update_model(m, :nq, 222)
   unsafe_store!(pm, m)
end


##############

function bad_map_data(pd::Ptr{mjData}) 
   # FUCKING NOPE
   c_data = unsafe_load(pd)
   mydata = unsafe_load( Ptr{jlData}(Libc.malloc(sizeof(jlData))) ) # make our data
   mydata.d = pd

   ptr_fields = intersect( fieldnames(jlData), fieldnames(mjData) )

   for f in ptr_fields
      #qpos = unsafe_wrap(Array, d.qpos, m.nq)
      mydata.f = unsafe_wrap(Array, c_data.f, f_size[f])
   end
end

function map_data(pm::Ptr{mjModel}, pd::Ptr{mjData}) 
   c_data = unsafe_load(pd)
   #mydata = unsafe_load( Ptr{jlData}(Libc.malloc(sizeof(jlData))) ) # make our data
   #mydata.d = pd
   args = Array{Any}(1)
   args[1] = pd

   ptr_fields = intersect( fieldnames(jlData), fieldnames(mjData) )

   f_size = mj.getdatasize(unsafe_load(pm), unsafe_load(pd))

   for f in ptr_fields
      push!(args, unsafe_wrap(Array, getfield(c_data, f), f_size[f]) )
   end
   return jlData(args...)
end

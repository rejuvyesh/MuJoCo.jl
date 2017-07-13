
# find the Mujoco key txt file for free or paid license.
function findkey()
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

   return key
end

# returns a julia centric version of mujoco's model and data fields
# that allows direct access to Ptr array fields as julia vectors
function mapmodel(pm::Ptr{mjModel})
   c_model= unsafe_load(pm)

   margs = Array{Any}(1)
   margs[1] = pm
   m_fields = intersect( fieldnames(jlModel), fieldnames(mjModel) )
   m_sizes = mj.getmodelsize(c_model)
   for f in m_fields
      #println(f)
      # TODO get field type and unsafe_wrap array or string
      push!(margs, unsafe_wrap(Array, getfield(c_model, f), m_sizes[f]) )
   end
   return jlModel(margs...)
end

function mapdata(pm::Ptr{mjModel}, pd::Ptr{mjData}) 
   c_model= unsafe_load(pm)
   c_data = unsafe_load(pd)
   
   dargs = Array{Any}(1)
   dargs[1] = pd
   d_fields = intersect( fieldnames(jlData), fieldnames(mjData) )
   d_sizes = mj.getdatasize(c_model, c_data)
   for f in d_fields
      push!(dargs, unsafe_wrap(Array, getfield(c_data, f), d_sizes[f]) )
   end
   return jlData(dargs...)
end

function mapmujoco(pm::Ptr{mjModel}, pd::Ptr{mjData}) 
   return mapmodel(pm), mapdata(pm, pd)
end


# struct manipulation and access

structinfo(T) = Dict(fieldname(T,i)=>(fieldoffset(T,i),  fieldtype(T,i)) for i = 1:nfields(T))
minfo = structinfo(mjModel)
dinfo = structinfo(mjData)
oinfo = structinfo(mjOption)
vinfo = structinfo(mjVisual)
sinfo = structinfo(mjStatistic)
cinfo = structinfo(mjContact)

function moffset(fstruct::Symbol, field::Symbol)
end

# access mujoco struct fields through the julia version of model and data
function get(m::jlModel, field::Symbol)
   f_off, f_type = minfo[field]
   pntr = Ptr{f_type}(m.m)
   return unsafe_load(pntr+f_off, 1)
end

function get(m::jlModel, fstruct::Symbol, field::Symbol)
   s_off, s_type = minfo[fstruct]
   @assert s_type in (MuJoCo._mjOption, MuJoCo._mjVisual, MuJoCo._mjStatistic)

   f_off, f_type = structinfo(s_type)[field]
   pntr = Ptr{f_type}(m.m)
   return unsafe_load(pntr+s_off+f_off, 1)
end

function get(m::jlData, field::Symbol)
end

# TODO cleanup these asserts

function update_ptr(p::Ptr, offset::Integer, val::Integer)
   unsafe_store!(convert(Ptr{Cint}, (p + offset)), convert(Cint, val))
end
function update_ptr(p::Ptr, offset::Integer, val::mjtNum)
   unsafe_store!(convert(Ptr{mjtNum}, (p + offset)), val)
end


# mutate mujoco struct fields through the julia version of model and data
function set(m::jlModel, field::Symbol, val::Union{Integer, mjtNum})
   f_off, f_type = minfo[field]
   #@assert isa(typeof(val), Int64) || isa(typeof(val), Int32) || isa(typeof(val), mjtNum)
   update_ptr(m.m, f_off, convert(f_type, val)) 
end

# set struct within mjmodel struct 
function set(m::jlModel, fstruct::Symbol, field::Symbol, val::Union{Integer, mjtNum})
   s_off, s_type = minfo[fstruct]
   @assert s_type in (MuJoCo._mjOption, MuJoCo._mjVisual, MuJoCo._mjStatistic)

   f_off, f_type = structinfo(s_type)[field]
   #@assert isa(typeof(val), Int64) || isa(typeof(val), Int32) || isa(typeof(val), mjtNum)
   update_ptr(m.m, s_off+f_off, convert(f_type, val))
end


function set(d::jlData, field::Symbol, val::Union{Integer, mjtNum})
   f_off, f_type = dinfo[field]
   #@assert isa(typeof(val), Int64) || isa(typeof(val), Int32) || isa(typeof(val), mjtNum)
   update_ptr(d.d, f_off, convert(f_type, val)) 
end




#################################### helpers

function step(m::jlModel, d::jlData)
   step(m.m, d.d)
end

function resetData(m::jlModel, d::jlData)
   resetData(m.m, d.d)
end

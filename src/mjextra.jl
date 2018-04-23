
# returns a julia centric version of mujoco's model and data fields
# that allows direct access to Ptr array fields as julia vectors
function mapmodel(pm::Ptr{Model})
   c_model= unsafe_load(pm)

   margs = Array{Any}(1)
   margs[1] = pm
   m_fields = intersect( fieldnames(jlModel), fieldnames(Model) )
   m_sizes = getmodelsize(c_model)
	jminfo = structinfo(jlModel)
	maxmodelmemptr = convert(UInt64, getfield(c_model, :names))
   for f in m_fields
		adr = convert(UInt64, getfield(c_model, f))
		if adr == 0x0  || adr > maxmodelmemptr # bad pointer
			m_off, m_type = jminfo[f]
			push!(margs, m_type(0) )
		else
         len = m_sizes[f][1] * m_sizes[f][2]
         raw = unsafe_wrap(Array, getfield(c_model, f), len)

         if m_sizes[f][2] > 1
            raw = reshape(raw, reverse(m_sizes[f]) )
         end
         push!(margs, raw)
		end
   end
   return jlModel(margs...)
end

function mapdata(pm::Ptr{Model}, pd::Ptr{Data}) 
   c_model= unsafe_load(pm)
   c_data = unsafe_load(pd)
   
   dargs = Array{Any}(1)
   dargs[1] = pd
   d_fields = intersect( fieldnames(jlData), fieldnames(Data) )
   d_sizes = getdatasize(c_model, c_data)
   for f in d_fields
      len = d_sizes[f][1] * d_sizes[f][2]
      raw = unsafe_wrap(Array, getfield(c_data, f), len)
      if d_sizes[f][2] > 1
         raw = reshape(raw, reverse(d_sizes[f]) )
      end
      push!(dargs, raw)
   end
   return jlData(dargs...)
end

function mapmujoco(pm::Ptr{Model}, pd::Ptr{Data}) 
   return mapmodel(pm), mapdata(pm, pd)
end

# struct manipulation and access
structinfo(T) = Dict(fieldname(T,i)=>(fieldoffset(T,i), fieldtype(T,i)) for i = 1:nfields(T))
const minfo = structinfo(Model)
const dinfo = structinfo(Data)

const mjstructs = Dict(Contact     => structinfo(Contact),
                       WarningStat => structinfo(WarningStat),
                       TimerStat   => structinfo(TimerStat),
                       SolverStat  => structinfo(SolverStat),
                                        
                       mjrContext    => structinfo(mjrContext),
                                        
                       VFS         => structinfo(VFS),
                       Option      => structinfo(Option),
                       #_global       => structinfo(_global),
                       #quality       => structinfo(quality),
                       #headlight     => structinfo(headlight),
                       #map           => structinfo(map),
                       #scale         => structinfo(scale),
                       #rgba          => structinfo(rgba),
                       Visual      => structinfo(Visual),
                       Statistic   => structinfo(Statistic),
                       Model       => structinfo(Model),
                                        
                       mjvPerturb    => structinfo(mjvPerturb),
                       mjvCamera     => structinfo(mjvCamera),
                       mjvGLCamera   => structinfo(mjvGLCamera),
                       mjvGeom       => structinfo(mjvGeom),
                       mjvLight      => structinfo(mjvLight),
                       mjvOption     => structinfo(mjvOption),
                       mjvScene      => structinfo(mjvScene),
                       mjvFigure     => structinfo(mjvFigure))

# access mujoco struct fields through the julia version of model and data
function get(m::jlModel, field::Symbol)
   f_off, f_type = minfo[field]
   pntr = Ptr{f_type}(m.m)
   return unsafe_load(pntr+f_off, 1)
end
function get(m::jlModel, fstruct::Symbol, field::Symbol)
   s_off, s_type = minfo[fstruct]
   @assert s_type in (MuJoCo.Option, MuJoCo.Visual, MuJoCo.Statistic)

   f_off, f_type = structinfo(s_type)[field]
   pntr = Ptr{f_type}(m.m)
   return unsafe_load(pntr+s_off+f_off, 1)
end
function get(d::jlData, field::Symbol)
   f_off, f_type = dinfo[field]
   pntr = Ptr{f_type}(d.d)
   return unsafe_load(pntr+f_off, 1)
end
function get{T<:Any}(p::Ptr{T}, field::Symbol)
   f_off, f_type = mjstructs[T][field]
   pntr = Ptr{f_type}(p)
   return unsafe_load(pntr+f_off, 1)
end
function get{T<:Any}(p::Ptr{T}, field::Symbol, i::Int)
   f_off, f_type = mjstructs[T][field]
   ET = eltype(f_type)
   @assert f_type <: SVector
   pntr = Ptr{ET}(p)
   unsafe_load(pntr+f_off, i)
end
function get{T<:Any}(p::Ptr{T}, field::Symbol, i::Int, j::Int) # does row-col conversion
   f_off, f_type = mjstructs[T][field]
	ET = eltype(eltype(f_type))
	@assert f_type <: SVector && eltype(f_type) <: SVector
   #r, c = size(f_type)
	c = sizeof(eltype(f_type))[1]
   #@assert i <= r && i >= 1
   #@assert j <= c && i >= 1
	#idx = (i-1)*c + (j-1)*sizeof(ET)
	idx = (j-1)*sizeof(ET)
	pntr = Ptr{ET}(p)
	unsafe_load(pntr+f_off + (i-1)*c + (j-1)*sizeof(ET), 1)
end


function update_ptr(p::Ptr, offset::Integer, val::Integer)
   unsafe_store!(convert(Ptr{Cint}, (p + offset)), convert(Cint, val))
end
function update_ptr(p::Ptr, offset::Integer, val::mjtNum)
   unsafe_store!(convert(Ptr{mjtNum}, (p + offset)), val)
end
function update_ptr(p::Ptr, offset::Integer, val::SVector)
   #T = eltype(SVector)
   T = typeof(val[1])
   for i=1:length(val)
      unsafe_store!(convert(Ptr{T}, p+offset+(i-1)*sizeof(T)),
                    val[i])
   end
end

# mutate mujoco struct fields through the julia version of model and data
function set(d::jlData, field::Symbol, val::Union{Integer, mjtNum})
   f_off, f_type = dinfo[field]
   update_ptr(d.d, f_off, convert(f_type, val)) 
end
function set(m::jlModel, field::Symbol, val::Union{Integer, mjtNum})
   f_off, f_type = minfo[field]
   update_ptr(m.m, f_off, convert(f_type, val)) 
end
function set{T<:Any}(p::Ptr{T}, field::Symbol, val::Union{Integer, mjtNum, SVector})
   f_off, f_type = mjstructs[T][field]
   update_ptr(p, f_off, convert(f_type, val)) 
end
function set{T<:Any}(p::Ptr{T}, field::Symbol, val, i::Int) # write to element in SVector
   f_off, f_type = mjstructs[T][field]
   ET = eltype(f_type)
   @assert f_type <: SVector
   #@assert typeof(val) == ET
   v = convert(ET, val) # use this as a check
   @assert i <= length(f_type) && i >= 1
   unsafe_store!(convert(Ptr{ET}, (p+f_off+(i-1)*sizeof(ET))), v)
end
function set{T<:Any}(p::Ptr{T}, field::Symbol, val, i::Int, j::Int) # write to element in SVector
   f_off, f_type = mjstructs[T][field]
	@assert f_type <: SVector && eltype(f_type) <: SVector
	ET = eltype(eltype(f_type))
   v = convert(ET, val) # use this as a check
	#r = size(f_type)[1]
	c = sizeof(eltype(f_type))[1]
   #@assert i <= r && i >= 1
   #@assert j <= c && i >= 1
   #idx = (i-1) + (j-1)*r
	idx = p+f_off + (i-1)*c + (j-1)*sizeof(ET)
	unsafe_store!(convert(Ptr{ET}, idx), v)
end

# set struct within model struct 
function set(m::jlModel, fstruct::Symbol, field::Symbol, val::Union{Integer, mjtNum, SVector})
   s_off, s_type = minfo[fstruct]
   @assert s_type in (MuJoCo.Option, MuJoCo.Visual, MuJoCo.Statistic)

   f_off, f_type = mjstructs[s_type][field]
   update_ptr(m.m, s_off+f_off, convert(f_type, val))
end
function set{T<:Any}(p::Ptr{T}, fstruct::Symbol, field::Symbol, val::Union{Integer, mjtNum})
   s_off, s_type = mjstructs[T][fstruct]
   f_off, f_type = mjstructs[s_type][field]
   update_ptr(p, f_off, convert(f_type, val)) 
end



#################################### easier wrappers

step(m::jlModel, d::jlData) = step(m.m, d.d)
forward(m::jlModel, d::jlData) = forward(m.m, d.d)
forwardSkip(m::jlModel, d::jlData,skipstage::Integer,skipsensorenergy::Integer) = forwardSkip(m.m,d.d,skipstage,skipsensorenergy)

inverse(m::jlModel, d::jlData) = inverse(m.m, d.d)
inverseSkip(m::jlModel, d::jlData,skipstage::Integer,skipsensorenergy::Integer) = inverseSkip(m.m,d.d,skipstage,skipsensorenergy)
resetData(m::jlModel, d::jlData) = resetData(m.m, d.d)


#################################### Name Wrappers

function name2idx(m::jlModel, num::Integer, names::Vector{Cint})
    sname = String(m.names)
    idx = names[1] + 1
    split_names = split(sname[idx:end], '\0', limit=(num+1))[1:num]
    d = Dict{Symbol, Integer}(Symbol(split_names[i]) => i for i=1:num)
    return d
end

function name2range(m::jlModel, names::Vector{Cint}, addresses::Vector{Cint})
    return name2range(m, names, addresses, ones(Cint, length(addresses)))
end

function name2range(m::jlModel, num::Integer,
                    names::Vector{Cint}, addresses::Vector{Cint}, dims::Vector{Cint})
    sname = String(m.names)
    idx = names[1] + 1
    split_names = split(sname[idx:end], '\0', limit=(num+1))[1:num]
    d = Dict{Symbol, Range}(Symbol(split_names[i]) => (addresses[i]+1):(addresses[i]+dims[i]) for i=1:num)
    return d
end



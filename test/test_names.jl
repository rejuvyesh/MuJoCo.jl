

# sensors: want to map from sensor name to vector of data

# geom, body, joint, sensor
# site: xpos, etc
# textures, materials, camera

pm = mj.loadXML(modelfile, "")
pd = mj.makeData(pm)
m, d  = mj.mapmujoco(pm, pd)

# sensors for now
sname = String(m.names)
nsnsr = length(m.name_sensoradr)
idx = m.name_sensoradr[1] + 1
snsr_names = split(sname[idx:end], '\0', limit=(nsnsr+1))[1:nsnsr]

sensors = Dict{Symbol, Range}(Symbol(snsr_names[i]) => (m.sensor_adr[i]+1):(m.sensor_adr[i]+m.sensor_dim[i]) for i=1:nsnsr)
snsrs   = Dict{Symbol, SubArray{Float64}}(Symbol(snsr_names[i]) =>
                                          view(d.sensordata, (m.sensor_adr[i]+1):(m.sensor_adr[i]+m.sensor_dim[i]))
                                          for i=1:nsnsr)

function name2range(m::jlModel, names::Vector{Cint}, addresses::Vector{Cint})
    name2range(m, names, addresses, ones(Cint, length(addresses)))
end

function name2range(m::jlModel, names::Vector{Cint}, addresses::Vector{Cint}, dims::Vector{Cint})
    sname = String(m.names)
    num = length(names)
    idx = names[1] + 1
    split_names = split(sname[idx:end], '\0', limit=(num+1))[1:num]
    d = Dict{Symbol, Range}(Symbol(split_names[i]) => (addresses[i]+1):(addresses[i]+dims[i]) for i=1:num)
    return d
end


steps = 1000
tic()
for i=1:steps
    mj.step(m,d)
end
t = toc()
info("Time for $steps mj.steps: ", t)

#@test isapprox( d.sensordata[sensors[:accel]][1], 8.524, rtol=1e-3)
#@test isapprox( snsrs[:accel]][1], 8.524, rtol=1e-3)

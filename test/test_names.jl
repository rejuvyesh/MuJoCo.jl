

# sensors: want to map from sensor name to vector of data

# geom, body, joint, sensor
# site: xpos, etc
# textures, materials, camera

pm = mj.loadXML(modelfile, C_NULL)
pd = mj.makeData(pm)
m, d  = mj.mapmujoco(pm, pd)

#sensors = Dict{Symbol, Range}(Symbol(snsr_names[i]) =>
#                              (m.sensor_adr[i]+1):(m.sensor_adr[i]+m.sensor_dim[i])
#                              for i=1:nsnsr)
sensors = mj.name2range(m, mj.get(m, :nsensor),
                        m.name_sensoradr, m.sensor_adr, m.sensor_dim)

sname = String(copy(m.names))   # See https://github.com/JuliaLang/julia/pull/26093
nsnsr = length(m.name_sensoradr)
idx = m.name_sensoradr[1] + 1
snsr_names = split(sname[idx:end], '\0', limit=(nsnsr+1))[1:nsnsr]
snsrs   = Dict{Symbol, SubArray{Float64}}(Symbol(snsr_names[i]) =>
                                          view(d.sensordata, (m.sensor_adr[i]+1):(m.sensor_adr[i]+m.sensor_dim[i]))
                                          for i=1:nsnsr)

bodies = mj.name2idx(m, mj.get(m, :nbody), m.name_bodyadr) # maps body name to it's index

steps = 1000

t = @elapsed begin
    for i=1:steps
        mj.step(m,d)
    end
end

@info "Time for $steps mj.steps: $t"

@test d.xquat[:, bodies[:world] ] == [1.0, 0.0, 0.0, 0.0]
@test isapprox( d.sensordata[sensors[:accel]][1], 8.524, rtol=1e-3)
@test isapprox( snsrs[:accel][1], 8.524, rtol=1e-3)

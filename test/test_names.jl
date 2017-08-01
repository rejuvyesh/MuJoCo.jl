

# sensors: want to map from sensor name to vector of data

pm = mj.loadXML(modelfile, "")
pd = mj.makeData(pm)
m, d  = mj.mapmujoco(pm, pd)

# sensors for now
sname = String(m.names)
nsnsr = length(m.name_sensoradr)
idx = m.name_sensoradr[1] + 1
snsr_names = split(sname[idx:end], '\0', limit=(nsnsr+1))[1:nsnsr]

sensors = Dict{String, Range}(snsr_names[i] => (m.sensor_adr[i]+1):(m.sensor_adr[i]+m.sensor_dim[i]) for i=1:nsnsr)

steps = 1000
tic()
for i=1:steps
    mj.step(m,d)
end
t = toc()
info("Time for $steps mj.steps: ", t)

@test isapprox( d.sensordata[sensors["accel"]][1], 8.524, rtol=1e-3)

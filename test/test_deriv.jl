
## clean up
pm = mj.loadXML(modelfile, "")
pd = mj.makeData(pm)
m, d = mj.mapmujoco(pm, pd)

nstep = 500
for i=1:nstep
	mj.step(m, d)
end

ndata = Threads.nthreads()
info("Testing Derivatives for ", ndata, " threads.")
datas = Array{jlData}(ndata)
for i=1:ndata
	datas[i] = mj.mapdata(pm, mj.makeData(pm))
	@test qpos0 == datas[i].qpos
end


# alloc some space
nv = mj.get(m, :nv)
@test nv == 27
daccdpos = zeros(mjtNum, nv, nv)
daccdvel = zeros(mjtNum, nv, nv)
daccdfrc = zeros(mjtNum, nv, nv)
dinvdpos = zeros(mjtNum, nv, nv)
dinvdvel = zeros(mjtNum, nv, nv)
dinvdacc = zeros(mjtNum, nv, nv)

olditer = mj.get(m, :opt, :iterations)
oldtolr = mj.get(m, :opt, :tolerance)
niter = 30
mj.set(m, :opt, :iterations, niter)
mj.set(m, :opt, :tolerance, 0)
@test mj.get(m, :opt, :iterations) == niter
@test mj.get(m, :opt, :tolerance) == 0

isforward = false 
for id=1:ndata
   mj.fdworker(m, d, datas[id],
               id, isforward,
               dinvdpos, dinvdvel, dinvdacc)
end
isforward = true
for id=1:ndata
   mj.fdworker(m, d, datas[id],
               id, isforward,
               daccdpos, daccdvel, daccdfrc)
end

mj.set(m, :opt, :iterations, olditer)
mj.set(m, :opt, :tolerance,  oldtolr)

function printneat(arr, nr, nc)
   for i=1:nr
      for j=1:nc
         @printf "%1.2f " arr[i, j] 
      end
      println()
   end
end

err = mj.checkderiv(m, d,
                    dinvdpos, dinvdvel, dinvdacc,
                    daccdpos, daccdvel, daccdfrc)

println()
println(err)



## clean up
pm = mj.loadXML(modelfile, C_NULL)
pd = mj.makeData(pm)
m, d = mj.mapmujoco(pm, pd)
qpos0 = copy(d.qpos)

nstep = 500
for i=1:nstep
  mj.step(m, d)
end

ndata = Threads.nthreads()
@info "Testing Derivatives for $(ndata), threads."
datas = Array{jlData}(undef, ndata)
for i=1:ndata
  datas[i] = mj.mapdata(pm, mj.makeData(pm))
  @test qpos0 == datas[i].qpos
end

# alloc some space
nv = mj.get(m, :nv)
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

for id=1:ndata
   mj.invworker(m, d, datas[id], id,
               dinvdpos, dinvdvel, dinvdacc)
end
for id=1:ndata
   mj.fwdworker(m, d, datas[id], id,
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

function relnorm(res, base)
   l1r = norm(res,1)
   l1b = norm(base,1)
   log10( max(mj.MINVAL, (l1r / max(mj.MINVAL, l1b)) ))
end


function checkderiv(m::jlModel, d::jlData,
              G0::Matrix{mjtNum}, G1::Matrix{mjtNum}, G2::Matrix{mjtNum},
              F0::Matrix{mjtNum}, F1::Matrix{mjtNum}, F2::Matrix{mjtNum})
              
    nv = mj.get(m, :nv)

    error = zeros(8)

    # G2*F2 - I
    mat = G2*F2 - Matrix(I, nv, nv)
    error[1] = relnorm(mat, G2)

    # G2 - G2'
    mat = G2 - G2'
    error[2] = relnorm(mat, G2)

    # G1 - G1'
    mat = G1 - G1'
    error[3] = relnorm(mat, G1)

    # F2 - F2'
    mat = F2 - F2'
    error[4] = relnorm(mat, F2)

    # G1 + G2*F1
    mat = G1 + ( G2 * F1 )
    error[5] = relnorm(mat, G1)

    # G0 + G2*F0
    mat = G0 + G2 * F0
    error[6] = relnorm(mat, G0)

    # F1 + F2*G1
    mat = F1 + F2 * G1
    error[7] = relnorm(mat, F1)

    # F0 + F2*G0
    mat = F0 + F2 * G0
    error[8] = relnorm(mat, F0)

    return error
end


err = checkderiv(m, d,
                 dinvdpos, dinvdvel, dinvdacc,
                 daccdpos, daccdvel, daccdfrc)

println()
println(err)

mj.deleteModel(m.m)
mj.deleteData(d.d)
for i=1:ndata
   mj.deleteData(datas[i].d)
end



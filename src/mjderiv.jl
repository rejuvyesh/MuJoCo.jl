
type jlWorkers
   nthreads::Cint
   d::Vector{mjData}
end

immutable mjDerivatives
   dinvdpos::Ptr{mjtNum}
   dinvdvel::Ptr{mjtNum}
   dinvdacc::Ptr{mjtNum}
   daccdpos::Ptr{mjtNum}
   daccdvel::Ptr{mjtNum}
   daccdfrc::Ptr{mjtNum}
end

type jlDerivatives
   dinvdpos::Vector{mjtNum} #Matrix{mjtNum}
   dinvdvel::Vector{mjtNum} #Matrix{mjtNum}
   dinvdacc::Vector{mjtNum} #Matrix{mjtNum}
   daccdpos::Vector{mjtNum} #Matrix{mjtNum}
   daccdvel::Vector{mjtNum} #Matrix{mjtNum}
   daccdfrc::Vector{mjtNum} #Matrix{mjtNum}
end

type dinv 
   dpos::Vector{mjtNum} #Matrix{mjtNum}
   dvel::Vector{mjtNum} #Matrix{mjtNum}
   dacc::Vector{mjtNum} #Matrix{mjtNum}
   dinv(nv::Integer) = new(Vector{mjtNum}(nv*nv),
                           Vector{mjtNum}(nv*nv),
                           Vector{mjtNum}(nv*nv))
end

type dacc
   dpos::Vector{mjtNum} #Matrix{mjtNum}
   dvel::Vector{mjtNum} #Matrix{mjtNum}
   dfrc::Vector{mjtNum} #Matrix{mjtNum}
   dacc(nv::Integer) = new(Vector{mjtNum}(nv*nv),
                           Vector{mjtNum}(nv*nv),
                           Vector{mjtNum}(nv*nv))
end


function allocderivs(m::jlModel)
   return dinv(get(m, :nv)), dacc(get(m, :nv))
end

function dataworkers(m::jlModel, d::jlData, nthreads::Integer=Threads.nthreads())

   wrkrs = jlWorkers(nthreads, [ mj.makeData(m.m) for i=1:nthreads ])
   derivs = ccall((:derivsetup, libmujocoextra),
                  mjDerivatives,
                  (Ptr{mjModel}, Cint),
                  m.m, nthread)

   d_size = get(m, :nv)^2
   myderivs = jlDerivatives(
                            unsafe_wrap(Array, derivs.dinvdpos, d_size),
                            unsafe_wrap(Array, derivs.dinvdvel, d_size),
                            unsafe_wrap(Array, derivs.dinvdacc, d_size),
                            unsafe_wrap(Array, derivs.daccdpos, d_size),
                            unsafe_wrap(Array, derivs.daccdvel, d_size),
                            unsafe_wrap(Array, derivs.daccdfrc, d_size)
                           )

   ccall((:workers, libmujocoextra),
         Void,
         (Ptr{mjModel},Ptr{mjData},Ptr{mjData},Cint,Cint,mjDerivatives),
         m.m, d.d, wrkrs.d, isforward) #, isforward, derivs)

   #Threads.@threads() for i=1:nthreads
   #   ccall((:workers, libmujocoextra),
   #         Void,
   #         (Ptr{mjModel},Ptr{mjData},Ptr{mjData},Cint,Cint,mjDerivatives),
   #         m.m, d.d, wrkrs.d, i-1, isforward, derivs)

   #end
end


void invworker(const mjModel* m, const mjData* dmain, mjData* d, int id,
               mjDerivatives deriv)

function invworker(m::jlModel, dmain::jlData, d::jlData,
                   id::Integer, isforward::Bool,
                   dpos::Vector{mjtNum}, dvel::Vector{mjtNum}, dacc::Vector{mjtNum})

   nv = get(m, :nv)
   eps = 1e-6
   nwarmup = 3
   nthread = Threads.nthreads() # TODO here?

   # allocate stack space for result at center
   mjMARKSTACK
   #mjtNum* center = mj.stackAlloc(d, nv)
   #mjtNum* warmstart = mj.stackAlloc(d, nv)
   center = unsafe_wrap(Array, mj.stackAlloc(d.d, nv), nv)
   warmstart = unsafe_wrap(Array, mj.stackAlloc(d.d, nv), nv)

   # prepare static schedule: range of derivative columns to be computed by this thread
   chunk = Integer(floor( (nv + nthread-1) / nthread ))
   istart = id * chunk
   iend = min(istart + chunk, nv)

   # copy state and control from dmain to thread-specific d
   set(d, :time, get(dmain, :time)) #d.time = dmain.time
   copy!(d.qpos, dmain.qpos)
   copy!(d.qvel, dmain.qvel)
   copy!(d.qacc, dmain.qacc)
   copy!(d.qacc_warmstart, dmain.qacc_warmstart)
   copy!(d.qfrc_applied, dmain.qfrc_applied)
   copy!(d.xfrc_applied, dmain.xfrc_applied)
   copy!(d.ctrl, dmain.ctrl)

   # run full computation at center point (usually faster than copying dmain)
   if isforward
      mj.forward(m, d);

      # extra solver iterations to improve warmstart (qacc) at center point
      for rep=1:nwarmup
         mj_forwardSkip(m, d, mjSTAGE_VEL, 1);
      end
   else
      mj.inverse(m, d);
   end

   # select output from forward or inverse dynamics
   output = isforward ? d->qacc : d->qfrc_inverse # should be by reference

   # save output for center point and warmstart (needed in forward only)
   copy!(center, output)
   copy!(warmstart, d.qacc_warmstart)

   # select target vector and original vector for force or acceleration derivative
   mjtNum* target = (isforward ? d->qfrc_applied : d->qacc);
   const mjtNum* original = (isforward ? dmain->qfrc_applied : dmain->qacc);

   mjtNum* target = d.qacc
   const mjtNum* original = dmain.qacc

   #  dinv/dpos, dinv/dvel, dinv/dacc, dacc/dpos, dacc/dvel, dacc/dfrc
   block1 = dpos # by reference; input depends on isforward and not sloppy user
   block2 = dvel
   block3 = dacc
   #block1 = (isforward ? deriv.daccdpos : deriv.dinvdpos);
   #block2 = (isforward ? deriv.daccdvel : deriv.dinvdvel);
   #block3 = (isforward ? deriv.daccdfrc : deriv.dinvdacc);

   # finite-difference over force or acceleration: skip = mjSTAGE_VEL
   for i=istart:iend
      # perturb selected target
      target[i] += eps

      # evaluate dynamics, with center warmstart
      if isforward
         copy!(d.qacc_warmstart, warmstart);
         mj.forwardSkip(m, d, mjSTAGE_VEL, 1);
      else
         mj.inverseSkip(m, d, mjSTAGE_VEL, 1)
      end

      # undo perturbation
      target[i] = original[i]

      # compute column i of derivative 2
      for j=0:nv
         block3[(2)*nv*nv + i + j*nv] = (output[j] - center[j])/eps
      end
   end

   # finite-difference over velocity: skip = mjSTAGE_POS
   for i=istart:iend
      # perturb velocity
      d.qvel[i] += eps

      # evaluate dynamics, with center warmstart
      if isforward
         copy!(d.qacc_warmstart, warmstart);
         mj.forwardSkip(m, d, mjSTAGE_POS, 1);
      else
         mj.inverseSkip(m, d, mjSTAGE_POS, 1)
      end

      # undo perturbation
      d.qvel[i] = dmain.qvel[i]

      # compute column i of derivative 1
      for( int j=0 j<nv j++ )
         block2[(1)*nv*nv + i + j*nv] = (output[j] - center[j])/eps
      end
   end


   # finite-difference over position: skip = mjSTAGE_NONE
   for i=istart:iend
      # get joint id for this dof
      int jid = m.dof_jntid[i]

      # get quaternion address and dof position within quaternion (-1: not in quaternion)
      int quatadr = -1, dofpos = 0
      if m.jnt_type[jid]==mjJNT_BALL
         quatadr = m.jnt_qposadr[jid]
         dofpos = i - m.jnt_dofadr[jid]
      elseif( m.jnt_type[jid]==mjJNT_FREE && i>=m.jnt_dofadr[jid]+3 )
         quatadr = m.jnt_qposadr[jid] + 3
         dofpos = i - m.jnt_dofadr[jid] - 3
      end

      # apply quaternion or simple perturbation
      if quatadr>=0
          mjtNum angvel[3] = {0,0,0}
          angvel[dofpos] = eps
          # TODO fix this BS
          mju_quatIntegrate(d.qpos+quatadr, angvel, 1)
      else
         d.qpos[i] += eps
      end

      # evaluate dynamics, with center warmstart
      if isforward
         copy!(d.qacc_warmstart, warmstart);
         mj.forwardSkip(m, d, mjSTAGE_NONE, 1);
      else
         mj.inverseSkip(m, d, mjSTAGE_NONE, 1)
      end

      # undo perturbation
      mju_copy(d.qpos, dmain.qpos, m.nq)

      # compute column i of derivative 0
      for( int j=0 j<nv j++ )
         block1[(0)*nv*nv + i + j*nv] = (output[j] - center[j])/eps
      end
   end

   mjFREESTACK
end

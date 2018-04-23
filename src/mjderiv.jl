

#=
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
=#


function fwdworker(m::jlModel, dmain::jlData, d::jlData,
                   tid::Integer,
                   block1::Matrix{mjtNum}, block2::Matrix{mjtNum}, block3::Matrix{mjtNum})
   const nv = length(d.qvel)
   const eps = 1e-6
   const nwarmup = 3
   const nthread = Threads.nthreads() # TODO here?

   mark = mj.MARKSTACK(d)

   center = unsafe_wrap(Array, mj.stackAlloc(d.d, nv), nv)
   warmstart = unsafe_wrap(Array, mj.stackAlloc(d.d, nv), nv)
   angvel = unsafe_wrap(Array, mj.stackAlloc(d.d, 3), 3)
   quat = unsafe_wrap(Array, mj.stackAlloc(d.d, 4), 4)

   # prepare static schedule: range of derivative columns to be computed by this thread
   chunk = Integer(floor( (nv + nthread-1) / nthread ))
   istart = (tid-1) * chunk + 1
   iend = min(istart + chunk - 1, nv)
   println(istart, " ", chunk, " ", iend)

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
   mj.forward(m, d)
   for rep=1:nwarmup
      mj.forwardSkip(m, d, Int(mj.STAGE_VEL), 1)
   end

   # save output for center point and warmstart (needed in forward only)
   copy!(center, d.qacc)
   copy!(warmstart, d.qacc_warmstart)

   # finite-difference over force or acceleration: skip = mjSTAGE_VEL
   for i=istart:iend
      # perturb selected target
      d.qfrc_applied[i] += eps

      # evaluate dynamics, with center warmstart
      copy!(d.qacc_warmstart, warmstart)
      mj.forwardSkip(m, d, Int(mj.STAGE_VEL), 1)

      # undo perturbation
      d.qfrc_applied[i] = dmain.qfrc_applied[i]

      # compute column i of derivative 2
      block3[:,i] = (d.qacc - center)/eps
   end

   # finite-difference over velocity: skip = mjSTAGE_POS
   for i=istart:iend
      # perturb velocity
      d.qvel[i] += eps

      # evaluate dynamics, with center warmstart
      copy!(d.qacc_warmstart, warmstart)
      mj.forwardSkip(m, d, Int(mj.STAGE_POS), 1)

      # undo perturbation
      d.qvel[i] = dmain.qvel[i]

      # compute column i of derivative 1
      block2[:,i] = (d.qacc - center)/eps
   end

   # finite-difference over position: skip = mjSTAGE_NONE
   for i=istart:iend
      # get joint id for this dof
      jid = m.dof_jntid[i] + 1

      # get quaternion address and dof position within quaternion (-1: not in quaternion)
      quatadr = -1
      dofpos = 0
      if m.jnt_type[jid] == Int(mj.JNT_BALL)
         quatadr = m.jnt_qposadr[jid]
         dofpos = (i-1) - m.jnt_dofadr[jid]
      elseif m.jnt_type[jid] == Int(mj.JNT_FREE) && i>=m.jnt_dofadr[jid]+4
         quatadr = m.jnt_qposadr[jid] + 3
         dofpos = (i-1) - m.jnt_dofadr[jid] - 3
      end

      # apply quaternion or simple perturbation
      if quatadr>=0
         angvel .= 0.0 
         angvel[dofpos+1] = eps # already +1 from i
         quat .= d.qpos[(quatadr+1):(quatadr+4)]
         mju_quatIntegrate(quat, angvel, 1.0)
         d.qpos[(quatadr+1):(quatadr+4)] .= quat
      else
         d.qpos[i] += eps
      end

      # evaluate dynamics, with center warmstart
      copy!(d.qacc_warmstart, warmstart)
      mj.forwardSkip(m, d, Int(mj.STAGE_NONE), 1)

      # undo perturbation
      copy!(d.qpos, dmain.qpos)

      # compute column i of derivative 0
      block1[:,i] = (d.qacc - center)/eps
   end

   mj.FREESTACK(d, mark)
end

function invworker(m::jlModel, dmain::jlData, d::jlData,
                   tid::Integer,
                   block1::Matrix{mjtNum}, block2::Matrix{mjtNum}, block3::Matrix{mjtNum})

   const nv = length(d.qvel)
   const eps = 1e-6
   const nwarmup = 3
   const nthread = Threads.nthreads() # TODO here?

   mark = mj.MARKSTACK(d)

   center = unsafe_wrap(Array, mj.stackAlloc(d.d, nv), nv)
   warmstart = unsafe_wrap(Array, mj.stackAlloc(d.d, nv), nv)
   angvel = unsafe_wrap(Array, mj.stackAlloc(d.d, 3), 3)
   quat = unsafe_wrap(Array, mj.stackAlloc(d.d, 4), 4)

   # prepare static schedule: range of derivative columns to be computed by this thread
   chunk = Integer(floor( (nv + nthread-1) / nthread ))
   istart = (tid-1) * chunk + 1
   iend = min(istart + chunk - 1, nv)
   println(istart, " ", chunk, " ", iend)

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
   mj.inverse(m, d)

   # save output for center point and warmstart (needed in forward only)
   copy!(center, d.qfrc_inverse)
   copy!(warmstart, d.qacc_warmstart)

   # finite-difference over force or acceleration: skip = mjSTAGE_VEL
   for i=istart:iend
      # perturb selected target
      d.qacc[i] += eps

      # evaluate dynamics, with center warmstart
      mj.inverseSkip(m, d, Int(mj.STAGE_VEL), 1)

      # undo perturbation
      d.qacc[i] = dmain.qacc[i]

      # compute column i of derivative 2
      block3[:,i] = (d.qfrc_inverse - center)/eps
   end

   # finite-difference over velocity: skip = mjSTAGE_POS
   for i=istart:iend
      # perturb velocity
      d.qvel[i] += eps

      # evaluate dynamics, with center warmstart
      mj.inverseSkip(m, d, Int(mj.STAGE_POS), 1)

      # undo perturbation
      d.qvel[i] = dmain.qvel[i]

      # compute column i of derivative 1
      block2[:,i] = (d.qfrc_inverse - center)/eps
   end

   # finite-difference over position: skip = mjSTAGE_NONE
   for i=istart:iend
      # get joint id for this dof
      jid = m.dof_jntid[i] + 1

      # get quaternion address and dof position within quaternion (-1: not in quaternion)
      quatadr = -1
      if m.jnt_type[jid] == Int(mj.JNT_BALL)
         quatadr = m.jnt_qposadr[jid]
         dofpos = (i-1) - m.jnt_dofadr[jid]
      elseif m.jnt_type[jid] == Int(mj.JNT_FREE) && i>=m.jnt_dofadr[jid]+4
         quatadr = m.jnt_qposadr[jid] + 3
         dofpos = (i-1) - m.jnt_dofadr[jid] - 3
      end

      # apply quaternion or simple perturbation
      if quatadr>=0
         angvel .= 0.0 
         angvel[dofpos+1] = eps # already +1 from i
         quat .= d.qpos[(quatadr+1):(quatadr+4)]
         mju_quatIntegrate(quat, angvel, 1.0)
         d.qpos[(quatadr+1):(quatadr+4)] .= quat
      else
         d.qpos[i] += eps
      end

      # evaluate dynamics, with center warmstart
      mj.inverseSkip(m, d, Int(mj.STAGE_NONE), 1)

      # undo perturbation
      copy!(d.qpos, dmain.qpos)

      # compute column i of derivative 0
      block1[:,i] = (d.qfrc_inverse - center)/eps
   end

   mj.FREESTACK(d, mark)
end



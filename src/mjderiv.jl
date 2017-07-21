
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

function fdworker(m::jlModel, dmain::jlData, d::jlData,
                  id::Integer, isforward::Bool,
                  block1::Matrix{mjtNum}, block2::Matrix{mjtNum}, block3::Matrix{mjtNum})

   nv = get(m, :nv)
   eps = 1e-6
   nwarmup = 3
   nthread = Threads.nthreads() # TODO here?

   mark = mj.MARKSTACK(d)

   #mjtNum* center = mj.stackAlloc(d, nv)
   #mjtNum* warmstart = mj.stackAlloc(d, nv)
   center = unsafe_wrap(Array, mj.stackAlloc(d.d, nv), nv)
   warmstart = unsafe_wrap(Array, mj.stackAlloc(d.d, nv), nv)
	#center = zeros(nv)
	#warmstart = zeros(nv)

   # prepare static schedule: range of derivative columns to be computed by this thread
   chunk = Integer(floor( (nv + nthread-1) / nthread ))
   istart = (id-1) * chunk + 1
   iend = min(istart + chunk - 1, nv)
   #println(istart, " ", chunk, " ", iend)

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
      mj.forward(m, d)
      for rep=1:nwarmup
         mj.forwardSkip(m, d, Int(mjSTAGE_VEL), 1)
      end
   else
      mj.inverse(m, d)
   end

   # select output from forward or inverse dynamics
   output = isforward ? d.qacc : d.qfrc_inverse # should be by reference

   # save output for center point and warmstart (needed in forward only)
   copy!(center, output)
   copy!(warmstart, d.qacc_warmstart)

   # select target vector and original vector for force or acceleration derivative
	target = isforward ? d.qfrc_applied : d.qacc
	original = isforward ? dmain.qfrc_applied : dmain.qacc

   # finite-difference over force or acceleration: skip = mjSTAGE_VEL
   for i=istart:iend
      # perturb selected target
      target[i] += eps

      # evaluate dynamics, with center warmstart
      if isforward
         copy!(d.qacc_warmstart, warmstart)
         mj.forwardSkip(m, d, Int(mjSTAGE_VEL), 1)
      else
         mj.inverseSkip(m, d, Int(mjSTAGE_VEL), 1) # TODO WTF
      end

      # undo perturbation
      target[i] = original[i]

      # compute column i of derivative 2
      block3[:,i] = (output - center)/eps
   end

   # finite-difference over velocity: skip = mjSTAGE_POS
   for i=istart:iend
      # perturb velocity
      d.qvel[i] += eps

      # evaluate dynamics, with center warmstart
      if isforward
         copy!(d.qacc_warmstart, warmstart)
         mj.forwardSkip(m, d, Int(mjSTAGE_POS), 1)
      else
         mj.inverseSkip(m, d, Int(mjSTAGE_POS), 1)
      end

      # undo perturbation
      d.qvel[i] = dmain.qvel[i]

      # compute column i of derivative 1
      block2[:,i] = (output - center)/eps
   end

   # finite-difference over position: skip = mjSTAGE_NONE
	for i=istart:iend
		# get joint id for this dof
		jid = m.dof_jntid[i] + 1

		# get quaternion address and dof position within quaternion (-1: not in quaternion)
		quatadr = -1
		dofpos = 0
		if m.jnt_type[jid] == Int(mjJNT_BALL)
			quatadr = m.jnt_qposadr[jid]
			dofpos = (i-1) - m.jnt_dofadr[jid]
		elseif m.jnt_type[jid] == Int(mjJNT_FREE) && i>=m.jnt_dofadr[jid]+4
			quatadr = m.jnt_qposadr[jid] + 3
			dofpos = (i-1) - m.jnt_dofadr[jid] - 3
		end

		# apply quaternion or simple perturbation
		if quatadr>=0
			angvel = MVector(0.0, 0.0, 0.0)
			
			angvel[dofpos+1] = eps # already +1 from i
			angvel = SVector(angvel)

			quat = SVector{4, mjtNum}( d.qpos[(quatadr+1):(quatadr+4)] ) # julia 1 indexing
			mju_quatIntegrate(quat, angvel, 1.0)
			d.qpos[(quatadr+1):(quatadr+4)] = quat
		else
			d.qpos[i] += eps
		end

		# evaluate dynamics, with center warmstart
		if isforward
			copy!(d.qacc_warmstart, warmstart)
			mj.forwardSkip(m, d, Int(mjSTAGE_NONE), 1)
		else
			mj.inverseSkip(m, d, Int(mjSTAGE_NONE), 1)
		end

		# undo perturbation
		copy!(d.qpos, dmain.qpos)

		# compute column i of derivative 0
		if isforward == false
			diff = output - center
		end
		block1[:,i] = (output - center)/eps
	end

	mj.FREESTACK(d, mark)
end

function checkderiv(m::jlModel, d::jlData,
						  dinvdpos::Matrix{mjtNum}, dinvdvel::Matrix{mjtNum}, dinvdacc::Matrix{mjtNum},
						  daccdpos::Matrix{mjtNum}, daccdvel::Matrix{mjtNum}, daccdfrc::Matrix{mjtNum})
						  
   nv = get(m, :nv)

   # get pointers to derivative matrices
   G0 = dinvdpos       # dinv/dpos
   G1 = dinvdvel       # dinv/dvel
   G2 = dinvdacc       # dinv/dacc
   F0 = daccdpos       # dacc/dpos
   F1 = daccdvel       # dacc/dvel
   F2 = daccdfrc       # dacc/dfrc

	error = zeros(8)

	relnorm(res, base) = log10( max( mjMINVAL, sum(abs.(res)) / max(mjMINVAL,sum(abs.(base)) ) ) )

   # G2*F2 - I
	mat = G2*F2 - eye(nv)
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



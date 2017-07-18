
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

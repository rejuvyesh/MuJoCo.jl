
@enum mjtGDDActuate mjGDDACT_TRANSMISSION = (UInt32)(0) mjGDDACT_NONFREE = (UInt32)(1) mjGDDACT_ALL = (UInt32)(2)

@enum mjtGDDFail mjGDDFAIL_NONE = (UInt32)(0) mjGDDFAIL_REGULAR = (UInt32)(1) mjGDDFAIL_BOTH = (UInt32)(2)

type _mjoGDDOption
   maxiter::Cint
   actuation::Cint
   penalty::mjtNum
   steptol::mjtNum
   gradtol::mjtNum
   errtol::mjtNum
   minstep::mjtNum
   maxstep::mjtNum
   margin::mjtNum
   Q::Ptr{mjtNum}
   q::Ptr{mjtNum}
end

const mjoGDDOption = _mjoGDDOption

type _mjoGDDStats
   iterations::Cint
   updates::Cint
   fail::Cint
   cost::mjtNum
   penalty::mjtNum
   error::mjtNum
   gradient::mjtNum
   lambda::mjtNum
   stepsize::mjtNum
   segments::mjtNum
   edges::mjtNum
   changes::mjtNum
end

const mjoGDDStats = _mjoGDDStats

#---------------------- Optimization ---------------------------------------------------

# acceleration constraint penalty
function mjo_penalty(m::Ptr{mjModel},d::Ptr{mjData},jar::Ptr{mjtNum},ghat::Ptr{mjtNum},Hhat::Ptr{mjtNum},state::Ptr{Cint})
   ccall((:mjo_penalty,:libmujoco141nogl),mjtNum,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Ptr{Cint}),m,d,jar,ghat,Hhat,state)
end

# set default options for GDD solver
function mjo_GDD_defaultOption(opt::Ptr{mjoGDDOption})
   ccall((:mjo_GDD_defaultOption,:libmujoco141nogl),Void,(Ptr{mjoGDDOption},),opt)
end

# run GDD solver
function mjo_GDD_solve(m::Ptr{mjModel},d::Ptr{mjData},opt::Ptr{mjoGDDOption})
   ccall((:mjo_GDD_solve,:libmujoco141nogl),mjoGDDStats,(Ptr{mjModel},Ptr{mjData},Ptr{mjoGDDOption}),m,d,opt)
end


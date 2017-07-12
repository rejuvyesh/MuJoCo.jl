
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


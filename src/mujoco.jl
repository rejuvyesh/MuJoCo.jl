
# TODO write these as functions

# Skipping MacroDefinition: mjMARKSTACK int _mark = d -> pstack ;
# Skipping MacroDefinition: mjFREESTACK d -> pstack = _mark ;
MARKSTACK(d::jlData) = return mj.get(d, :pstack)
FREESTACK(d::jlData, mark::Cint) = mj.set(d, :pstack, mark)
# Skipping MacroDefinition: mjDISABLED ( x ) ( m -> opt . disableflags & ( x ) )
# Skipping MacroDefinition: mjENABLED ( x ) ( m -> opt . enableflags & ( x ) )
mjMAX(a,b) = max(a,b)
mjMIN(a,b) = min(a,b)

## user error and memory handlers
#MJAPI extern void  (*mju_user_error)(const char*);
#MJAPI extern void  (*mju_user_warning)(const char*);
#MJAPI extern void* (*mju_user_malloc)(size_t);
#MJAPI extern void  (*mju_user_free)(void*);
#
## callbacks extending computation pipeline
#MJAPI extern mjfGeneric  mjcb_passive;
#MJAPI extern mjfGeneric  mjcb_control;
#MJAPI extern mjfSensor   mjcb_sensor;
#MJAPI extern mjfTime     mjcb_time;
#MJAPI extern mjfAct      mjcb_act_dyn;
#MJAPI extern mjfAct      mjcb_act_gain;
#MJAPI extern mjfAct      mjcb_act_bias;
#MJAPI extern mjfSolImp   mjcb_sol_imp;
#MJAPI extern mjfSolRef   mjcb_sol_ref;
#
## collision function table
#MJAPI extern mjfCollision mjCOLLISIONFUNC[mjNGEOMTYPES][mjNGEOMTYPES];
#
## string names
#MJAPI extern const char* mjDISABLESTRING[mjNDISABLE];
#MJAPI extern const char* mjENABLESTRING[mjNENABLE];
#MJAPI extern const char* mjTIMERSTRING[mjNTIMER];
#MJAPI extern const char* mjLABELSTRING[mjNLABEL];
#MJAPI extern const char* mjFRAMESTRING[mjNFRAME];
#MJAPI extern const char* mjVISSTRING[mjNVISFLAG][3];
#MJAPI extern const char* mjRNDSTRING[mjNRNDFLAG][3];


const PtrVec = Union{Ptr{mjtNum},Vector{mjtNum}}

#---------------------- License activation and certificate (mutex-protected) -----------

# activate license, call mju_error on failure; return 1 if ok, 0 if failure
function activate(filename::String)
   ccall((:mj_activate,libmujoco),Cint,(Cstring,),filename)
end

# deactivate license, free memory
function deactivate()
   ccall((:mj_deactivate,libmujoco),Void,())
end

# server: generate certificate question
function certQuestion(question::SVector{16, mjtNum})
   ccall((:mj_certQuestion,libmujoco),Void,(SVector{16, mjtNum},),question)
end

# client: generate certificate answer given question
function certAnswer(question::SVector{16, mjtNum},answer::SVector{16, mjtNum})
   ccall((:mj_certAnswer,libmujoco),Void,(SVector{16, mjtNum},SVector{16, mjtNum}),question,answer)
end

# server: check certificate question-answer pair; return 1 if match, 0 if mismatch
function certCheck(question::SVector{16, mjtNum},answer::SVector{16, mjtNum})
   ccall((:mj_certCheck,libmujoco),Cint,(SVector{16, mjtNum},SVector{16, mjtNum}),question,answer)
end

#---------------------- XML parser and C++ compiler (mutex-protected) ------------------

# parse XML file or string in MJCF or URDF format, compile it, return low-level model
#  if xmlstring is not NULL, it has precedence over filename
#  error can be NULL; otherwise assumed to have size error_sz
function loadXML(filename::String,xmlstring::String) #,error::String,error_sz::Integer)
   errsz = 1000
   err = Vector{UInt8}(errsz) 
   if length(xmlstring) == 0
      m=ccall((:mj_loadXML,libmujoco),Ptr{mjModel},(Cstring,Cstring,Ptr{UInt8},Cint),filename,C_NULL,err,errsz)
   else
      m=ccall((:mj_loadXML,libmujoco),Ptr{mjModel},(Cstring,Cstring,Ptr{UInt8},Cint),filename,xmlstring,err,errsz)
   end
   if m == C_NULL
      err[end] = 0;
      warn("Error in XML loading: ", unsafe_string(pointer(err)), "\nCheck path and .xml file.")
      return nothing
   end
   return m
end

# update XML data structures with info from low-level model, save as MJCF
#  error can be NULL; otherwise assumed to have size error_sz
function saveXML(filename::String,m::Ptr{mjModel},error::String,error_sz::Integer)
   ccall((:mj_saveXML,libmujoco),Cint,(Cstring,Ptr{mjModel},Cstring,Cint),filename,m,error,error_sz)
end

# print internal XML schema as plain text or HTML, with style-padding or &nbsp;
function printSchema(filename::String,buffer::String,buffer_sz::Integer,flg_html::Integer,flg_pad::Integer)
   ccall((:mj_printSchema,libmujoco),Cint,(Cstring,Cstring,Cint,Cint,Cint),filename,buffer,buffer_sz,flg_html,flg_pad)
end

#---------------------- Main entry points ----------------------------------------------


# advance simulation: use control callback, no external force, RK4 available
function step(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_step,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# advance simulation in two steps: before external force/control is set by user
function step1(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_step1,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# advance simulation in two steps: after external force/control is set by user
function step2(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_step2,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# forward dynamics
function forward(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_forward,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# inverse dynamics
function inverse(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_inverse,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# forward dynamics with skip; skipstage is mjtStage
function forwardSkip(m::Ptr{mjModel},d::Ptr{mjData},skipstage::Integer,skipsensorenergy::Integer)
   ccall((:mj_forwardSkip,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Cint,Cint),m,d,skipstage,skipsensorenergy)
end

# inverse dynamics with skip; skipstage is mjtStage
function inverseSkip(m::Ptr{mjModel},d::Ptr{mjData},skipstage::Integer,skipsensorenergy::Integer)
   ccall((:mj_inverseSkip,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Cint,Cint),m,d,skipstage,skipsensorenergy)
end

#---------------------- Model and data initialization ----------------------------------

# set default solver parameters
function defaultSolRefImp(solref::Ptr{mjtNum},solimp::Ptr{mjtNum})
   ccall((:mj_defaultSolRefImp,libmujoco),Void,(Ptr{mjtNum},Ptr{mjtNum}),solref,solimp)
end

# set physics options to default values
function defaultOption(opt::Ptr{mjOption})
   ccall((:mj_defaultOption,libmujoco),Void,(Ptr{mjOption},),opt)
end

# set visual options to default values
function defaultVisual(vis::Ptr{mjVisual})
   ccall((:mj_defaultVisual,libmujoco),Void,(Ptr{mjVisual},),vis)
end

# copy mjModel; allocate new if dest is NULL
function copyModel(dest::Ptr{mjModel},src::Ptr{mjModel})
   ccall((:mj_copyModel,libmujoco),Ptr{mjModel},(Ptr{mjModel},Ptr{mjModel}),dest,src)
end

# save model to binary file or memory buffer (buffer has precedence if szbuf>0)
function saveModel(m::Ptr{mjModel},filename::String,buffer::Ptr{Void},buffer_sz::Integer)
   ccall((:mj_saveModel,libmujoco),Void,(Ptr{mjModel},Cstring,Ptr{Void},Cint),m,filename,buffer,buffer_sz)
end

# load model from binary file or memory buffer (buffer has precedence if szbuf>0)
function loadModel(filename::String,buffer::Ptr{Void},buffer_sz::Integer)
   ccall((:mj_loadModel,libmujoco),Ptr{mjModel},(Cstring,Ptr{Void},Cint),filename,buffer,buffer_sz)
end

# de-allocate model
function deleteModel(m::Ptr{mjModel})
   ccall((:mj_deleteModel,libmujoco),Void,(Ptr{mjModel},),m)
end

# size of buffer needed to hold model
function sizeModel(m::Ptr{mjModel})
   ccall((:mj_sizeModel,libmujoco),Cint,(Ptr{mjModel},),m)
end

# allocate mjData correponding to given model
function makeData(m::Ptr{mjModel})
   ccall((:mj_makeData,libmujoco),Ptr{mjData},(Ptr{mjModel},),m)
end

# copy mjData
function copyData(dest::Ptr{mjData},m::Ptr{mjModel},src::Ptr{mjData})
   ccall((:mj_copyData,libmujoco),Ptr{mjData},(Ptr{mjData},Ptr{mjModel},Ptr{mjData}),dest,m,src)
end

# set data to defaults
function resetData(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_resetData,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# set data to defaults, fill everything else with debug_value
function resetDataDebug(m::Ptr{mjModel},d::Ptr{mjData},debug_value::Cuchar)
   ccall((:mj_resetDataDebug,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Cuchar),m,d,debug_value)
end

# reset data, set fields from specified keyframe
function resetDataKeyframe(m::Ptr{mjModel},d::Ptr{mjData},key::Integer)
   ccall((:mj_resetDataKeyframe,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Cint),m,d,key)
end

# mjData stack allocate
function stackAlloc(d::Ptr{mjData},size::Integer)
   ccall((:mj_stackAlloc,libmujoco),Ptr{mjtNum},(Ptr{mjData},Cint),d,size)
end

# de-allocate data
function deleteData(d::Ptr{mjData})
   ccall((:mj_deleteData,libmujoco),Void,(Ptr{mjData},),d)
end

# reset callbacks to defaults
function resetCallbacks()
   ccall((:mj_resetCallbacks,libmujoco),Void,())
end

# set constant fields of mjModel
function setConst(m::Ptr{mjModel},d::Ptr{mjData},flg_actrange::Integer)
   ccall((:mj_setConst,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Cint),m,d,flg_actrange)
end

#---------------------- Printing -------------------------------------------------------

# print model to text file
function printModel(m::Ptr{mjModel},filename::String)
   ccall((:mj_printModel,libmujoco),Void,(Ptr{mjModel},Cstring),m,filename)
end

# print data to text file
function printData(m::Ptr{mjModel},d::Ptr{mjData},filename::String)
   ccall((:mj_printData,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Cstring),m,d,filename)
end

# print matrix to screen
function mju_printMat(mat::Ptr{mjtNum},nr::Integer,nc::Integer)
   ccall((:mju_printMat,libmujoco),Void,(Ptr{mjtNum},Cint,Cint),mat,nr,nc)
end

#---------------------- Components: forward dynamics -----------------------------------

# position-dependent computations
function fwdPosition(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_fwdPosition,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# velocity-dependent computations
function fwdVelocity(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_fwdVelocity,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# compute actuator force
function fwdActuation(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_fwdActuation,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# add up all non-constraint forces, compute qacc_unc
function fwdAcceleration(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_fwdAcceleration,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# constraint solver
function fwdConstraint(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_fwdConstraint,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# Euler integrator, semi-implicit in velocity
function Euler(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_Euler,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# Runge-Kutta explicit order-N integrator
function RungeKutta(m::Ptr{mjModel},d::Ptr{mjData},N::Integer)
   ccall((:mj_RungeKutta,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Cint),m,d,N)
end

#---------------------- Components: inverse dynamics -----------------------------------

# position-dependent computations
function invPosition(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_invPosition,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# velocity-dependent computations
function invVelocity(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_invVelocity,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# constraint solver
function invConstraint(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_invConstraint,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# compare forward and inverse dynamics, without changing results of forward dynamics
function compareFwdInv(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_compareFwdInv,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

#---------------------- Components: forward and inverse dynamics -----------------------

# position-dependent sensors
function sensorPos(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_sensorPos,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# velocity-dependent sensors
function sensorVel(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_sensorVel,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# acceleration/force-dependent sensors
function sensorAcc(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_sensorAcc,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# position-dependent energy (potential)
function energyPos(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_energyPos,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# velocity-dependent energy (kinetic)
function energyVel(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_energyVel,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

#---------------------- Sub-components -------------------------------------------------

# check positions; reset if bad
function checkPos(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_checkPos,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# check velocities; reset if bad
function checkVel(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_checkVel,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# check accelerations; reset if bad
function checkAcc(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_checkAcc,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# forward kinematics
function kinematics(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_kinematics,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# map inertias and motion dofs to global frame centered at CoM
function comPos(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_comPos,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# compute camera and light positions and orientations
function camlight(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_camlight,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# compute tendon lengths, velocities and moment arms
function tendon(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_tendon,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# compute actuator transmission lengths and moments
function transmission(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_transmission,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# composite rigid body inertia algorithm
function crb(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_crb,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# sparse L'*D*L factorizaton of the inertia matrix
function factorM(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_factorM,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# sparse backsubstitution:  x = inv(L'*D*L)*y
function backsubM(m::Ptr{mjModel},d::Ptr{mjData},x::Ptr{mjtNum},y::Ptr{mjtNum},n::Integer)
   ccall((:mj_backsubM,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum},Cint),m,d,x,y,n)
end

# half of sparse backsubstitution:  x = sqrt(inv(D))*inv(L')*y
function backsubM2(m::Ptr{mjModel},d::Ptr{mjData},x::Ptr{mjtNum},y::Ptr{mjtNum},n::Integer)
   ccall((:mj_backsubM2,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum},Cint),m,d,x,y,n)
end

# compute cvel, cdof_dot
function comVel(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_comVel,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# spring-dampers and body viscosity
function passive(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_passive,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term
function rne(m::Ptr{mjModel},d::Ptr{mjData},flg_acc::Integer,result::Ptr{mjtNum})
   ccall((:mj_rne,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Cint,Ptr{mjtNum}),m,d,flg_acc,result)
end

# RNE with complete data: compute cacc, cfrc_ext, cfrc_int
function rnePostConstraint(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_rnePostConstraint,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# collision detection
function collision(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_collision,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# construct constraints
function makeConstraint(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_makeConstraint,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# compute dense matrices: efc_AR, e_ARchol, fc_half, fc_AR
function projectConstraint(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_projectConstraint,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

# compute efc_vel, efc_aref
function referenceConstraint(m::Ptr{mjModel},d::Ptr{mjData})
   ccall((:mj_referenceConstraint,libmujoco),Void,(Ptr{mjModel},Ptr{mjData}),m,d)
end

#---------------------- Support functions ----------------------------------------------

# add contact to d->contact list; return 0 if success; 1 if buffer full
function addContact(m::Ptr{mjModel},d::Ptr{mjData},con::Ptr{mjContact})
   ccall((:mj_addContact,libmujoco),Cint,(Ptr{mjModel},Ptr{mjData},Ptr{mjContact}),m,d,con)
end

# determine type of friction cone
function isPyramid(m::Ptr{mjModel})
   ccall((:mj_isPyramid,libmujoco),Cint,(Ptr{mjModel},),m)
end

# determine type of constraint Jacobian
function isSparse(m::Ptr{mjModel})
   ccall((:mj_isSparse,libmujoco),Cint,(Ptr{mjModel},),m)
end

# multiply Jacobian by vector
function mulJacVec(m::Ptr{mjModel},d::Ptr{mjData},res::Ptr{mjtNum},vec::Ptr{mjtNum})
   ccall((:mj_mulJacVec,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum}),m,d,res,vec)
end

# multiply JacobianT by vector
function mulJacTVec(m::Ptr{mjModel},d::Ptr{mjData},res::Ptr{mjtNum},vec::Ptr{mjtNum})
   ccall((:mj_mulJacTVec,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum}),m,d,res,vec)
end

# compute 3/6-by-nv Jacobian of global point attached to given body
function jac(m::Ptr{mjModel},d::Ptr{mjData},jacp::Ptr{mjtNum},jacr::Ptr{mjtNum},point::SVector{3, mjtNum},body::Integer)
   ccall((:mj_jac,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum},SVector{3, mjtNum},Cint),m,d,jacp,jacr,point,body)
end

# compute body frame Jacobian
function jacBody(m::Ptr{mjModel},d::Ptr{mjData},jacp::Ptr{mjtNum},jacr::Ptr{mjtNum},body::Integer)
   ccall((:mj_jacBody,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum},Cint),m,d,jacp,jacr,body)
end

# compute body center-of-mass Jacobian
function jacBodyCom(m::Ptr{mjModel},d::Ptr{mjData},jacp::Ptr{mjtNum},jacr::Ptr{mjtNum},body::Integer)
   ccall((:mj_jacBodyCom,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum},Cint),m,d,jacp,jacr,body)
end

# compute geom Jacobian
function jacGeom(m::Ptr{mjModel},d::Ptr{mjData},jacp::Ptr{mjtNum},jacr::Ptr{mjtNum},geom::Integer)
   ccall((:mj_jacGeom,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum},Cint),m,d,jacp,jacr,geom)
end

# compute site Jacobian
function jacSite(m::Ptr{mjModel},d::Ptr{mjData},jacp::Ptr{mjtNum},jacr::Ptr{mjtNum},site::Integer)
   ccall((:mj_jacSite,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum},Cint),m,d,jacp,jacr,site)
end

# compute translation Jacobian of point, and rotation Jacobian of axis
function jacPointAxis(m::Ptr{mjModel},d::Ptr{mjData},jacPoint::Ptr{mjtNum},jacAxis::Ptr{mjtNum},point::SVector{3, mjtNum},axis::SVector{3, mjtNum},body::Integer)
   ccall((:mj_jacPointAxis,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum},SVector{3, mjtNum},SVector{3, mjtNum},Cint),m,d,jacPoint,jacAxis,point,axis,body)
end

# get id of object with specified name; -1: not found; type is mjtObj
function name2id(m::Ptr{mjModel},_type::Integer,name::String)
   ccall((:mj_name2id,libmujoco),Cint,(Ptr{mjModel},Cint,Cstring),m,_type,name)
end

# get name of object with specified id; 0: invalid type or id; type is mjtObj
function id2name(m::Ptr{mjModel},_type::Integer,id::Integer)
   ccall((:mj_id2name,libmujoco),Cstring,(Ptr{mjModel},Cint,Cint),m,_type,id)
end

# convert sparse inertia matrix M into full matrix
function fullM(m::Ptr{mjModel},dst::Ptr{mjtNum},M::PtrVec)
   ccall((:mj_fullM,libmujoco),Void,(Ptr{mjModel},Ptr{mjtNum},Ptr{mjtNum}),m,dst,M)
end

# multiply vector by inertia matrix
function mulM(m::Ptr{mjModel},d::Ptr{mjData},res::Ptr{mjtNum},vec::PtrVec)
   ccall((:mj_mulM,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum}),m,d,res,vec)
end

# apply cartesian force and torque (outside xfrc_applied mechanism)
function applyFT(m::Ptr{mjModel},d::Ptr{mjData},force::Ptr{mjtNum},torque::Ptr{mjtNum},point::Ptr{mjtNum},body::Integer,qfrc_target::Ptr{mjtNum})
   ccall((:mj_applyFT,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint,Ptr{mjtNum}),m,d,force,torque,point,body,qfrc_target)
end

# compute object 6D velocity in object-centered frame, world/local orientation
function objectVelocity(m::Ptr{mjModel},d::Ptr{mjData},objtype::Integer,objid::Integer,res::Ptr{mjtNum},flg_local::Integer)
   ccall((:mj_objectVelocity,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Cint,Cint,Ptr{mjtNum},Cint),m,d,objtype,objid,res,flg_local)
end

# compute object 6D acceleration in object-centered frame, world/local orientation
function objectAcceleration(m::Ptr{mjModel},d::Ptr{mjData},objtype::Integer,objid::Integer,res::Ptr{mjtNum},flg_local::Integer)
   ccall((:mj_objectAcceleration,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Cint,Cint,Ptr{mjtNum},Cint),m,d,objtype,objid,res,flg_local)
end

# compute velocity by finite-differencing two positions
function differentiatePos(m::Ptr{mjModel},qvel::Ptr{mjtNum},dt::mjtNum,qpos1::Ptr{mjtNum},qpos2::Ptr{mjtNum})
   ccall((:mj_differentiatePos,libmujoco),Void,(Ptr{mjModel},Ptr{mjtNum},mjtNum,Ptr{mjtNum},Ptr{mjtNum}),m,qvel,dt,qpos1,qpos2)
end

# extract 6D force:torque for one contact, in contact frame
function contactForce(m::Ptr{mjModel},d::Ptr{mjData},id::Integer,result::Ptr{mjtNum})
   ccall((:mj_contactForce,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Cint,Ptr{mjtNum}),m,d,id,result)
end

# integrate position with given velocity
function integratePos(m::Ptr{mjModel},qpos::Ptr{mjtNum},qvel::Ptr{mjtNum},dt::mjtNum)
   ccall((:mj_integratePos,libmujoco),Void,(Ptr{mjModel},Ptr{mjtNum},Ptr{mjtNum},mjtNum),m,qpos,qvel,dt)
end

# normalize all quaterions in qpos-type vector
function normalizeQuat(m::Ptr{mjModel},qpos::Ptr{mjtNum})
   ccall((:mj_normalizeQuat,libmujoco),Void,(Ptr{mjModel},Ptr{mjtNum}),m,qpos)
end

# map from body local to global Cartesian coordinates
function local2Global(d::Ptr{mjData},xpos::Ptr{mjtNum},xmat::Ptr{mjtNum},pos::Ptr{mjtNum},quat::Ptr{mjtNum},body::Integer)
   ccall((:mj_local2Global,libmujoco),Void,(Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint),d,xpos,xmat,pos,quat,body)
end

# sum all body masses
function getTotalmass(m::Ptr{mjModel})
   ccall((:mj_getTotalmass,libmujoco),mjtNum,(Ptr{mjModel},),m)
end

# scale body masses and inertias to achieve specified total mass
function setTotalmass(m::Ptr{mjModel},newmass::mjtNum)
   ccall((:mj_setTotalmass,libmujoco),Void,(Ptr{mjModel},mjtNum),m,newmass)
end

# version number: 1.0.2 is encoded as 102 #TODO comment??
function version()
   ccall((:mj_version,libmujoco),Cint,())
end

#---------------------- Abstract interaction -------------------------------------------

# set default camera
function mjv_defaultCamera(cam::Ptr{mjvCamera})
   ccall((:mjv_defaultCamera,libmujoco),Void,(Ptr{mjvCamera},),cam)
end

# set default perturbation
function mjv_defaultPerturb(pert::Ptr{mjvPerturb})
   ccall((:mjv_defaultPerturb,libmujoco),Void,(Ptr{mjvPerturb},),pert)
end

# transform pose from room to model space
function mjv_room2model(modelpos::Ptr{mjtNum},modelquat::Ptr{mjtNum},roompos::Ptr{mjtNum},roomquat::Ptr{mjtNum},scn::Ptr{mjvScene})
   ccall((:mjv_room2model,libmujoco),Void,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Ptr{mjvScene}),modelpos,modelquat,roompos,roomquat,scn)
end

# transform pose from model to room space
function mjv_model2room(roompos::Ptr{mjtNum},roomquat::Ptr{mjtNum},modelpos::Ptr{mjtNum},modelquat::Ptr{mjtNum},scn::Ptr{mjvScene})
   ccall((:mjv_model2room,libmujoco),Void,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Ptr{mjvScene}),roompos,roomquat,modelpos,modelquat,scn)
end

# get camera info in model space: average left and right OpenGL cameras
function mjv_cameraInModel(headpos::Ptr{mjtNum},forward::Ptr{mjtNum},scn::Ptr{mjvScene})
   ccall((:mjv_cameraInModel,libmujoco),Void,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjvScene}),headpos,forward,scn)
end

# get camera info in room space: average left and right OpenGL cameras
function mjv_cameraInRoom(headpos::Ptr{mjtNum},forward::Ptr{mjtNum},scn::Ptr{mjvScene})
   ccall((:mjv_cameraInRoom,libmujoco),Void,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjvScene}),headpos,forward,scn)
end

# get frustum height at unit distance from camera; average left and right OpenGL cameras
function mjv_frustumHeight(scn::Ptr{mjvScene})
   ccall((:mjv_frustumHeight,libmujoco),mjtNum,(Ptr{mjvScene},),scn)
end

# rotate 3D vec in horizontal plane by angle between (0,1) and (forward_x,forward_y)
function mjv_alignToCamera(res::Ptr{mjtNum},vec::Ptr{mjtNum},forward::Ptr{mjtNum})
   ccall((:mjv_alignToCamera,libmujoco),Void,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum}),res,vec,forward)
end

# move camera with mouse; action is mjtMouse
function mjv_moveCamera(m::Ptr{mjModel},action::Integer,reldx::mjtNum,reldy::mjtNum,scn::Ptr{mjvScene},cam::Ptr{mjvCamera})
   ccall((:mjv_moveCamera,libmujoco),Void,(Ptr{mjModel},Cint,mjtNum,mjtNum,Ptr{mjvScene},Ptr{mjvCamera}),m,action,reldx,reldy,scn,cam)
end

# move perturb object with mouse; action is mjtMouse
function mjv_movePerturb(m::Ptr{mjModel},d::Ptr{mjData},action::Integer,reldx::mjtNum,reldy::mjtNum,scn::Ptr{mjvScene},pert::Ptr{mjvPerturb})
   ccall((:mjv_movePerturb,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Cint,mjtNum,mjtNum,Ptr{mjvScene},Ptr{mjvPerturb}),m,d,action,reldx,reldy,scn,pert)
end

# move model with mouse; action is mjtMouse
function mjv_moveModel(m::Ptr{mjModel},action::Integer,reldx::mjtNum,reldy::mjtNum,roomup::Ptr{mjtNum},scn::Ptr{mjvScene})
   ccall((:mjv_moveModel,libmujoco),Void,(Ptr{mjModel},Cint,mjtNum,mjtNum,Ptr{mjtNum},Ptr{mjvScene}),m,action,reldx,reldy,roomup,scn)
end

# copy perturb pos,quat from selected body; set scale for perturbation
function mjv_initPerturb(m::Ptr{mjModel},d::Ptr{mjData},scn::Ptr{mjvScene},pert::Ptr{mjvPerturb})
   ccall((:mjv_initPerturb,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Ptr{mjvScene},Ptr{mjvPerturb}),m,d,scn,pert)
end

# set perturb pos,quat in d->mocap when selected body is mocap, and in d->qpos otherwise
#  d->qpos written only if flg_paused and subtree root for selected body has free joint
function mjv_applyPerturbPose(m::Ptr{mjModel},d::Ptr{mjData},pert::Ptr{mjvPerturb},flg_paused::Integer)
   ccall((:mjv_applyPerturbPose,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Ptr{mjvPerturb},Cint),m,d,pert,flg_paused)
end

# set perturb force,torque in d->xfrc_applied, if selected body is dynamic
function mjv_applyPerturbForce(m::Ptr{mjModel},d::Ptr{mjData},pert::Ptr{mjvPerturb})
   ccall((:mjv_applyPerturbForce,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Ptr{mjvPerturb}),m,d,pert)
end

#---------------------- Asbtract visualization -----------------------------------------

# set default visualization options
function mjv_defaultOption(opt::Ptr{mjvOption})
   ccall((:mjv_defaultOption,libmujoco),Void,(Ptr{mjvOption},),opt)
end

# allocate and init abstract scene
function mjv_makeScene(scn::Ptr{mjvScene},maxgeom::Integer)
   ccall((:mjv_makeScene,libmujoco),Void,(Ptr{mjvScene},Cint),scn,maxgeom)
end

# free abstract scene
function mjv_freeScene(scn::Ptr{mjvScene})
   ccall((:mjv_freeScene,libmujoco),Void,(Ptr{mjvScene},),scn)
end

# update entire scene
function mjv_updateScene(m::Ptr{mjModel},d::Ptr{mjData},opt::Ptr{mjvOption},pert::Ptr{mjvPerturb},cam::Ptr{mjvCamera},catmask::Integer,scn::Ptr{mjvScene})
   ccall((:mjv_updateScene,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Ptr{mjvOption},Ptr{mjvPerturb},Ptr{mjvCamera},Cint,Ptr{mjvScene}),m,d,opt,pert,cam,catmask,scn)
end

# add geoms from selected categories to existing scene
function mjv_addGeoms(m::Ptr{mjModel},d::Ptr{mjData},opt::Ptr{mjvOption},pert::Ptr{mjvPerturb},catmask::Integer,scn::Ptr{mjvScene})
   ccall((:mjv_addGeoms,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Ptr{mjvOption},Ptr{mjvPerturb},Cint,Ptr{mjvScene}),m,d,opt,pert,catmask,scn)
end

# update camera only
function mjv_updateCamera(m::Ptr{mjModel},d::Ptr{mjData},cam::Ptr{mjvCamera},scn::Ptr{mjvScene})
   ccall((:mjv_updateCamera,libmujoco),Void,(Ptr{mjModel},Ptr{mjData},Ptr{mjvCamera},Ptr{mjvScene}),m,d,cam,scn)
end

#---------------------- OpenGL rendering -----------------------------------------------

# set default mjrContext
function mjr_defaultContext(con::Ptr{mjrContext})
   ccall((:mjr_defaultContext,libmujoco),Void,(Ptr{mjrContext},),con)
end

# allocate resources in custom OpenGL context; fontscale is mjtFontScale
function mjr_makeContext(m::Ptr{mjModel},con::Ptr{mjrContext},fontscale::Integer)
   ccall((:mjr_makeContext,libmujoco),Void,(Ptr{mjModel},Ptr{mjrContext},Cint),m,con,fontscale)
end

# free resources in custom OpenGL context, set to default
function mjr_freeContext(con::Ptr{mjrContext})
   ccall((:mjr_freeContext,libmujoco),Void,(Ptr{mjrContext},),con)
end

# (re) upload texture to GPU
function mjr_uploadTexture(m::Ptr{mjModel},con::Ptr{mjrContext},texid::Integer)
   ccall((:mjr_uploadTexture,libmujoco),Void,(Ptr{mjModel},Ptr{mjrContext},Cint),m,con,texid)
end

# (re) upload mesh to GPU
function mjr_uploadMesh(m::Ptr{mjModel},con::Ptr{mjrContext},meshid::Integer)
   ccall((:mjr_uploadMesh,libmujoco),Void,(Ptr{mjModel},Ptr{mjrContext},Cint),m,con,meshid)
end

# (re) upload height field to GPU
function mjr_uploadHField(m::Ptr{mjModel},con::Ptr{mjrContext},hfieldid::Integer)
   ccall((:mjr_uploadHField,libmujoco),Void,(Ptr{mjModel},Ptr{mjrContext},Cint),m,con,hfieldid)
end

# set OpenGL framebuffer for rendering: mjFB_WINDOW or mjFB_OFFSCREEN
#  if only one buffer is available, set that buffer and ignore framebuffer argument
function mjr_setBuffer(framebuffer::Integer,con::Ptr{mjrContext})
   ccall((:mjr_setBuffer,libmujoco),Void,(Cint,Ptr{mjrContext}),framebuffer,con)
end

# read pixels from current OpenGL framebuffer to client buffer
#  viewport is in OpenGL framebuffer; client buffer starts at (0,0)
function mjr_readPixels(rgb::Ptr{Cuchar},depth::Ptr{Cfloat},viewport::mjrRect,con::Ptr{mjrContext})
   ccall((:mjr_readPixels,libmujoco),Void,(Ptr{Cuchar},Ptr{Cfloat},mjrRect,Ptr{mjrContext}),rgb,depth,viewport,con)
end

# draw pixels from client buffer to current OpenGL framebuffer
#  viewport is in OpenGL framebuffer; client buffer starts at (0,0)
function mjr_drawPixels(rgb::Ptr{Cuchar},depth::Ptr{Cfloat},viewport::mjrRect,con::Ptr{mjrContext})
   ccall((:mjr_drawPixels,libmujoco),Void,(Ptr{Cuchar},Ptr{Cfloat},mjrRect,Ptr{mjrContext}),rgb,depth,viewport,con)
end

# blit from src viewpoint in current framebuffer to dst viewport in other framebuffer
#  if src, dst have different size and flg_depth==0, color is interpolated with GL_LINEAR
function mjr_blitBuffer(src::mjrRect,dst::mjrRect,flg_color::Integer,flg_depth::Integer,con::Ptr{mjrContext})
   ccall((:mjr_blitBuffer,libmujoco),Void,(mjrRect,mjrRect,Cint,Cint,Ptr{mjrContext}),src,dst,flg_color,flg_depth,con)
end

# draw text at (x,y) in relative coordinates; font is mjtFont
function mjr_text(font::Integer,txt::String,con::Ptr{mjrContext},x::Cfloat,y::Cfloat,r::Cfloat,g::Cfloat,b::Cfloat)
   ccall((:mjr_text,libmujoco),Void,(Cint,Cstring,Ptr{mjrContext},Cfloat,Cfloat,Cfloat,Cfloat,Cfloat),font,txt,con,x,y,r,g,b)
end

# draw text overlay; font is mjtFont; gridpos is mjtGridPos
function mjr_overlay(font::Integer,gridpos::Integer,viewport::mjrRect,overlay::String,overlay2::String,con::Ptr{mjrContext})
   ccall((:mjr_overlay,libmujoco),Void,(Cint,Cint,mjrRect,Cstring,Cstring,Ptr{mjrContext}),font,gridpos,viewport,overlay,overlay2,con)
end

# get maximum viewport for active buffer
function mjr_maxViewport(con::Ptr{mjrContext})
   ccall((:mjr_maxViewport,libmujoco),mjrRect,(Ptr{mjrContext},),con)
end

# draw rectangle
function mjr_rectangle(viewport::mjrRect,r::Cfloat,g::Cfloat,b::Cfloat,a::Cfloat)
   ccall((:mjr_rectangle,libmujoco),Void,(mjrRect,Cfloat,Cfloat,Cfloat,Cfloat),viewport,r,g,b,a)
end

# draw lines
function mjr_lines(viewport::mjrRect,nline::Integer,rgb::Ptr{Cfloat},npoint::Ptr{Cint},data::Ptr{mjtNum})
   ccall((:mjr_lines,libmujoco),Void,(mjrRect,Cint,Ptr{Cfloat},Ptr{Cint},Ptr{mjtNum}),viewport,nline,rgb,npoint,data)
end

# 3D rendering
function mjr_render(viewport::mjrRect,scn::Ptr{mjvScene},con::Ptr{mjrContext})
   ccall((:mjr_render,libmujoco),Void,(mjrRect,Ptr{mjvScene},Ptr{mjrContext}),viewport,scn,con)
end

# 3D selection
function mjr_select(viewport::mjrRect,scn::Ptr{mjvScene},con::Ptr{mjrContext},mousex::Integer,mousey::Integer,pos::Ptr{mjtNum},depth::Ptr{mjtNum})
   ccall((:mjr_select,libmujoco),Cint,(mjrRect,Ptr{mjvScene},Ptr{mjrContext},Cint,Cint,Ptr{mjtNum},Ptr{mjtNum}),viewport,scn,con,mousex,mousey,pos,depth)
end

# call glFinish
function mjr_finish()
   ccall((:mjr_finish,libmujoco),Void,())
end

# call glGetError and return result
function mjr_getError()
   ccall((:mjr_getError,libmujoco),Cint,())
end

#---------------------- Utility functions: error and memory ----------------------------

# main error function; does not return to caller
function mju_error(msg::String)
   ccall((:mju_error,libmujoco),Void,(Cstring,),msg)
end

# error function with int argument; msg is a printf format string
function mju_error_i(msg::String,i::Integer)
   ccall((:mju_error_i,libmujoco),Void,(Cstring,Cint),msg,i)
end

# error function with string argument
function mju_error_s(msg::String,text::String)
   ccall((:mju_error_s,libmujoco),Void,(Cstring,Cstring),msg,text)
end

# main warning function; returns to caller
function mju_warning(msg::String)
   ccall((:mju_warning,libmujoco),Void,(Cstring,),msg)
end

# warning function with int argument
function mju_warning_i(msg::String,i::Integer)
   ccall((:mju_warning_i,libmujoco),Void,(Cstring,Cint),msg,i)
end

# warning function with string argument
function mju_warning_s(msg::String,text::String)
   ccall((:mju_warning_s,libmujoco),Void,(Cstring,Cstring),msg,text)
end

# clear user error and memory handlers
function mju_clearHandlers()
   ccall((:mju_clearHandlers,libmujoco),Void,())
end

# allocate memory; byte-align on 8; pad size to multiple of 8
function mju_malloc(size::Integer)
   ccall((:mju_malloc,libmujoco),Ptr{Void},(Cint,),size)
end

# free memory (with free() by default)
function mju_free(ptr::Ptr{Void})
   ccall((:mju_free,libmujoco),Void,(Ptr{Void},),ptr)
end

# high-level warning function: count warnings in mjData, print only the first
function warning(d::Ptr{mjData},warning::Integer,info::Integer)
   ccall((:mj_warning,libmujoco),Void,(Ptr{mjData},Cint,Cint),d,warning,info)
end

#---------------------- Utility functions: basic math ----------------------------------

# set vector to zero
function mju_zero3(res::SVector{3, mjtNum})
   ccall((:mju_zero3,libmujoco),Void,(SVector{3, mjtNum},),res)
end

# copy vector
function mju_copy3(res::SVector{3, mjtNum},data::SVector{3, mjtNum})
   ccall((:mju_copy3,libmujoco),Void,(SVector{3, mjtNum},SVector{3, mjtNum}),res,data)
end

# scale vector
function mju_scl3(res::SVector{3, mjtNum},vec::SVector{3, mjtNum},scl::mjtNum)
   ccall((:mju_scl3,libmujoco),Void,(SVector{3, mjtNum},SVector{3, mjtNum},mjtNum),res,vec,scl)
end

# add vectors
function mju_add3(res::SVector{3, mjtNum},vec1::SVector{3, mjtNum},vec2::SVector{3, mjtNum})
   ccall((:mju_add3,libmujoco),Void,(SVector{3, mjtNum},SVector{3, mjtNum},SVector{3, mjtNum}),res,vec1,vec2)
end

# subtract vectors
function mju_sub3(res::SVector{3, mjtNum},vec1::SVector{3, mjtNum},vec2::SVector{3, mjtNum})
   ccall((:mju_sub3,libmujoco),Void,(SVector{3, mjtNum},SVector{3, mjtNum},SVector{3, mjtNum}),res,vec1,vec2)
end

# add to vector
function mju_addTo3(res::SVector{3, mjtNum},vec::SVector{3, mjtNum})
   ccall((:mju_addTo3,libmujoco),Void,(SVector{3, mjtNum},SVector{3, mjtNum}),res,vec)
end

# add scaled to vector
function mju_addToScl3(res::SVector{3, mjtNum},vec::SVector{3, mjtNum},scl::mjtNum)
   ccall((:mju_addToScl3,libmujoco),Void,(SVector{3, mjtNum},SVector{3, mjtNum},mjtNum),res,vec,scl)
end

# res = vec1 + scl*vec2
function mju_addScl3(res::SVector{3, mjtNum},vec1::SVector{3, mjtNum},vec2::SVector{3, mjtNum},scl::mjtNum)
   ccall((:mju_addScl3,libmujoco),Void,(SVector{3, mjtNum},SVector{3, mjtNum},SVector{3, mjtNum},mjtNum),res,vec1,vec2,scl)
end

# normalize vector, return length before normalization
function mju_normalize3(res::SVector{3, mjtNum})
   ccall((:mju_normalize3,libmujoco),mjtNum,(SVector{3, mjtNum},),res)
end

# compute vector length (without normalizing)
function mju_norm3(vec::SVector{3, mjtNum})
   ccall((:mju_norm3,libmujoco),mjtNum,(SVector{3, mjtNum},),vec)
end

# vector dot-product
function mju_dot3(vec1::SVector{3, mjtNum},vec2::SVector{3, mjtNum})
   ccall((:mju_dot3,libmujoco),mjtNum,(SVector{3, mjtNum},SVector{3, mjtNum}),vec1,vec2)
end

# Cartesian distance between 3D vectors
function mju_dist3(pos1::SVector{3, mjtNum},pos2::SVector{3, mjtNum})
   ccall((:mju_dist3,libmujoco),mjtNum,(SVector{3, mjtNum},SVector{3, mjtNum}),pos1,pos2)
end

# multiply vector by 3D rotation matrix
function mju_rotVecMat(res::SVector{3, mjtNum},vec::SVector{3, mjtNum},mat::SVector{9, mjtNum})
   ccall((:mju_rotVecMat,libmujoco),Void,(SVector{3, mjtNum},SVector{3, mjtNum},SVector{9, mjtNum}),res,vec,mat)
end

# multiply vector by transposed 3D rotation matrix
function mju_rotVecMatT(res::SVector{3, mjtNum},vec::SVector{3, mjtNum},mat::SVector{9, mjtNum})
   ccall((:mju_rotVecMatT,libmujoco),Void,(SVector{3, mjtNum},SVector{3, mjtNum},SVector{9, mjtNum}),res,vec,mat)
end

# vector cross-product, 3D
function mju_cross(res::SVector{3, mjtNum},a::SVector{3, mjtNum},b::SVector{3, mjtNum})
   ccall((:mju_cross,libmujoco),Void,(SVector{3, mjtNum},SVector{3, mjtNum},SVector{3, mjtNum}),res,a,b)
end

# set vector to zero
function mju_zero4(res::SVector{4, mjtNum})
   ccall((:mju_zero4,libmujoco),Void,(SVector{4, mjtNum},),res)
end

# set unit quaterion
function mju_unit4(res::SVector{4, mjtNum})
   ccall((:mju_unit4,libmujoco),Void,(SVector{4, mjtNum},),res)
end

# copy vector
function mju_copy4(res::SVector{4, mjtNum},data::SVector{4, mjtNum})
   ccall((:mju_copy4,libmujoco),Void,(SVector{4, mjtNum},SVector{4, mjtNum}),res,data)
end

# normalize vector, return length before normalization
function mju_normalize4(res::SVector{4, mjtNum})
   ccall((:mju_normalize4,libmujoco),mjtNum,(SVector{4, mjtNum},),res)
end

# set vector to zero
function mju_zero(res::Ptr{mjtNum},n::Integer)
   ccall((:mju_zero,libmujoco),Void,(Ptr{mjtNum},Cint),res,n)
end

# copy vector
function mju_copy(res::Ptr{mjtNum},data::Ptr{mjtNum},n::Integer)
   ccall((:mju_copy,libmujoco),Void,(Ptr{mjtNum},Ptr{mjtNum},Cint),res,data,n)
end

# scale vector
function mju_scl(res::Ptr{mjtNum},vec::Ptr{mjtNum},scl::mjtNum,n::Integer)
   ccall((:mju_scl,libmujoco),Void,(Ptr{mjtNum},Ptr{mjtNum},mjtNum,Cint),res,vec,scl,n)
end

# add vectors
function mju_add(res::Ptr{mjtNum},vec1::Ptr{mjtNum},vec2::Ptr{mjtNum},n::Integer)
   ccall((:mju_add,libmujoco),Void,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint),res,vec1,vec2,n)
end

# subtract vectors
function mju_sub(res::Ptr{mjtNum},vec1::Ptr{mjtNum},vec2::Ptr{mjtNum},n::Integer)
   ccall((:mju_sub,libmujoco),Void,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint),res,vec1,vec2,n)
end

# add to vector
function mju_addTo(res::Ptr{mjtNum},vec::Ptr{mjtNum},n::Integer)
   ccall((:mju_addTo,libmujoco),Void,(Ptr{mjtNum},Ptr{mjtNum},Cint),res,vec,n)
end

# add scaled to vector
function mju_addToScl(res::Ptr{mjtNum},vec::Ptr{mjtNum},scl::mjtNum,n::Integer)
   ccall((:mju_addToScl,libmujoco),Void,(Ptr{mjtNum},Ptr{mjtNum},mjtNum,Cint),res,vec,scl,n)
end

# res = vec1 + scl*vec2
function mju_addScl(res::Ptr{mjtNum},vec1::Ptr{mjtNum},vec2::Ptr{mjtNum},scl::mjtNum,n::Integer)
   ccall((:mju_addScl,libmujoco),Void,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},mjtNum,Cint),res,vec1,vec2,scl,n)
end

# normalize vector, return length before normalization
function mju_normalize(res::Ptr{mjtNum},n::Integer)
   ccall((:mju_normalize,libmujoco),mjtNum,(Ptr{mjtNum},Cint),res,n)
end

# compute vector length (without normalizing)
function mju_norm(res::Ptr{mjtNum},n::Integer)
   ccall((:mju_norm,libmujoco),mjtNum,(Ptr{mjtNum},Cint),res,n)
end

# vector dot-product
function mju_dot(vec1::Ptr{mjtNum},vec2::Ptr{mjtNum},n::Integer)
   ccall((:mju_dot,libmujoco),mjtNum,(Ptr{mjtNum},Ptr{mjtNum},Cint),vec1,vec2,n)
end

# multiply matrix and vector
function mju_mulMatVec(res::Ptr{mjtNum},mat::Ptr{mjtNum},vec::Ptr{mjtNum},nr::Integer,nc::Integer)
   ccall((:mju_mulMatVec,libmujoco),Void,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint,Cint),res,mat,vec,nr,nc)
end

# multiply transposed matrix and vector
function mju_mulMatTVec(res::Ptr{mjtNum},mat::Ptr{mjtNum},vec::Ptr{mjtNum},nr::Integer,nc::Integer)
   ccall((:mju_mulMatTVec,libmujoco),Void,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint,Cint),res,mat,vec,nr,nc)
end

# transpose matrix
function mju_transpose(res::Ptr{mjtNum},mat::Ptr{mjtNum},r::Integer,c::Integer)
   ccall((:mju_transpose,libmujoco),Void,(Ptr{mjtNum},Ptr{mjtNum},Cint,Cint),res,mat,r,c)
end

# multiply matrices
function mju_mulMatMat(res::Ptr{mjtNum},mat1::Ptr{mjtNum},mat2::Ptr{mjtNum},r1::Integer,c1::Integer,c2::Integer)
   ccall((:mju_mulMatMat,libmujoco),Void,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint,Cint,Cint),res,mat1,mat2,r1,c1,c2)
end

# multiply matrices, second argument transposed
function mju_mulMatMatT(res::Ptr{mjtNum},mat1::Ptr{mjtNum},mat2::Ptr{mjtNum},r1::Integer,c1::Integer,r2::Integer)
   ccall((:mju_mulMatMatT,libmujoco),Void,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint,Cint,Cint),res,mat1,mat2,r1,c1,r2)
end

# multiply matrices, first argument transposed
function mju_mulMatTMat(res::Ptr{mjtNum},mat1::Ptr{mjtNum},mat2::Ptr{mjtNum},r1::Integer,c1::Integer,c2::Integer)
   ccall((:mju_mulMatTMat,libmujoco),Void,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint,Cint,Cint),res,mat1,mat2,r1,c1,c2)
end

# compute M*M'; scratch must be at least r*c
function mju_sqrMat(res::Ptr{mjtNum},mat::Ptr{mjtNum},r::Integer,c::Integer,scratch::Ptr{mjtNum},nscratch::Integer)
   ccall((:mju_sqrMat,libmujoco),Void,(Ptr{mjtNum},Ptr{mjtNum},Cint,Cint,Ptr{mjtNum},Cint),res,mat,r,c,scratch,nscratch)
end

# compute M'*diag*M (diag=NULL: compute M'*M)
function mju_sqrMatTD(res::Ptr{mjtNum},mat::Ptr{mjtNum},diag::Ptr{mjtNum},r::Integer,c::Integer)
   ccall((:mju_sqrMatTD,libmujoco),Void,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint,Cint),res,mat,diag,r,c)
end

# coordinate transform of 6D motion or force vector in rotation:translation format
#  rotnew2old is 3-by-3, NULL means no rotation; flg_force specifies force or motion type
function mju_transformSpatial(res::SVector{6, mjtNum},vec::SVector{6, mjtNum},flg_force::Integer,newpos::SVector{3, mjtNum},oldpos::SVector{3, mjtNum},rotnew2old::SVector{9, mjtNum})
   ccall((:mju_transformSpatial,libmujoco),Void,(SVector{6, mjtNum},SVector{6, mjtNum},Cint,SVector{3, mjtNum},SVector{3, mjtNum},SVector{9, mjtNum}),res,vec,flg_force,newpos,oldpos,rotnew2old)
end

#---------------------- Utility functions: quaternions ---------------------------------

# rotate vector by quaternion
function mju_rotVecQuat(res::SVector{3, mjtNum},vec::SVector{3, mjtNum},quat::SVector{4, mjtNum})
   ccall((:mju_rotVecQuat,libmujoco),Void,(SVector{3, mjtNum},SVector{3, mjtNum},SVector{4, mjtNum}),res,vec,quat)
end

# negate quaternion
function mju_negQuat(res::SVector{4, mjtNum},quat::SVector{4, mjtNum})
   ccall((:mju_negQuat,libmujoco),Void,(SVector{4, mjtNum},SVector{4, mjtNum}),res,quat)
end

# muiltiply quaternions
function mju_mulQuat(res::SVector{4, mjtNum},quat1::SVector{4, mjtNum},quat2::SVector{4, mjtNum})
   ccall((:mju_mulQuat,libmujoco),Void,(SVector{4, mjtNum},SVector{4, mjtNum},SVector{4, mjtNum}),res,quat1,quat2)
end

# muiltiply quaternion and axis
function mju_mulQuatAxis(res::SVector{4, mjtNum},quat::SVector{4, mjtNum},axis::SVector{3, mjtNum})
   ccall((:mju_mulQuatAxis,libmujoco),Void,(SVector{4, mjtNum},SVector{4, mjtNum},SVector{3, mjtNum}),res,quat,axis)
end

# convert axisAngle to quaternion
function mju_axisAngle2Quat(res::SVector{4, mjtNum},axis::SVector{3, mjtNum},angle::mjtNum)
   ccall((:mju_axisAngle2Quat,libmujoco),Void,(SVector{4, mjtNum},SVector{3, mjtNum},mjtNum),res,axis,angle)
end

# convert quaternion (corresponding to orientation difference) to 3D velocity
function mju_quat2Vel(res::SVector{3, mjtNum},quat::SVector{4, mjtNum},dt::mjtNum)
   ccall((:mju_quat2Vel,libmujoco),Void,(SVector{3, mjtNum},SVector{4, mjtNum},mjtNum),res,quat,dt)
end

# convert quaternion to 3D rotation matrix
function mju_quat2Mat(res::SVector{9, mjtNum},quat::SVector{4, mjtNum})
   ccall((:mju_quat2Mat,libmujoco),Void,(SVector{9, mjtNum},SVector{4, mjtNum}),res,quat)
end

# convert 3D rotation matrix to quaterion
function mju_mat2Quat(quat::SVector{4, mjtNum},mat::SVector{9, mjtNum})
   ccall((:mju_mat2Quat,libmujoco),Void,(SVector{4, mjtNum},SVector{9, mjtNum}),quat,mat)
end

# time-derivative of quaternion, given 3D rotational velocity
function mju_derivQuat(res::SVector{4, mjtNum},quat::SVector{4, mjtNum},vel::SVector{3, mjtNum})
   ccall((:mju_derivQuat,libmujoco),Void,(SVector{4, mjtNum},SVector{4, mjtNum},SVector{3, mjtNum}),res,quat,vel)
end

# integrate quaterion given 3D angular velocity
function mju_quatIntegrate(quat::SVector{4, mjtNum},vel::SVector{3, mjtNum},scale::mjtNum)
   ccall((:mju_quatIntegrate,libmujoco),Void,(SVector{4, mjtNum},SVector{3, mjtNum},mjtNum),quat,vel,scale)
end

# compute quaternion performing rotation from z-axis to given vector
function mju_quatZ2Vec(quat::SVector{4, mjtNum},vec::SVector{3, mjtNum})
   ccall((:mju_quatZ2Vec,libmujoco),Void,(SVector{4, mjtNum},SVector{3, mjtNum}),quat,vec)
end

#---------------------- Utility functions: poses (pos, quat) ---------------------------

# multiply two poses
function mju_mulPose(posres::SVector{3, mjtNum},quatres::SVector{4, mjtNum},pos1::SVector{3, mjtNum},quat1::SVector{4, mjtNum},pos2::SVector{3, mjtNum},quat2::SVector{4, mjtNum})
   ccall((:mju_mulPose,libmujoco),Void,(SVector{3, mjtNum},SVector{4, mjtNum},SVector{3, mjtNum},SVector{4, mjtNum},SVector{3, mjtNum},SVector{4, mjtNum}),posres,quatres,pos1,quat1,pos2,quat2)
end

# negate pose
function mju_negPose(posres::SVector{3, mjtNum},quatres::SVector{4, mjtNum},pos::SVector{3, mjtNum},quat::SVector{4, mjtNum})
   ccall((:mju_negPose,libmujoco),Void,(SVector{3, mjtNum},SVector{4, mjtNum},SVector{3, mjtNum},SVector{4, mjtNum}),posres,quatres,pos,quat)
end

# transform vector by pose
function mju_trnVecPose(res::SVector{3, mjtNum},pos::SVector{3, mjtNum},quat::SVector{4, mjtNum},vec::SVector{3, mjtNum})
   ccall((:mju_trnVecPose,libmujoco),Void,(SVector{3, mjtNum},SVector{3, mjtNum},SVector{4, mjtNum},SVector{3, mjtNum}),res,pos,quat,vec)
end

#---------------------- Utility functions: matrix decomposition ------------------------

# Cholesky decomposition
function mju_cholFactor(mat::Ptr{mjtNum},diag::Ptr{mjtNum},n::Integer,minabs::mjtNum,minrel::mjtNum,correct::Ptr{mjtNum})
   ccall((:mju_cholFactor,libmujoco),Cint,(Ptr{mjtNum},Ptr{mjtNum},Cint,mjtNum,mjtNum,Ptr{mjtNum}),mat,diag,n,minabs,minrel,correct)
end

# Cholesky backsubstitution: phase&i enables forward(i=1), backward(i=2) pass
function mju_cholBacksub(res::Ptr{mjtNum},mat::Ptr{mjtNum},vec::Ptr{mjtNum},n::Integer,nvec::Integer,phase::Integer)
   ccall((:mju_cholBacksub,libmujoco),Void,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint,Cint,Cint),res,mat,vec,n,nvec,phase)
end

# eigenvalue decomposition of symmetric 3x3 matrix
function mju_eig3(eigval::Ptr{mjtNum},eigvec::Ptr{mjtNum},quat::Ptr{mjtNum},mat::Ptr{mjtNum})
   ccall((:mju_eig3,libmujoco),Cint,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum}),eigval,eigvec,quat,mat)
end

#---------------------- Utility functions: miscellaneous -------------------------------

# muscle FVL curve: prm = (lminrel, lmaxrel, widthrel, vmaxrel, fmax, fvsat)
function mju_muscleFVL(len::mjtNum,vel::mjtNum,lmin::mjtNum,lmax::mjtNum,prm::Ptr{mjtNum})
   ccall((:mju_muscleFVL,libmujoco),mjtNum,(mjtNum,mjtNum,mjtNum,mjtNum,Ptr{mjtNum}),len,vel,lmin,lmax,prm)
end

# muscle passive force: prm = (lminrel, lmaxrel, fpassive)
function mju_musclePassive(len::mjtNum,lmin::mjtNum,lmax::mjtNum,prm::Ptr{mjtNum})
   ccall((:mju_musclePassive,libmujoco),mjtNum,(mjtNum,mjtNum,mjtNum,Ptr{mjtNum}),len,lmin,lmax,prm)
end

# pneumatic cylinder dynamics
function mju_pneumatic(len::mjtNum,len0::mjtNum,vel::mjtNum,prm::Ptr{mjtNum},act::mjtNum,ctrl::mjtNum,timestep::mjtNum,jac::Ptr{mjtNum})
   ccall((:mju_pneumatic,libmujoco),mjtNum,(mjtNum,mjtNum,mjtNum,Ptr{mjtNum},mjtNum,mjtNum,mjtNum,Ptr{mjtNum}),len,len0,vel,prm,act,ctrl,timestep,jac)
end

# convert contact force to pyramid representation
function mju_encodePyramid(pyramid::Ptr{mjtNum},force::Ptr{mjtNum},mu::Ptr{mjtNum},dim::Integer)
   ccall((:mju_encodePyramid,libmujoco),Void,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint),pyramid,force,mu,dim)
end

# convert pyramid representation to contact force
function mju_decodePyramid(force::Ptr{mjtNum},pyramid::Ptr{mjtNum},mu::Ptr{mjtNum},dim::Integer)
   ccall((:mju_decodePyramid,libmujoco),Void,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint),force,pyramid,mu,dim)
end

# integrate spring-damper analytically, return pos(dt)
function mju_springDamper(pos0::mjtNum,vel0::mjtNum,Kp::mjtNum,Kv::mjtNum,dt::mjtNum)
   ccall((:mju_springDamper,libmujoco),mjtNum,(mjtNum,mjtNum,mjtNum,mjtNum,mjtNum),pos0,vel0,Kp,Kv,dt)
end

# min function, single evaluation of a and b
function mju_min(a::mjtNum,b::mjtNum)
   ccall((:mju_min,libmujoco),mjtNum,(mjtNum,mjtNum),a,b)
end

# max function, single evaluation of a and b
function mju_max(a::mjtNum,b::mjtNum)
   ccall((:mju_max,libmujoco),mjtNum,(mjtNum,mjtNum),a,b)
end

# sign function
function mju_sign(x::mjtNum)
   ccall((:mju_sign,libmujoco),mjtNum,(mjtNum,),x)
end

# round to nearest integer
function mju_round(x::mjtNum)
   ccall((:mju_round,libmujoco),Cint,(mjtNum,),x)
end

# convert type id (mjtObj) to type name
function mju_type2Str(_type::Integer)
   ccall((:mju_type2Str,libmujoco),Cstring,(Cint,),_type)
end

# convert type name to type id (mjtObj)
function mju_str2Type(str::String)
   ccall((:mju_str2Type,libmujoco),Cint,(Cstring,),str)
end

# warning text
function mju_warningText(warning::Integer,info::Integer)
   ccall((:mju_warningText,libmujoco),Cstring,(Cint,Cint),warning,info)
end

# return 1 if nan or abs(x)>mjMAXVAL, 0 otherwise
function mju_isBad(x::mjtNum)
   ccall((:mju_isBad,libmujoco),Cint,(mjtNum,),x)
end

# return 1 if all elements are 0
function mju_isZero(vec::Ptr{mjtNum},n::Integer)
   ccall((:mju_isZero,libmujoco),Cint,(Ptr{mjtNum},Cint),vec,n)
end

# standard normal random number generator (optional second number)
function mju_standardNormal(num2::Ptr{mjtNum})
   ccall((:mju_standardNormal,libmujoco),mjtNum,(Ptr{mjtNum},),num2)
end

# convert from float to mjtNum
function mju_f2n(res::Ptr{mjtNum},vec::Ptr{Cfloat},n::Integer)
   ccall((:mju_f2n,libmujoco),Void,(Ptr{mjtNum},Ptr{Cfloat},Cint),res,vec,n)
end

# convert from mjtNum to float
function mju_n2f(res::Ptr{Cfloat},vec::Ptr{mjtNum},n::Integer)
   ccall((:mju_n2f,libmujoco),Void,(Ptr{Cfloat},Ptr{mjtNum},Cint),res,vec,n)
end

# convert from double to mjtNum
function mju_d2n(res::Ptr{mjtNum},vec::Ptr{Cdouble},n::Integer)
   ccall((:mju_d2n,libmujoco),Void,(Ptr{mjtNum},Ptr{Cdouble},Cint),res,vec,n)
end

# convert from mjtNum to double
function mju_n2d(res::Ptr{Cdouble},vec::Ptr{mjtNum},n::Integer)
   ccall((:mju_n2d,libmujoco),Void,(Ptr{Cdouble},Ptr{mjtNum},Cint),res,vec,n)
end

# insertion sort, increasing order
function mju_insertionSort(list::Ptr{mjtNum},n::Integer)
   ccall((:mju_insertionSort,libmujoco),Void,(Ptr{mjtNum},Cint),list,n)
end



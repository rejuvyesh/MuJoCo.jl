#MARKSTACK(d::jlData) = return mj.get(d, :pstack)
#FREESTACK(d::jlData, mark::Cint) = mj.set(d, :pstack, mark)

# Skipping MacroDefinition: mjDISABLED ( x ) ( m -> opt . disableflags & ( x ) )
# Skipping MacroDefinition: mjENABLED ( x ) ( m -> opt . enableflags & ( x ) )
#MAX(a,b) = max(a,b)
#MIN(a,b) = min(a,b)

## user error and memory handlers
# extern void  (*mju_user_error)(const char*);
# extern void  (*mju_user_warning)(const char*);
# extern void* (*mju_user_malloc)(size_t);
# extern void  (*mju_user_free)(void*);
#
## callbacks extending computation pipeline
# extern mjfGeneric  mjcb_passive;
# extern mjfGeneric  mjcb_control;
# extern mjfSensor   mjcb_sensor;
# extern mjfTime     mjcb_time;
# extern mjfAct      mjcb_act_dyn;
# extern mjfAct      mjcb_act_gain;
# extern mjfAct      mjcb_act_bias;
# extern mjfSolImp   mjcb_sol_imp;
# extern mjfSolRef   mjcb_sol_ref;
#
## collision function table
# extern mjfCollision mjCOLLISIONFUNC[mjNGEOMTYPES][mjNGEOMTYPES];

# p = convert(Ptr{Ptr{Cchar}}, cglobal((:mjDISABLESTRING, libmujoco)))
# all_p = unsafe_wrap(Array, p, mj.NDISABLE)
# DISABLESTRING = [ unsafe_string(all_p[i]) for i=1:Int(mj.NDISABLE) ]

global const DISABLESTRING = ["Constraint","Equality","Frictionloss","Limit","Contact","Passive","Gravity","Clampctrl","Warmstart","Filterparent","Actuation","Refsafe"]
global const ENABLESTRING  = ["Override","Energy","Fwdinv","Sensornoise"]
global const TIMERSTRING   = ["step","forward","inverse","position",
                              "velocity","actuation","acceleration","constraint",
                              "pos_kinematics","pos_inertia","pos_collision","pos_make","pos_project"]
global const LABELSTRING   = ["None","Body","Joint","Geom","Site","Camera","Light","Tendon","Actuator","Constraint","Skin","Selection","SelPoint","ContactForce"]
global const FRAMESTRING   = ["None","Body","Geom","Site","Camera","Light","World"]
global const VISSTRING     = ["Convex Hull"     "0"  "H";
                              "Texture"         "1"  "X";
                              "Joint"           "0"  "J";
                              "Actuator"        "0"  "U";
                              "Camera"          "0"  "Q";
                              "Light"           "0"  "Z";
                              "Tendon"          "1"  "V";
                              "Range Finder"    "1"  "Y";
                              "Constraint"      "0"  "N";
                              "Inertia"         "0"  "I";
                              "Perturb Force"   "0"  "B";
                              "Perturb Object"  "1"  "O";
                              "Contact Point"   "0"  "C";
                              "Contact Force"   "0"  "F";
                              "Contact Split"   "0"  "P";
                              "Transparent"     "0"  "T";
                              "Auto Connect"    "0"  "A";
                              "Center of Mass"  "0"  "M";
                              "Select Point"    "0"  "E";
                              "Static Body"     "1"  "D";
                              "Skin"            "1"  ";"]
global const RNDSTRING     = ["Shadow"      "1"  "S";
                              "Wireframe"   "0"  "W";
                              "Reflection"  "1"  "R";
                              "Additive"    "0"  "L";
                              "Skybox"      "1"  "K";
                              "Fog"         "0"  "G";
                              "Haze"        "1"  "/";
                              "Segment"     "0"  ",";
                              "Id Color"    "0"  "."]

const PV{T} = Union{Ptr{T},AbstractVector{T},Ptr{Cvoid}}

import Base.unsafe_convert
unsafe_convert(::Type{Ptr{mjModel}}, m::jlModel) = m.m
unsafe_convert(::Type{Ptr{mjData}}, d::jlData) = d.d
const MODEL = Union{Ptr{mjModel},jlModel}
const DATA = Union{Ptr{mjData},jlData}


#---------------------- License activation and certificate (mutex-protected) -----------

""""activate license, call mju_error on failure; return 1 if ok, 0 if failure"""
function mj_activate(filename::String)
   ccall((:mj_activate,libmujoco),Cint,(Cstring,),filename)
end

""""deactivate license, free memory"""
function mj_deactivate()
   ccall((:mj_deactivate,libmujoco),Cvoid,())
end

"""server: generate certificate question"""
function mj_certQuestion(question::Vector{mjtNum})
   @assert length(question) >= 16
   ccall((:mj_certQuestion,libmujoco),Cvoid,(Vector{mjtNum},),question)
end

"""client: generate certificate answer given question"""
function mj_certAnswer(question::Vector{mjtNum},answer::Vector{mjtNum})
   @assert length(question) >= 16
   @assert length(answer) >= 16
   ccall((:mj_certAnswer,libmujoco),Cvoid,(Vector{mjtNum},Vector{mjtNum}),question,answer)
end

"""server: check certificate question-answer pair; return 1 if match, 0 if mismatch"""
function mj_certCheck(question::Vector{mjtNum},answer::Vector{mjtNum})
   @assert length(question) >= 16
   @assert length(answer) >= 16
   ccall((:mj_certCheck,libmujoco),Cint,(Vector{mjtNum},Vector{mjtNum}),question,answer)
end

#---------------------- Virtual file system --------------------------------------------

"""Initialize VFS to empty (no deallocation)."""
function mj_defaultVFS(vfs::Ptr{mjVFS})
   ccall((:mj_defaultVFS,libmujoco),Cvoid,(Ptr{mjVFS},),vfs)
end

"""Add file to VFS, return 0: success, 1: full, 2: repeated name, -1: not found on disk."""
function mj_addFileVFS(vfs::Ptr{mjVFS},directory::String,filename::String)
   ccall((:mj_addFileVFS,libmujoco),Cint,(Ptr{mjVFS},Cstring,Cstring),vfs,directory,filename)
end

"""Make empty file in VFS, return 0: success, 1: full, 2: repeated name."""
function mj_makeEmptyFileVFS(vfs::Ptr{mjVFS},filename::String,filesize::Integer)
   ccall((:mj_makeEmptyFileVFS,libmujoco),Cint,(Ptr{mjVFS},Cstring,Cstring),vfs,filename,filesize)
end

"""Return file index in VFS, or -1 if not found in VFS."""
function mj_findFileVFS(vfs::Ptr{mjVFS},filename::String)
   ccall((:mj_findFileVFS,libmujoco),Cint,(Ptr{mjVFS},Cstring),vfs,filename)
end

"""Delete file from VFS, return 0: success, -1: not found in VFS."""
function mj_deleteFileVFS(vfs::Ptr{mjVFS},filename::String)
   ccall((:mj_deleteFileVFS,libmujoco),Cint,(Ptr{mjVFS},Cstring),vfs,filename)
end

"""Delete all files from VFS."""
function mj_deleteVFS(vfs::Ptr{mjVFS})
   ccall((:mj_deleteVFS,libmujoco),Cvoid,(Ptr{mjVFS},),vfs)
end


#---------------------- XML parser and C++ compiler (mutex-protected) ------------------

"""
parse XML file or string in MJCF or URDF format, compile it, return low-level model
if xmlstring is not NULL, it has precedence over filename
error can be NULL; otherwise assumed to have size error_sz
"""
function mj_loadXML(filename::String,vfs::Union{Ptr{mjVFS},Ptr{Cvoid}}=C_NULL)
   errsz = 1000
   err = Vector{UInt8}(undef, errsz) 
   m=ccall((:mj_loadXML,libmujoco),Ptr{mjModel},(Cstring,Cstring,Ptr{UInt8},Cint),filename,vfs,err,errsz)
   if m == C_NULL
      err[end] = 0;
      @warn "Error in XML loading: $(unsafe_string(pointer(err))) \nCheck path and .xml file."
      return nothing
   end
   return m
end

"""
update XML data structures with info from low-level model, save as MJCF
rror can be NULL; otherwise assumed to have size error_sz
"""
function mj_saveLastXML(filename::String,m::MODEL,error::String,error_sz::Integer)
   ccall((:mj_saveLastXML,libmujoco),Cint,(Cstring,Ptr{mjModel},Cstring,Cint),filename,m,error,error_sz)
end

function mj_freeLastXML()
   ccall((:mj_freeLastXML,libmujoco),Cvoid,(Cvoid,),Cvoid)
end

""" print internal XML schema as plain text or HTML, with style-padding or &nbsp;"""
function mj_printSchema(filename::String,buffer::String,buffer_sz::Integer,flg_html::Integer,flg_pad::Integer)
   ccall((:mj_printSchema,libmujoco),Cint,(Cstring,Cstring,Cint,Cint,Cint),filename,buffer,buffer_sz,flg_html,flg_pad)
end

#---------------------- Main entry points ----------------------------------------------
""" advance simulation: use control callback, no external force, RK4 available"""
function mj_step(m::MODEL,d::DATA)
   ccall((:mj_step,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

""" advance simulation in two steps: before external force/control is set by user"""
function mj_step1(m::MODEL,d::DATA)
   ccall((:mj_step1,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

""" advance simulation in two steps: after external force/control is set by user"""
function mj_step2(m::MODEL,d::DATA)
   ccall((:mj_step2,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

""" forward dynamics"""
function mj_forward(m::MODEL,d::DATA)
   ccall((:mj_forward,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

""" inverse dynamics"""
function mj_inverse(m::MODEL,d::DATA)
   ccall((:mj_inverse,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

""" forward dynamics with skip; skipstage is mjtStage"""
function mj_forwardSkip(m::MODEL,d::DATA,skipstage::Integer,skipsensorenergy::Integer)
   ccall((:mj_forwardSkip,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Cint,Cint),m,d,skipstage,skipsensorenergy)
end

""" inverse dynamics with skip; skipstage is mjtStage"""
function mj_inverseSkip(m::MODEL,d::DATA,skipstage::Integer,skipsensorenergy::Integer)
   ccall((:mj_inverseSkip,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Cint,Cint),m,d,skipstage,skipsensorenergy)
end

#---------------------- Model and data initialization ----------------------------------
"""set default options for length range computation."""
function mj_defaultLROpt(opt::Ptr{mjLROpt})
   ccall((:mj_defaultLROpt,libmujoco),Cvoid,(Ptr{mjLROpt},),opt)
end

"""set default solver parameters"""
function mj_defaultSolRefImp(solref::PV{mjtNum},solimp::PV{mjtNum})
   ccall((:mj_defaultSolRefImp,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum}),solref,solimp)
end

"""set physics options to default values"""
function mj_defaultOption(opt::Ptr{mjOption})
   ccall((:mj_defaultOption,libmujoco),Cvoid,(Ptr{mjOption},),opt)
end

"""set visual options to default values"""
function mj_defaultVisual(vis::Ptr{mjVisual})
   ccall((:mj_defaultVisual,libmujoco),Cvoid,(Ptr{mjVisual},),vis)
end

"""copy Model; allocate new if dest is NULL"""
function mj_copyModel(dest::Ptr{mjModel},src::Ptr{mjModel})
   ccall((:mj_copyModel,libmujoco),Ptr{mjModel},(Ptr{mjModel},Ptr{mjModel}),dest,src)
end

"""save model to binary file or memory buffer (buffer has precedence if szbuf>0)"""
function mj_saveModel(m::MODEL,filename::String,buffer::Ptr{Cvoid},buffer_sz::Integer)
   ccall((:mj_saveModel,libmujoco),Cvoid,(Ptr{mjModel},Cstring,Ptr{Cvoid},Cint),m,filename,buffer,buffer_sz)
end

"""load model from binary file or memory buffer (buffer has precedence if szbuf>0)"""
function mj_loadModel(filename::String,buffer::Ptr{Cvoid},buffer_sz::Integer)
   ccall((:mj_loadModel,libmujoco),Ptr{mjModel},(Cstring,Ptr{Cvoid},Cint),filename,buffer,buffer_sz)
end

"""de-allocate model"""
function mj_deleteModel(m::MODEL)
   ccall((:mj_deleteModel,libmujoco),Cvoid,(Ptr{mjModel},),m)
end

"""size of buffer needed to hold model"""
function mj_sizeModel(m::MODEL)
   ccall((:mj_sizeModel,libmujoco),Cint,(Ptr{mjModel},),m)
end

"""allocate Data correponding to given model"""
function mj_makeData(m::MODEL)
   ccall((:mj_makeData,libmujoco),Ptr{mjData},(Ptr{mjModel},),m)
end

"""copy Data"""
function mj_copyData(dest::Ptr{mjData},m::MODEL,src::Ptr{mjData})
   ccall((:mj_copyData,libmujoco),Ptr{mjData},(Ptr{mjData},Ptr{mjModel},Ptr{mjData}),dest,m,src)
end

"""set data to defaults"""
function mj_resetData(m::MODEL,d::DATA)
   ccall((:mj_resetData,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""set data to defaults, fill everything else with debug_value"""
function mj_resetDataDebug(m::MODEL,d::DATA,debug_value::Cuchar)
   ccall((:mj_resetDataDebug,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Cuchar),m,d,debug_value)
end

"""reset data, set fields from specified keyframe"""
function mj_resetDataKeyframe(m::MODEL,d::DATA,key::Integer)
   ccall((:mj_resetDataKeyframe,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Cint),m,d,key)
end

"""Data stack allocate"""
function mj_stackAlloc(d::DATA,size::Integer)
   ccall((:mj_stackAlloc,libmujoco),Ptr{mjtNum},(Ptr{mjData},Cint),d,size)
end

"""de-allocate data"""
function mj_deleteData(d::DATA)
   ccall((:mj_deleteData,libmujoco),Cvoid,(Ptr{mjData},),d)
end

"""reset callbacks to defaults"""
function mj_resetCallbacks()
   ccall((:mj_resetCallbacks,libmujoco),Cvoid,())
end

"""set constant fields of Model"""
function mj_setConst(m::MODEL,d::DATA)
   ccall((:mj_setConst,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""Set actuator_lengthrange for specified actuator; return 1 if ok, 0 if error."""
function mj_setLengthRange(m::MODEL,d::DATA,index::Integer,opt::Ptr{mjLROpt},error::String,error_sz::Integer)
   ccall((:mj_setLengthRange,libmujoco),Cint,(Ptr{mjModel},Ptr{mjData},Cint,Ptr{mjLROpt},Cstring,Cint),m,d,index,opt,error,error_sz);
end

#---------------------- Printing -------------------------------------------------------

"""print model to text file"""
function mj_printModel(m::MODEL,filename::String)
   ccall((:mj_printModel,libmujoco),Cvoid,(Ptr{mjModel},Cstring),m,filename)
end

"""print data to text file"""
function mj_printData(m::MODEL,d::DATA,filename::String)
   ccall((:mj_printData,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Cstring),m,d,filename)
end

#---------------------- Components: forward dynamics -----------------------------------

"""position-dependent computations"""
function mj_fwdPosition(m::MODEL,d::DATA)
   ccall((:mj_fwdPosition,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""velocity-dependent computations"""
function mj_fwdVelocity(m::MODEL,d::DATA)
   ccall((:mj_fwdVelocity,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""compute actuator force"""
function mj_fwdActuation(m::MODEL,d::DATA)
   ccall((:mj_fwdActuation,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""add up all non-constraint forces, compute qacc_unc"""
function mj_fwdAcceleration(m::MODEL,d::DATA)
   ccall((:mj_fwdAcceleration,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""constraint solver"""
function mj_fwdConstraint(m::MODEL,d::DATA)
   ccall((:mj_fwdConstraint,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""Euler integrator, semi-implicit in velocity"""
function mj_Euler(m::MODEL,d::DATA)
   ccall((:mj_Euler,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""Runge-Kutta explicit order-N integrator"""
function mj_RungeKutta(m::MODEL,d::DATA,N::Integer)
   ccall((:mj_RungeKutta,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Cint),m,d,N)
end

#---------------------- Components: inverse dynamics -----------------------------------

"""position-dependent computations"""
function mj_invPosition(m::MODEL,d::DATA)
   ccall((:mj_invPosition,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""velocity-dependent computations"""
function mj_invVelocity(m::MODEL,d::DATA)
   ccall((:mj_invVelocity,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""constraint solver"""
function mj_invConstraint(m::MODEL,d::DATA)
   ccall((:mj_invConstraint,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""compare forward and inverse dynamics, without changing results of forward dynamics"""
function mj_compareFwdInv(m::MODEL,d::DATA)
   ccall((:mj_compareFwdInv,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

#---------------------- Components: forward and inverse dynamics -----------------------

"""position-dependent sensors"""
function mj_sensorPos(m::MODEL,d::DATA)
   ccall((:mj_sensorPos,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""velocity-dependent sensors"""
function mj_sensorVel(m::MODEL,d::DATA)
   ccall((:mj_sensorVel,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""acceleration/force-dependent sensors"""
function mj_sensorAcc(m::MODEL,d::DATA)
   ccall((:mj_sensorAcc,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""position-dependent energy (potential)"""
function mj_energyPos(m::MODEL,d::DATA)
   ccall((:mj_energyPos,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""velocity-dependent energy (kinetic)"""
function mj_energyVel(m::MODEL,d::DATA)
   ccall((:mj_energyVel,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

#---------------------- Sub-components -------------------------------------------------

"""check positions; reset if bad"""
function mj_checkPos(m::MODEL,d::DATA)
   ccall((:mj_checkPos,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""check velocities; reset if bad"""
function mj_checkVel(m::MODEL,d::DATA)
   ccall((:mj_checkVel,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""check accelerations; reset if bad"""
function mj_checkAcc(m::MODEL,d::DATA)
   ccall((:mj_checkAcc,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""forward kinematics"""
function mj_kinematics(m::MODEL,d::DATA)
   ccall((:mj_kinematics,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""map inertias and motion dofs to global frame centered at CoM"""
function mj_comPos(m::MODEL,d::DATA)
   ccall((:mj_comPos,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""compute camera and light positions and orientations"""
function mj_camlight(m::MODEL,d::DATA)
   ccall((:mj_camlight,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""compute tendon lengths, velocities and moment arms"""
function mj_tendon(m::MODEL,d::DATA)
   ccall((:mj_tendon,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""compute actuator transmission lengths and moments"""
function mj_transmission(m::MODEL,d::DATA)
   ccall((:mj_transmission,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""composite rigid body inertia algorithm"""
function mj_crb(m::MODEL,d::DATA)
   ccall((:mj_crb,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""sparse L'*D*L factorizaton of the inertia matrix"""
function mj_factorM(m::MODEL,d::DATA)
   ccall((:mj_factorM,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""Solve linear system M * x = y using factorization:  x = inv(L'*D*L)*y"""
function mj_solveM(m::MODEL,d::DATA,x::PV{mjtNum},y::PV{mjtNum},n::Integer)
   ccall((:mj_solveM,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum},Cint),m,d,x,y,n)
end

"""Half of linear solve:  x = sqrt(inv(D))*inv(L')*y"""
function mj_solveM2(m::MODEL,d::DATA,x::PV{mjtNum},y::PV{mjtNum},n::Integer)
   ccall((:mj_solveM2,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum},Cint),m,d,x,y,n)
end

"""compute cvel, cdof_dot"""
function mj_comVel(m::MODEL,d::DATA)
   ccall((:mj_comVel,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""spring-dampers and body viscosity"""
function mj_passive(m::MODEL,d::DATA)
   ccall((:mj_passive,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""subtree linear velocity and angular momentum"""
function mj_subtreeVel(m::MODEL,d::DATA)
   ccall((:mj_subtreeVel,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term"""
function mj_rne(m::MODEL,d::DATA,flg_acc::Integer,result::PV{mjtNum})
   ccall((:mj_rne,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Cint,Ptr{mjtNum}),m,d,flg_acc,result)
end

"""RNE with complete data: compute cacc, cfrc_ext, cfrc_int"""
function mj_rnePostConstraint(m::MODEL,d::DATA)
   ccall((:mj_rnePostConstraint,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""collision detection"""
function mj_collision(m::MODEL,d::DATA)
   ccall((:mj_collision,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""construct constraints"""
function mj_makeConstraint(m::MODEL,d::DATA)
   ccall((:mj_makeConstraint,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""compute dense matrices: efc_AR, e_ARchol, fc_half, fc_AR"""
function mj_projectConstraint(m::MODEL,d::DATA)
   ccall((:mj_projectConstraint,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""compute efc_vel, efc_aref"""
function mj_referenceConstraint(m::MODEL,d::DATA)
   ccall((:mj_referenceConstraint,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData}),m,d)
end

"""Compute efc_state, efc_force, qfrc_constraint, and (optionally) cone Hessians.
If cost is not NULL, set *cost = s(jar) where jar = Jac*qacc-aref.
"""
function mj_constraintUpdate(m::MODEL,d::DATA,jar::PV{mjtNum},cost::PV{mjtNum},flg_coneHessian::Integer)
   ccall((:mj_constraintUpdate,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},PV{mjtNum},PV{mjtNum},Integer),m,d,jar,cost,flg_coneHessian)
end



#---------------------- Support functions ----------------------------------------------

"""add contact to d->contact list; return 0 if success; 1 if buffer full"""
function mj_addContact(m::MODEL,d::DATA,con::Ptr{mjContact})
   ccall((:mj_addContact,libmujoco),Cint,(Ptr{mjModel},Ptr{mjData},Ptr{mjContact}),m,d,con)
end

"""determine type of friction cone"""
function mj_isPyramidal(m::MODEL)
   ccall((:mj_isPyramidal,libmujoco),Cint,(Ptr{mjModel},),m)
end

"""determine type of constraint Jacobian"""
function mj_isSparse(m::MODEL)
   ccall((:mj_isSparse,libmujoco),Cint,(Ptr{mjModel},),m)
end

"""Determine type of solver (PGS is dual, CG and Newton are primal)."""
function mj_isDual(m::MODEL)
   ccall((:mj_isDual,libmujoco),Cint,(Ptr{mjModel},),m)
end

"""multiply Jacobian by vector"""
function mj_mulJacVec(m::MODEL,d::DATA,res::PV{mjtNum},vec::PV{mjtNum})
   ccall((:mj_mulJacVec,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum}),m,d,res,vec)
end

"""multiply JacobianT by vector"""
function mj_mulJacTVec(m::MODEL,d::DATA,res::PV{mjtNum},vec::PV{mjtNum})
   ccall((:mj_mulJacTVec,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum}),m,d,res,vec)
end

"""compute 3/6-by-nv Jacobian of global point attached to given body"""
function mj_jac(m::MODEL,d::DATA,jacp::PV{mjtNum},jacr::PV{mjtNum},point::SVector{3, mjtNum},body::Integer)
   ccall((:mj_jac,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum},SVector{3, mjtNum},Cint),m,d,jacp,jacr,point,body)
end

"""compute body frame Jacobian"""
function mj_jacBody(m::MODEL,d::DATA,jacp::PV{mjtNum},jacr::PV{mjtNum},body::Integer)
   ccall((:mj_jacBody,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum},Cint),m,d,jacp,jacr,body)
end

"""compute body center-of-mass Jacobian"""
function mj_jacBodyCom(m::MODEL,d::DATA,jacp::PV{mjtNum},jacr::PV{mjtNum},body::Integer)
   ccall((:mj_jacBodyCom,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum},Cint),m,d,jacp,jacr,body)
end

"""compute geom Jacobian"""
function mj_jacGeom(m::MODEL,d::DATA,jacp::PV{mjtNum},jacr::PV{mjtNum},geom::Integer)
   ccall((:mj_jacGeom,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum},Cint),m,d,jacp,jacr,geom)
end

"""compute site Jacobian"""
function mj_jacSite(m::MODEL,d::DATA,jacp::PV{mjtNum},jacr::PV{mjtNum},site::Integer)
   ccall((:mj_jacSite,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum},Cint),m,d,jacp,jacr,site)
end

"""compute translation Jacobian of point, and rotation Jacobian of axis"""
function mj_jacPointAxis(m::MODEL,d::DATA,jacPoint::PV{mjtNum},jacAxis::PV{mjtNum},point::SVector{3, mjtNum},axis::SVector{3, mjtNum},body::Integer)
   ccall((:mj_jacPointAxis,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum},SVector{3, mjtNum},SVector{3, mjtNum},Cint),m,d,jacPoint,jacAxis,point,axis,body)
end

"""get id of object with specified name; -1: not found; type is mjtObj"""
function mj_name2id(m::MODEL,_type::Integer,name::String)
   ccall((:mj_name2id,libmujoco),Cint,(Ptr{mjModel},Cint,Cstring),m,_type,pointer(name))
end

"""get name of object with specified id; 0: invalid type or id; type is mjtObj"""
function mj_id2name(m::MODEL,_type::Integer,id::Integer)
   name=ccall((:mj_id2name,libmujoco),Cstring,(Ptr{mjModel},Cint,Cint),m,_type,id-1) # julia to c indexing
   return unsafe_string(name)
end

"""convert sparse inertia matrix M into full matrix"""
function mj_fullM(m::MODEL,dst::PV{mjtNum},M::PV{mjtNum})
   ccall((:mj_fullM,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjtNum},Ptr{mjtNum}),m,dst,M)
end

"""multiply vector by inertia matrix"""
function mj_mulM(m::MODEL,d::DATA,res::PV{mjtNum},vec::PV{mjtNum})
   ccall((:mj_mulM,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum}),m,d,res,vec)
end

"""Multiply vector by (inertia matrix)^(1/2)."""
function mj_mulM2(m::MODEL,d::DATA,res::PV{mjtNum},vec::PV{mjtNum})
   ccall((:mj_mulM2,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum}),m,d,res,vec)
end

"""Add inertia matrix to destination matrix.
Destination can be sparse uncompressed, or dense when all int* are NULL
"""
function mj_addM(m::MODEL,d::DATA,dst::PV{mjtNum},rownnz::Vector{Integer},rowadr::Vector{Integer},colind::Vector{Integer})
   ccall((:mj_addM,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{Cint},Ptr{Cint},Ptr{Cint}),m,d,dst,rownnz,rowadr,colind)
end

"""apply cartesian force and torque (outside xfrc_applied mechanism)"""
function mj_applyFT(m::MODEL,d::DATA,force::PV{mjtNum},torque::PV{mjtNum},point::PV{mjtNum},body::Integer,qfrc_target::PV{mjtNum})
   ccall((:mj_applyFT,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint,Ptr{mjtNum}),m,d,force,torque,point,body,qfrc_target)
end

"""compute object 6D velocity in object-centered frame, world/local orientation"""
function mj_objectVelocity(m::MODEL,d::DATA,objtype::Integer,objid::Integer,res::PV{mjtNum},flg_local::Integer)
   ccall((:mj_objectVelocity,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Cint,Cint,Ptr{mjtNum},Cint),m,d,objtype,objid,res,flg_local)
end

"""compute object 6D acceleration in object-centered frame, world/local orientation"""
function mj_objectAcceleration(m::MODEL,d::DATA,objtype::Integer,objid::Integer,res::PV{mjtNum},flg_local::Integer)
   ccall((:mj_objectAcceleration,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Cint,Cint,Ptr{mjtNum},Cint),m,d,objtype,objid,res,flg_local)
end

"""extract 6D force:torque for one contact, in contact frame"""
function mj_contactForce(m::MODEL,d::DATA,id::Integer,result::PV{mjtNum})
   ccall((:mj_contactForce,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Cint,Ptr{mjtNum}),m,d,id,result)
end

"""compute velocity by finite-differencing two positions"""
function mj_differentiatePos(m::MODEL,qvel::PV{mjtNum},dt::mjtNum,qpos1::PV{mjtNum},qpos2::PV{mjtNum})
   ccall((:mj_differentiatePos,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjtNum},mjtNum,Ptr{mjtNum},Ptr{mjtNum}),m,qvel,dt,qpos1,qpos2)
end

"""integrate position with given velocity"""
function mj_integratePos(m::MODEL,qpos::PV{mjtNum},qvel::PV{mjtNum},dt::mjtNum)
   ccall((:mj_integratePos,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjtNum},Ptr{mjtNum},mjtNum),m,qpos,qvel,dt)
end

"""normalize all quaterions in qpos-type vector"""
function mj_normalizeQuat(m::MODEL,qpos::PV{mjtNum})
   ccall((:mj_normalizeQuat,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjtNum}),m,qpos)
end

"""map from body local to global Cartesian coordinates"""
function mj_local2Global(d::DATA,xpos::PV{mjtNum},xmat::PV{mjtNum},pos::PV{mjtNum},quat::PV{mjtNum},body::Integer,sameframe::mjtByte)
   ccall((:mj_local2Global,libmujoco),Cvoid,(Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint,mjtByte),d,xpos,xmat,pos,quat,body,sameframe)
end

"""sum all body masses"""
function mj_getTotalmass(m::MODEL)
   ccall((:mj_getTotalmass,libmujoco),mjtNum,(Ptr{mjModel},),m)
end

"""scale body masses and inertias to achieve specified total mass"""
function mj_setTotalmass(m::MODEL,newmass::mjtNum)
   ccall((:mj_setTotalmass,libmujoco),Cvoid,(Ptr{mjModel},mjtNum),m,newmass)
end

"""version number: 1.0.2 is encoded as 102 #TODO comment??"""
function mj_version()
   ccall((:mj_version,libmujoco),Cint,())
end

#---------------------- Ray collisions -------------------------------------------------

"""
Intersect ray (pnt+x*vec, x>=0) with visible geoms, except geoms in bodyexclude.
Return geomid and distance (x) to nearest surface, or -1 if no intersection.
geomgroup, flg_static are as in mjvOption; geomgroup==NULL skips group exclusion.
"""
function mj_ray(m::MODEL,d::DATA,pnt::PV{mjtNum},vec::PV{mjtNum},
                geomgroup::Vector{mjtByte},flg_static::mjtByte,bodyexclude::Integer, 
                geomid::Vector{Integer})
   ccall((:mj_ray,libmujoco),mjtNum,
         (Ptr{mjModel},Ptr{mjData},Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtByte},mjtByte,Cint,Ptr{Cint}),
         m,d,pnt,vec,
         geomgroup,flg_static,bodyexclude, 
         geomid)
end

"""Interect ray with hfield, return nearest distance or -1 if no intersection."""
function mj_rayHfield(m::MODEL,d::DATA,geomid::Integer,
                      pnt::PV{mjtNum},vec::PV{mjtNum});
   ccall((:mj_rayHfield,libmujoco),mjtNum,(Ptr{mjModel},Ptr{mjData},Cint,Ptr{mjtNum},Ptr{mjtNum}),m,d,geomid,pnt,vec)
end

"""Interect ray with mesh, return nearest distance or -1 if no intersection."""
function mj_rayMesh(m::MODEL,d::DATA,geomid::Integer,pnt::PV{mjtNum},vec::PV{mjtNum})
   ccall((:mj_rayMesh,libmujoco),mjtNum,(Ptr{mjModel},Ptr{mjData},Cint,Ptr{mjtNum},Ptr{mjtNum}),m,d,geomid,pnt,vec)
end


#---------------------- Abstract interaction -------------------------------------------

"""set default camera"""
function mjv_defaultCamera(cam::Ptr{mjvCamera})
   ccall((:mjv_defaultCamera,libmujoco),Cvoid,(Ptr{mjvCamera},),cam)
end

"""set default perturbation"""
function mjv_defaultPerturb(pert::Ptr{mjvPerturb})
   ccall((:mjv_defaultPerturb,libmujoco),Cvoid,(Ptr{mjvPerturb},),pert)
end

"""transform pose from room to model space"""
function mjv_room2model(modelpos::PV{mjtNum},modelquat::PV{mjtNum},roompos::PV{mjtNum},roomquat::PV{mjtNum},scn::Ptr{mjvScene})
   ccall((:mjv_room2model,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Ptr{mjvScene}),modelpos,modelquat,roompos,roomquat,scn)
end

"""transform pose from model to room space"""
function mjv_model2room(roompos::PV{mjtNum},roomquat::PV{mjtNum},modelpos::PV{mjtNum},modelquat::PV{mjtNum},scn::Ptr{mjvScene})
   ccall((:mjv_model2room,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Ptr{mjvScene}),roompos,roomquat,modelpos,modelquat,scn)
end

"""get camera info in model space: average left and right OpenGL cameras"""
function mjv_cameraInModel(headpos::PV{mjtNum},forward::PV{mjtNum},scn::Ptr{mjvScene})
   ccall((:mjv_cameraInModel,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjvScene}),headpos,forward,scn)
end

"""get camera info in room space: average left and right OpenGL cameras"""
function mjv_cameraInRoom(headpos::PV{mjtNum},forward::PV{mjtNum},scn::Ptr{mjvScene})
   ccall((:mjv_cameraInRoom,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjvScene}),headpos,forward,scn)
end

"""get frustum height at unit distance from camera; average left and right OpenGL cameras"""
function mjv_frustumHeight(scn::Ptr{mjvScene})
   ccall((:mjv_frustumHeight,libmujoco),mjtNum,(Ptr{mjvScene},),scn)
end

"""rotate 3D vec in horizontal plane by angle between (0,1) and (forward_x,forward_y)"""
function mjv_alignToCamera(res::PV{mjtNum},vec::PV{mjtNum},forward::PV{mjtNum})
   ccall((:mjv_alignToCamera,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum}),res,vec,forward)
end

"""move camera with mouse; action is mjtMouse"""
function mjv_moveCamera(m::MODEL,action::Integer,reldx::mjtNum,reldy::mjtNum,scn::Ptr{mjvScene},cam::Ptr{mjvCamera})
   ccall((:mjv_moveCamera,libmujoco),Cvoid,(Ptr{mjModel},Cint,mjtNum,mjtNum,Ptr{mjvScene},Ptr{mjvCamera}),m,action,reldx,reldy,scn,cam)
end

"""move perturb object with mouse; action is mjtMouse"""
function mjv_movePerturb(m::MODEL,d::DATA,action::Integer,reldx::mjtNum,reldy::mjtNum,scn::Ptr{mjvScene},pert::Ptr{mjvPerturb})
   ccall((:mjv_movePerturb,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Cint,mjtNum,mjtNum,Ptr{mjvScene},Ptr{mjvPerturb}),m,d,action,reldx,reldy,scn,pert)
end

"""move model with mouse; action is mjtMouse"""
function mjv_moveModel(m::MODEL,action::Integer,reldx::mjtNum,reldy::mjtNum,roomup::PV{mjtNum},scn::Ptr{mjvScene})
   ccall((:mjv_moveModel,libmujoco),Cvoid,(Ptr{mjModel},Cint,mjtNum,mjtNum,Ptr{mjtNum},Ptr{mjvScene}),m,action,reldx,reldy,roomup,scn)
end

"""copy perturb pos,quat from selected body; set scale for perturbation"""
function mjv_initPerturb(m::MODEL,d::DATA,scn::Ptr{mjvScene},pert::Ptr{mjvPerturb})
   ccall((:mjv_initPerturb,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Ptr{mjvScene},Ptr{mjvPerturb}),m,d,scn,pert)
end

"""set perturb pos,quat in d->mocap when selected body is mocap, and in d->qpos otherwise
d->qpos written only if flg_paused and subtree root for selected body has free joint
"""
function mjv_applyPerturbPose(m::MODEL,d::DATA,pert::Ptr{mjvPerturb},flg_paused::Integer)
   ccall((:mjv_applyPerturbPose,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Ptr{mjvPerturb},Cint),m,d,pert,flg_paused)
end

"""set perturb force,torque in d->xfrc_applied, if selected body is dynamic"""
function mjv_applyPerturbForce(m::MODEL,d::DATA,pert::Ptr{mjvPerturb})
   ccall((:mjv_applyPerturbForce,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Ptr{mjvPerturb}),m,d,pert)
end

"""Return the average of two OpenGL cameras."""
function mjv_averageCamera(cam1::Ptr{mjvGLCamera}, cam2::Ptr{mjvGLCamera})
   ccall((:mjv_averageCamera,libmujoco),mjvGLCamera,(Ptr{mjvGLCamera},Ptr{mjvGLCamera}),cam1,cam2)
end

"""Select geom or skin with mouse, return bodyid; -1: none selected."""
function mjv_select(m::MODEL,d::DATA,vopt::Ptr{mjvOption},
                    aspectratio::mjtNum, relx::mjtNum, rely::mjtNum,
                    scn::Ptr{mjvScene}, selpnt::PV{mjtNum}, geomid::PV{Integer}, skinid::PV{Integer})
   ccall((:mjv_select,libmujoco),Cint,(Ptr{mjModel},Ptr{mjData},Ptr{mjvOption},
                                       mjtNum,mjtNum,mjtNum,
                                       Ptr{mjvScene},Ptr{mjtNum},Ptr{Cint},Ptr{Cint}),
         m,d,vopt,aspectratio,relx,rely,
         scn,selpnt,geomid,skinid)
end


#---------------------- Asbtract visualization -----------------------------------------

"""set default visualization options"""
function mjv_defaultOption(opt::Ptr{mjvOption})
   ccall((:mjv_defaultOption,libmujoco),Cvoid,(Ptr{mjvOption},),opt)
end

"""Set default figure."""
function mjv_defaultFigure(fig::Ptr{mjvFigure})
   ccall((:mjv_defaultFigure,libmujoco),Cvoid,(Ptr{mjvFigure},),fig)
end

"""Initialize given geom fields when not NULL, set the rest to their default values."""
function mjv_initGeom(geom::Ptr{mjvGeom},_type::Integer,size::PV{mjtNum},
                      pos::PV{mjtNum},mat::PV{mjtNum},rgba::Ptr{Float32})
   ccall((:mjv_initGeom,libmujoco),Cvoid,(Ptr{mjvGeom},Cint,Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Ptr{Cfloat}),geom,_type,size,pos,mat,rbga)
end

"""Set (type, size, pos, mat) for connector-type geom between given points.
Assume that mjv_initGeom was already called to set all other properties.
"""
function mjv_makeConnector(geom::Ptr{mjvGeom},_type::Integer,width::mjtNum, 
                           a0::mjtNum,a1::mjtNum,a2::mjtNum, 
                           b0::mjtNum,b1::mjtNum,b2::mjtNum)
   ccall((:mjv_makeConnector,libmujoco),Cvoid,(Ptr{mjvGeom},Cint,mjtNum,mjtNum,mjtNum,mjtNum,mjtNum,mjtNum,mjtNum),geom,_type,width,a0,a1,a2,b0,b1,b2)
end

"""Set default abstract scene."""
function mjv_defaultScene(scn::Ptr{mjvScene})
   ccall((:mjv_defaultScene,libmujoco),Cvoid,(Ptr{mjvScene},),scn)
end

"""allocate and init abstract scene"""
function mjv_makeScene(scn::Ptr{mjvScene},maxgeom::Integer)
   ccall((:mjv_makeScene,libmujoco),Cvoid,(Ptr{mjvScene},Cint),scn,maxgeom)
end

"""free abstract scene"""
function mjv_freeScene(scn::Ptr{mjvScene})
   ccall((:mjv_freeScene,libmujoco),Cvoid,(Ptr{mjvScene},),scn)
end

"""update entire scene"""
function mjv_updateScene(m::MODEL,d::DATA,opt::Ptr{mjvOption},pert::Ptr{mjvPerturb},cam::Ptr{mjvCamera},catmask::Integer,scn::Ptr{mjvScene})
   ccall((:mjv_updateScene,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Ptr{mjvOption},Ptr{mjvPerturb},Ptr{mjvCamera},Cint,Ptr{mjvScene}),m,d,opt,pert,cam,catmask,scn)
end

"""add geoms from selected categories"""
function mjv_addGeoms(m::MODEL,d::DATA,opt::Ptr{mjvOption},pert::Ptr{mjvPerturb},catmask::Integer,scn::Ptr{mjvScene})
   ccall((:mjv_addGeoms,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Ptr{mjvOption},Ptr{mjvPerturb},Cint,Ptr{mjvScene}),m,d,opt,pert,catmask,scn)
end

"""Make list of lights."""
function mjv_makeLights(m::MODEL,d::DATA,scn::Ptr{mjvScene})
   ccall((:mjv_makeLights,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Ptr{mjvScene}),m,d,scn)
end

"""update camera only"""
function mjv_updateCamera(m::MODEL,d::DATA,cam::Ptr{mjvCamera},scn::Ptr{mjvScene})
   ccall((:mjv_updateCamera,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Ptr{mjvCamera},Ptr{mjvScene}),m,d,cam,scn)
end

"""Update skins."""
function mjv_updateSkin(m::MODEL,d::DATA,scn::Ptr{mjvScene})
   ccall((:mjv_updateSkin,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjData},Ptr{mjvScene}),m,d,scn)
end


#---------------------- OpenGL rendering -----------------------------------------------

"""set default mjrContext"""
function mjr_defaultContext(con::Ptr{mjrContext})
   ccall((:mjr_defaultContext,libmujoco),Cvoid,(Ptr{mjrContext},),con)
end

"""allocate resources in custom OpenGL context; fontscale is mjtFontScale"""
function mjr_makeContext(m::MODEL,con::Ptr{mjrContext},fontscale::Integer)
   ccall((:mjr_makeContext,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjrContext},Cint),m,con,fontscale)
end

"""Change font of existing context."""
function mjr_changeFont(fontscale::Integer,con::Ptr{mjrContext})
   ccall((:mjr_changeFont,libmujoco),Cvoid,(Cint,Ptr{mjrContext}),fontscale,con)
end

"""Add Aux buffer with given index to context; free previous Aux buffer."""
function mjr_addAux(index::Integer,width::Integer,height::Integer,samples::Integer,con::Ptr{mjrContext})
   ccall((:mjr_addAux,libmujoco),Cvoid,(Cint,Cint,Cint,Cint,Ptr{mjrContext}),index,width,height,samples,con)
end

"""free resources in custom OpenGL context, set to default"""
function mjr_freeContext(con::Ptr{mjrContext})
   ccall((:mjr_freeContext,libmujoco),Cvoid,(Ptr{mjrContext},),con)
end

"""(re) upload texture to GPU"""
function mjr_uploadTexture(m::MODEL,con::Ptr{mjrContext},texid::Integer)
   ccall((:mjr_uploadTexture,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjrContext},Cint),m,con,texid)
end

"""(re) upload mesh to GPU"""
function mjr_uploadMesh(m::MODEL,con::Ptr{mjrContext},meshid::Integer)
   ccall((:mjr_uploadMesh,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjrContext},Cint),m,con,meshid)
end

"""(re) upload height field to GPU"""
function mjr_uploadHField(m::MODEL,con::Ptr{mjrContext},hfieldid::Integer)
   ccall((:mjr_uploadHField,libmujoco),Cvoid,(Ptr{mjModel},Ptr{mjrContext},Cint),m,con,hfieldid)
end

"""Make con->currentBuffer current again."""
function mjr_restoreBuffer(con::Ptr{mjrContext})
   ccall((:mjr_restoreBuffer,libmujoco),Cvoid,(Ptr{mjrContext},),con)
end

"""set OpenGL framebuffer for rendering: mjFB_WINDOW or mjFB_OFFSCREEN
if only one buffer is available, set that buffer and ignore framebuffer argument
"""
function mjr_setBuffer(framebuffer::Integer,con::Ptr{mjrContext})
   ccall((:mjr_setBuffer,libmujoco),Cvoid,(Cint,Ptr{mjrContext}),framebuffer,con)
end

"""read pixels from current OpenGL framebuffer to client buffer
viewport is in OpenGL framebuffer; client buffer starts at (0,0)
"""
function mjr_readPixels(rgb::PV{Cuchar},depth::PV{Cfloat},viewport::mjrRect,con::Ptr{mjrContext})
   ccall((:mjr_readPixels,libmujoco),Cvoid,(Ptr{Cuchar},Ptr{Cfloat},mjrRect,Ptr{mjrContext}),rgb,depth,viewport,con)
end

"""draw pixels from client buffer to current OpenGL framebuffer
viewport is in OpenGL framebuffer; client buffer starts at (0,0)
"""
function mjr_drawPixels(rgb::PV{Cuchar},depth::PV{Cfloat},viewport::mjrRect,con::Ptr{mjrContext})
   ccall((:mjr_drawPixels,libmujoco),Cvoid,(Ptr{Cuchar},Ptr{Cfloat},mjrRect,Ptr{mjrContext}),rgb,depth,viewport,con)
end

"""blit from src viewpoint in current framebuffer to dst viewport in other framebuffer
if src, dst have different size and flg_depth==0, color is interpolated with GL_LINEAR
"""
function mjr_blitBuffer(src::mjrRect,dst::mjrRect,flg_color::Integer,flg_depth::Integer,con::Ptr{mjrContext})
   ccall((:mjr_blitBuffer,libmujoco),Cvoid,(mjrRect,mjrRect,Cint,Cint,Ptr{mjrContext}),src,dst,flg_color,flg_depth,con)
end

"""Set Aux buffer for custom OpenGL rendering (call restoreBuffer when done)."""
function mjr_setAux(index::Integer,con::Ptr{mjrContext})
   ccall((:mjr_setAux,libmujoco),Cvoid,(Cint,Ptr{mjrContext}),index,con)
end

"""Blit from Aux buffer to con->currentBuffer."""
function mjr_blitAux(index::Integer,src::mjrRect,left::Integer,bottom::Integer,con::Ptr{mjrContext})
   ccall((:mjr_blitAux,libmujoco),Cvoid,(Cint,mjrRect,Cint,Cint,Ptr{mjrContext}),index,src,left,bottom,con)
end

"""draw text at (x,y) in relative coordinates; font is mjtFont"""
function mjr_text(font::Integer,txt::String,con::Ptr{mjrContext},x::Cfloat,y::Cfloat,r::Cfloat,g::Cfloat,b::Cfloat)
   ccall((:mjr_text,libmujoco),Cvoid,(Cint,Cstring,Ptr{mjrContext},Cfloat,Cfloat,Cfloat,Cfloat,Cfloat),font,txt,con,x,y,r,g,b)
end

"""draw text overlay; font is mjtFont; gridpos is mjtGridPos"""
function mjr_overlay(font::Integer,gridpos::Integer,viewport::mjrRect,overlay::String,overlay2::String,con::Ptr{mjrContext})
   ccall((:mjr_overlay,libmujoco),Cvoid,(Cint,Cint,mjrRect,Cstring,Cstring,Ptr{mjrContext}),font,gridpos,viewport,overlay,overlay2,con)
end

"""get maximum viewport for active buffer"""
function mjr_maxViewport(con::Ptr{mjrContext})
   ccall((:mjr_maxViewport,libmujoco),mjrRect,(Ptr{mjrContext},),con)
end

"""draw rectangle"""
function mjr_rectangle(viewport::mjrRect,r::Cfloat,g::Cfloat,b::Cfloat,a::Cfloat)
   ccall((:mjr_rectangle,libmujoco),Cvoid,(mjrRect,Cfloat,Cfloat,Cfloat,Cfloat),viewport,r,g,b,a)
end

"""draw lines"""
function mjr_figure(viewport::mjrRect,fig::Ptr{mjvFigure},con::Ptr{mjrContext})
   ccall((:mjr_figure,libmujoco),Cvoid,(mjrRect,Ptr{mjvFigure},Ptr{mjrContext}),viewport,fig,con)
end

"""3D rendering"""
function mjr_render(viewport::mjrRect,scn::Ptr{mjvScene},con::Ptr{mjrContext})
   ccall((:mjr_render,libmujoco),Cvoid,(mjrRect,Ptr{mjvScene},Ptr{mjrContext}),viewport,scn,con)
end

"""call glFinish"""
function mjr_finish()
   ccall((:mjr_finish,libmujoco),Cvoid,())
end

"""call glGetError and return result"""
function mjr_getError()
   ccall((:mjr_getError,libmujoco),Cint,())
end

"""Find first rectangle containing mouse, -1: not found."""
function mjr_findRect(x::Integer,y::Integer,nrect::Integer,rect::PV{mjrRect})
   ccall((:mjr_findRect,libmujoco),Cint,(Cint,Cint,Cint,Ptr{mjrRect}),x,y,nrect,rect)
end


#---------------------- Utility functions: error and memory ----------------------------

"""print matrix to screen"""
function mju_printMat(mat::PV{mjtNum},nr::Integer,nc::Integer)
   ccall((:mju_printMat,libmujoco),Cvoid,(Ptr{mjtNum},Cint,Cint),mat,nr,nc)
end

"""Print sparse matrix to screen."""
function mju_printMatSparse(mat::PV{mjtNum},nr::Integer,rownnz::Vector{Integer},rowadr::Vector{Integer},colind::Vector{Integer})
   ccall((:mju_printMatSparse,libmujoco),Cvoid,(Ptr{mjtNum},Cint,Ptr{Cint},Ptr{Cint},Ptr{Cint}),mat,nr,rownnz,rowadr,colind)
end

"""Interect ray with pure geom, return nearest distance or -1 if no intersection."""
function mju_rayGeom(pos::PV{mjtNum},mat::PV{mjtNum},size::PV{mjtNum},pnt::PV{mjtNum},vec::PV{mjtNum},geomtype::Integer)
   ccall((:mju_rayGeom,libmujoco),mjtNum,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint),pos,mat,size,pnt,vec,geomtype)
end

"""Interect ray with skin, return nearest vertex id."""
function mju_raySkin(nface::Integer,nvert::Integer,face::PV{Integer},vert::PV{Float32},pnt::PV{mjtNum},vec::PV{mjtNum},vertid::PV{Integer})
   ccall((:mju_raySkin,libmujoco),mjtNum,(Integer,Integer,Ptr{Cint},Ptr{Cfloat},Ptr{mjtNum},Ptr{mjtNum},Ptr{Cint}),nface,nvert,face,veert,pnt,vec,vertid)
end

"""main error function; does not return to caller"""
function mju_error(msg::String)
   ccall((:mju_error,libmujoco),Cvoid,(Cstring,),msg)
end

"""error function mju_with int argument; msg is a printf format string"""
function mju_error_i(msg::String,i::Integer)
   ccall((:mju_error_i,libmujoco),Cvoid,(Cstring,Cint),msg,i)
end

"""error function mju_with string argument"""
function mju_error_s(msg::String,text::String)
   ccall((:mju_error_s,libmujoco),Cvoid,(Cstring,Cstring),msg,text)
end

"""main warning function; returns to caller"""
function mju_warning(msg::String)
   ccall((:mju_warning,libmujoco),Cvoid,(Cstring,),msg)
end

"""warning function mju_with int argument"""
function mju_warning_i(msg::String,i::Integer)
   ccall((:mju_warning_i,libmujoco),Cvoid,(Cstring,Cint),msg,i)
end

"""warning function mju_with string argument"""
function mju_warning_s(msg::String,text::String)
   ccall((:mju_warning_s,libmujoco),Cvoid,(Cstring,Cstring),msg,text)
end

"""clear user error and memory handlers"""
function mju_clearHandlers()
   ccall((:mju_clearHandlers,libmujoco),Cvoid,())
end

"""allocate memory; byte-align on 8; pad size to multiple of 8"""
function mju_malloc(size::Integer)
   ccall((:mju_malloc,libmujoco),Ptr{Cvoid},(Cint,),size)
end

"""free memory (with free() by default)"""
function mju_free(ptr::Ptr{Cvoid})
   ccall((:mju_free,libmujoco),Cvoid,(Ptr{Cvoid},),ptr)
end

"""high-level warning function: count warnings in Data, print only the first"""
function mj_warning(d::DATA,warning::Integer,info::Integer)
   ccall((:mj_warning,libmujoco),Cvoid,(Ptr{mjData},Cint,Cint),d,warning,info)
end

"""Write [datetime, type: message] to MUJOCO_LOG.TXT."""
function mju_writeLog(_type::String,msg::String)
   ccall((:mju_writeLog,libmujoco),Cvoid,(Cstring,Cstring),_type,msg)
end

#---------------------- Utility functions: basic math ----------------------------------

"""set vector to zero"""
function mju_zero3(res::SVector{3, mjtNum})
   ccall((:mju_zero3,libmujoco),Nothing,(Ptr{mjtNum},),pointer_from_objref(res.data))
end

"""copy vector"""
function mju_copy3(res::SVector{3, mjtNum},data::SVector{3, mjtNum})
   ccall((:mju_copy3,libmujoco),Nothing,(Ptr{mjtNum},Ptr{mjtNum}),pointer_from_objref(res),pointer_from_objref(data))
end

"""scale vector"""
function mju_scl3(res::SVector{3, mjtNum},vec::SVector{3, mjtNum},scl::mjtNum)
   ccall((:mju_scl3,libmujoco),Nothing,(Ptr{mjtNum},Ptr{mjtNum},mjtNum),res,vec,scl)
end

"""add vectors"""
function mju_add3(res::SVector{3, mjtNum},vec1::SVector{3, mjtNum},vec2::SVector{3, mjtNum})
   ccall((:mju_add3,libmujoco),Nothing,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum}),res,vec1,vec2)
end

"""subtract vectors"""
function mju_sub3(res::SVector{3, mjtNum},vec1::SVector{3, mjtNum},vec2::SVector{3, mjtNum})
   ccall((:mju_sub3,libmujoco),Nothing,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum}),res,vec1,vec2)
end

"""add to vector"""
function mju_addTo3(res::SVector{3, mjtNum},vec::SVector{3, mjtNum})
   ccall((:mju_addTo3,libmujoco),Nothing,(Ptr{mjtNum},Ptr{mjtNum}),res,vec)
end

"""Set res = res - vec."""
function mju_subFrom3(res::SVector{3, mjtNum},vec::SVector{3, mjtNum})
   ccall((:mju_subFrom3,libmujoco),Nothing,(Ptr{mjtNum},Ptr{mjtNum}),res,vec)
end

"""add scaled to vector"""
function mju_addToScl3(res::SVector{3, mjtNum},vec::SVector{3, mjtNum},scl::mjtNum)
   ccall((:mju_addToScl3,libmujoco),Nothing,(Ptr{mjtNum},Ptr{mjtNum},mjtNum),res,vec,scl)
end

"""res = vec1 + scl*vec2"""
function mju_addScl3(res::SVector{3, mjtNum},vec1::SVector{3, mjtNum},vec2::SVector{3, mjtNum},scl::mjtNum)
   ccall((:mju_addScl3,libmujoco),Nothing,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},mjtNum),res,vec1,vec2,scl)
end

"""normalize vector, return length before normalization"""
function mju_normalize3(res::SVector{3, mjtNum})
   ccall((:mju_normalize3,libmujoco),mjtNum,(Ptr{mjtNum},),res)
end

"""compute vector length (without normalizing)"""
function mju_norm3(vec::SVector{3, mjtNum})
   ccall((:mju_norm3,libmujoco),mjtNum,(Ptr{mjtNum},),vec)
end

"""vector dot-product"""
function mju_dot3(vec1::SVector{3, mjtNum},vec2::SVector{3, mjtNum})
   ccall((:mju_dot3,libmujoco),mjtNum,(Ptr{mjtNum},Ptr{mjtNum}),vec1,vec2)
end

"""Cartesian distance between 3D vectors"""
function mju_dist3(pos1::SVector{3, mjtNum},pos2::SVector{3, mjtNum})
   ccall((:mju_dist3,libmujoco),mjtNum,(Ptr{mjtNum},Ptr{mjtNum}),pos1,pos2)
end

"""multiply vector by 3D rotation matrix"""
function mju_rotVecMat(res::SVector{3, mjtNum},vec::SVector{3, mjtNum},mat::SVector{9, mjtNum})
   ccall((:mju_rotVecMat,libmujoco),Nothing,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum}),res,vec,mat)
end

"""multiply vector by transposed 3D rotation matrix"""
function mju_rotVecMatT(res::SVector{3, mjtNum},vec::SVector{3, mjtNum},mat::SVector{9, mjtNum})
   ccall((:mju_rotVecMatT,libmujoco),Nothing,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum}),res,vec,mat)
end

"""vector cross-product, 3D"""
function mju_cross(res::SVector{3, mjtNum},a::SVector{3, mjtNum},b::SVector{3, mjtNum})
   ccall((:mju_cross,libmujoco),Nothing,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum}),res,a,b)
end

"""set vector to zero"""
function mju_zero4(res::SVector{4, mjtNum})
   ccall((:mju_zero4,libmujoco),Nothing,(Ptr{mjtNum},),res)
end

"""set unit quaterion"""
function mju_unit4(res::SVector{4, mjtNum})
   ccall((:mju_unit4,libmujoco),Nothing,(Ptr{mjtNum},),res)
end

"""copy vector"""
function mju_copy4(res::SVector{4, mjtNum},data::SVector{4, mjtNum})
   ccall((:mju_copy4,libmujoco),Nothing,(Ptr{mjtNum},Ptr{mjtNum}),res,data)
end

"""normalize vector, return length before normalization"""
function mju_normalize4(res::SVector{4, mjtNum})
   ccall((:mju_normalize4,libmujoco),mjtNum,(Ptr{mjtNum},),res)
end

"""set vector to zero"""
function mju_zero(res::PV{mjtNum},n::Integer)
   ccall((:mju_zero,libmujoco),Cvoid,(Ptr{mjtNum},Cint),res,n)
end

"""copy vector"""
function mju_copy(res::PV{mjtNum},data::PV{mjtNum},n::Integer)
   ccall((:mju_copy,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},Cint),res,data,n)
end

"""Return sum(vec)."""
function mju_sum(vec::PV{mjtNum},n::Integer)
   ccall((:mju_sum,libmujoco),mjtNum,(Ptr{mjtNum},Cint),vec,n)
end

"""Return L1 norm: sum(abs(vec))."""
function mju_L1(vec::PV{mjtNum},n::Integer)
   ccall((:mju_L1,libmujoco),mjtNum,(Ptr{mjtNum},Cint),vec,n)
end

"""scale vector"""
function mju_scl(res::PV{mjtNum},vec::PV{mjtNum},scl::mjtNum,n::Integer)
   ccall((:mju_scl,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},mjtNum,Cint),res,vec,scl,n)
end

"""add vectors"""
function mju_add(res::PV{mjtNum},vec1::PV{mjtNum},vec2::PV{mjtNum},n::Integer)
   ccall((:mju_add,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint),res,vec1,vec2,n)
end

"""subtract vectors"""
function mju_sub(res::PV{mjtNum},vec1::PV{mjtNum},vec2::PV{mjtNum},n::Integer)
   ccall((:mju_sub,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint),res,vec1,vec2,n)
end

"""add to vector"""
function mju_addTo(res::PV{mjtNum},vec::PV{mjtNum},n::Integer)
   ccall((:mju_addTo,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},Cint),res,vec,n)
end

"""Set res = res - vec."""
function mju_subFrom(res::PV{mjtNum},vec::PV{mjtNum},n::Integer)
   ccall((:mju_subFrom,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},Cint),res,vec,n)
end

"""add scaled to vector"""
function mju_addToScl(res::PV{mjtNum},vec::PV{mjtNum},scl::mjtNum,n::Integer)
   ccall((:mju_addToScl,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},mjtNum,Cint),res,vec,scl,n)
end

"""res = vec1 + scl*vec2"""
function mju_addScl(res::PV{mjtNum},vec1::PV{mjtNum},vec2::PV{mjtNum},scl::mjtNum,n::Integer)
   ccall((:mju_addScl,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},mjtNum,Cint),res,vec1,vec2,scl,n)
end

"""normalize vector, return length before normalization"""
function mju_normalize(res::PV{mjtNum},n::Integer)
   ccall((:mju_normalize,libmujoco),mjtNum,(Ptr{mjtNum},Cint),res,n)
end

"""compute vector length (without normalizing)"""
function mju_norm(res::PV{mjtNum},n::Integer)
   ccall((:mju_norm,libmujoco),mjtNum,(Ptr{mjtNum},Cint),res,n)
end

"""vector dot-product"""
function mju_dot(vec1::PV{mjtNum},vec2::PV{mjtNum},n::Integer)
   ccall((:mju_dot,libmujoco),mjtNum,(Ptr{mjtNum},Ptr{mjtNum},Cint),vec1,vec2,n)
end

"""multiply matrix and vector"""
function mju_mulMatVec(res::PV{mjtNum},mat::PV{mjtNum},vec::PV{mjtNum},nr::Integer,nc::Integer)
   ccall((:mju_mulMatVec,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint,Cint),res,mat,vec,nr,nc)
end

"""multiply transposed matrix and vector"""
function mju_mulMatTVec(res::PV{mjtNum},mat::PV{mjtNum},vec::PV{mjtNum},nr::Integer,nc::Integer)
   ccall((:mju_mulMatTVec,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint,Cint),res,mat,vec,nr,nc)
end

"""transpose matrix"""
function mju_transpose(res::PV{mjtNum},mat::PV{mjtNum},r::Integer,c::Integer)
   ccall((:mju_transpose,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},Cint,Cint),res,mat,r,c)
end

"""multiply matrices"""
function mju_mulMatMat(res::PV{mjtNum},mat1::PV{mjtNum},mat2::PV{mjtNum},r1::Integer,c1::Integer,c2::Integer)
   ccall((:mju_mulMatMat,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint,Cint,Cint),res,mat1,mat2,r1,c1,c2)
end

"""multiply matrices, second argument transposed"""
function mju_mulMatMatT(res::PV{mjtNum},mat1::PV{mjtNum},mat2::PV{mjtNum},r1::Integer,c1::Integer,r2::Integer)
   ccall((:mju_mulMatMatT,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint,Cint,Cint),res,mat1,mat2,r1,c1,r2)
end

"""multiply matrices, first argument transposed"""
function mju_mulMatTMat(res::PV{mjtNum},mat1::PV{mjtNum},mat2::PV{mjtNum},r1::Integer,c1::Integer,c2::Integer)
   ccall((:mju_mulMatTMat,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint,Cint,Cint),res,mat1,mat2,r1,c1,c2)
end

"""compute M'*diag*M (diag=NULL: compute M'*M)"""
function mju_sqrMatTD(res::PV{mjtNum},mat::PV{mjtNum},diag::PV{mjtNum},r::Integer,c::Integer)
   ccall((:mju_sqrMatTD,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint,Cint),res,mat,diag,r,c)
end

"""coordinate transform of 6D motion or force vector in rotation:translation format
rotnew2old is 3-by-3, NULL means no rotation; flg_force specifies force or motion type"""
function mju_transformSpatial(res::SVector{6, mjtNum},vec::SVector{6, mjtNum},flg_force::Integer,newpos::SVector{3, mjtNum},oldpos::SVector{3, mjtNum},rotnew2old::SVector{9, mjtNum})
   ccall((:mju_transformSpatial,libmujoco),Cvoid,(SVector{6, mjtNum},SVector{6, mjtNum},Cint,SVector{3, mjtNum},SVector{3, mjtNum},SVector{9, mjtNum}),res,vec,flg_force,newpos,oldpos,rotnew2old)
end

#---------------------- Sparse math ----------------------------------------------------

#=
"""Return dot-product of vec1 and vec2, where vec1 is sparse."""
function mju_dotSparse(vec1::PV{mjtNum},vec2::PV{mjtNum},nnz1::Integer,ind1::PV{Cint})
ccall((:mju_dotSparse,libmujoco),mjtNum,(Ptr{mjtNum},Ptr{mjtNum},Cint,Ptr{Cint}),vec1,vec2,nnz1,ind1)
end

"""Return dot-product of vec1 and vec2, where both vectors are sparse."""
function mju_dotSparse2(vec1::PV{mjtNum},vec2::PV{mjtNum},
nnz1::Integer,ind1::PV{Cint},
nnz2::Integer,ind2::PV{Cint})
ccall((:mju_dotSparse2,libmujoco),mjtNum,(Ptr{mjtNum},Ptr{mjtNum},Cint,Ptr{Cint},Cint,Ptr{Cint}),vec1,vec2,nnz1,ind1,nnz2,ind2)
end

"""Convert matrix from dense to sparse format."""
function mju_dense2sparse(res::PV{mjtNum},mat::PV{mjtNum},nr::Integer,nc::Integer,
rownnz::PV{Cint},rowadr::PV{Cint},colind::PV{Cint})
ccall((:mju_dense2sparse,libmujoco),Cvoid,
(Ptr{mjtNum},Ptr{mjtNum},Cint,Cint,Ptr{Integer},Ptr{Integer},Ptr{Integer}),
res,mat,nr,nc,rownnz,rowadr,colind)
end

"""Convert matrix from sparse to dense format."""
function mju_sparse2dense(res::PV{mjtNum},mat::PV{mjtNum},nr::Integer,nc::Integer,
rownnz::PV{Cint},rowadr::PV{Cint},colind::PV{Cint})
ccall((:mju_sparse2dense,libmujoco),Cvoid,
(Ptr{mjtNum},Ptr{mjtNum},Cint,Cint,Ptr{Integer},Ptr{Integer},Ptr{Integer}),
res,mat,nr,nc,rownnz,rowadr,colind)
end

"""Multiply sparse matrix and dense vector:  res = mat * vec."""
function mju_mulMatVecSparse(res::PV{mjtNum},mat::PV{mjtNum},vec::PV{mjtNum},nr::Integer,
rownnz::PV{Cint},rowadr::PV{Cint},colind::PV{Cint})
ccall((:mju_mulMatVecSparse,libmujoco),Cvoid,
(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint,Ptr{Integer},Ptr{Integer},Ptr{Integer}),
res,mat,vec,nr,rownnz,rowadr,colind)
end

"""Compress layout of sparse matrix."""
function mju_compressSparse(mat::PV{mjtNum},nr::Integer,nc::Integer,
rownnz::PV{Cint},rowadr::PV{Cint},colind::PV{Cint})
ccall((:mju_compressSparse,libmujoco),Cvoid,
(Ptr{mjtNum},Cint,Cint,Ptr{Integer},Ptr{Integer},Ptr{Integer}),
mat,nr,nc,rownnz,rowadr,colind)
end

"""Set dst = a*dst + b*src, return nnz of result, modify dst sparsity pattern as needed.
Both vectors are sparse. The required scratch space is 2*n.
"""
function mju_combineSparse(dst::PV{mjtNum},src::PV{mjtNum},n::Integer,a::mjtNum,b::mjtNum,
dst_nnz::Integer,src_nnz::Integer,dst_ind::PV{Cint},src_ind::PV{Cint}, 
scratch::PV{mjtNum},nscratch::Integer)
ccall((:mju_combineSparse,libmujoco),Cint,(Ptr{mjtNum},Ptr{mjtNum},Cint,mjtNum,mjtNum,
Cint,Cint,Ptr{Cint},Ptr{Cint},Ptr{mjtNum},Cint),
dst,src,n,a,b,dst_nnz,src_nnz,dst_ind,src_ind,scratch,nscratch)
end

"""Set res = matT * diag * mat if diag is not NULL, and res = matT * mat otherwise.
The required scratch space is 3*nc. The result has uncompressed layout.
"""
function mju_sqrMatTDSparse(res::PV{mjtNum},mat::PV{mjtNum},matT::PV{mjtNum}, 
diag::PV{mjtNum},nr::Integer,nc::Integer, 
res_rownnz::PV{Cint},res_rowadr::PV{Cint},res_colind::PV{Cint},
rownnz::PV{Cint},rowadr::PV{Cint},colind::PV{Cint},
rownnzT::PV{Cint},rowadrT::PV{Cint},colindT::PV{Cint},
scratch::PV{mjtNum},nscratch::Integer)
ccall((:mju_sqrMatTDSparse,libmujoco),Cvoid,(PV{mjtNum},PV{mjtNum},PV{mjtNum},PV{mjtNum},Cint,Cint,
PV{Cint},PV{Cint},PV{Cint},PV{Cint},PV{Cint},PV{Cint},PV{Cint},PV{Cint},PV{Cint},PV{mjtNum},PV{Cint}),
res,mat,matT,diag,nr,nc,res_rownnz,res_rowadr,res_colind,rownnz,rowadr,colind,rownnzT,rowadrT,colindT,scratch,nscratch)
end

"""Transpose sparse matrix."""
function mju_transposeSparse(res::PV{mjtNum},mat::PV{mjtNum},nr::Integer,nc::Integer,
res_rownnz::PV{Cint},res_rowadr::PV{Cint},res_colind::PV{Cint},
rownnz::PV{Cint},rowadr::PV{Cint},colind::PV{Cint})
ccall((:mju_transposeSparse,libmujoco),Cvoid,(PV{mjtNum},PV{mjtNum},Cint,Cint,
PV{Cint},PV{Cint},PV{Cint},PV{Cint},PV{Cint},PV{Cint}),
res,mat,nr,nc,res_rownnz,res_rowadr,res_colind,rownnz,rowadr,colind)
end

######## removed in 2.0
=#



#---------------------- Utility functions: quaternions ---------------------------------

"""rotate vector by quaternion"""
function mju_rotVecQuat(res::Vector{mjtNum},vec::Vector{mjtNum},quat::Vector{mjtNum})
   @assert length(res) >= 3
   @assert length(vec) >= 3
   @assert length(quat2) >= 4
   ccall((:mju_rotVecQuat,libmujoco),Cvoid,(Vector{mjtNum},Vector{mjtNum},Vector{mjtNum}),res,vec,quat)
end

"""Conjugate quaternion, corresponding to opposite rotation."""
function mju_negQuat(res::Vector{mjtNum},quat::Vector{mjtNum})
   @assert length(res) >= 4
   @assert length(quat) >= 4
   ccall((:mju_negQuat,libmujoco),Cvoid,(Vector{mjtNum},Vector{mjtNum}),res,quat)
end

"""muiltiply quaternions"""
function mju_mulQuat(res::Vector{mjtNum},quat1::Vector{mjtNum},quat2::Vector{mjtNum})
   @assert length(res) >= 4
   @assert length(quat1) >= 4
   @assert length(quat2) >= 4
   ccall((:mju_mulQuat,libmujoco),Cvoid,(Vector{mjtNum},Vector{mjtNum},Vector{mjtNum}),res,quat1,quat2)
end

"""muiltiply quaternion and axis"""
function mju_mulQuatAxis(res::Vector{mjtNum},quat::Vector{mjtNum},axis::Vector{mjtNum})
   @assert length(res) >= 4
   @assert length(quat) >= 4
   @assert length(axis) >= 3
   ccall((:mju_mulQuatAxis,libmujoco),Cvoid,(Vector{mjtNum},Vector{mjtNum},Vector{mjtNum}),res,quat,axis)
end

"""convert axisAngle to quaternion"""
function mju_axisAngle2Quat(res::Vector{mjtNum},axis::Vector{mjtNum},angle::mjtNum)
   @assert length(res) >= 4
   @assert length(axis) >= 3
   ccall((:mju_axisAngle2Quat,libmujoco),Cvoid,(Vector{mjtNum},Vector{mjtNum},mjtNum),res,axis,angle)
end

"""convert quaternion (corresponding to orientation difference) to 3D velocity"""
function mju_quat2Vel(res::Vector{mjtNum},quat::Vector{mjtNum},dt::mjtNum)
   @assert length(res) >= 3
   @assert length(quat) >= 4
   ccall((:mju_quat2Vel,libmujoco),Cvoid,(Vector{mjtNum},Vector{mjtNum},mjtNum),res,quat,dt)
end

"""Subtract quaternions, express as 3D velocity: qb*quat(res) = qa."""
function mju_subQuat(res::Vector{mjtNum},qa::Vector{mjtNum},qb::mjtNum)
   @assert length(res) >= 3
   @assert length(qa) >= 4
   @assert length(qb) >= 4
   ccall((:mju_subQuat,libmujoco),Cvoid,(Vector{mjtNum},Vector{mjtNum},Vector{mjtNum}),res,qa,qb)
end

"""convert quaternion to 3D rotation matrix"""
function mju_quat2Mat(res::Vector{mjtNum},quat::Vector{mjtNum})
   @assert length(res) >= 9
   @assert length(quat) >= 4
   ccall((:mju_quat2Mat,libmujoco),Cvoid,(Vector{mjtNum},Vector{mjtNum}),res,quat)
end

"""convert 3D rotation matrix to quaterion"""
function mju_mat2Quat(quat::Vector{mjtNum},mat::Vector{mjtNum})
   @assert length(quat) >= 4
   @assert length(mat) >= 9
   ccall((:mju_mat2Quat,libmujoco),Cvoid,(Vector{mjtNum},Vector{mjtNum}),quat,mat)
end

"""time-derivative of quaternion, given 3D rotational velocity"""
function mju_derivQuat(res::Vector{mjtNum},quat::Vector{mjtNum},vel::Vector{mjtNum})
   @assert length(res) >= 4
   @assert length(quat) >= 4
   @assert length(vel) >= 3
   ccall((:mju_derivQuat,libmujoco),Cvoid,(Vector{mjtNum},Vector{mjtNum},Vector{mjtNum}),res,quat,vel)
end

"""integrate quaterion given 3D angular velocity"""
function mju_quatIntegrate(quat::Vector{mjtNum},vel::Vector{mjtNum},scale::mjtNum)
   @assert length(quat) >= 4
   @assert length(vel) >= 3
   ccall((:mju_quatIntegrate,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},mjtNum),quat,vel,scale)
end

"""compute quaternion performing rotation from z-axis to given vector"""
function mju_quatZ2Vec(quat::Vector{mjtNum},vec::Vector{mjtNum})
   @assert length(quat) >= 4
   @assert length(vec) >= 3
   ccall((:mju_quatZ2Vec,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum}),quat,vec)
end

#---------------------- Utility functions: poses (pos, quat) ---------------------------

"""multiply two poses"""
function mju_mulPose(posres::Vector{mjtNum},quatres::Vector{mjtNum},pos1::Vector{mjtNum},quat1::Vector{mjtNum},pos2::Vector{mjtNum},quat2::Vector{mjtNum})
   @assert length(posres) >= 3
   @assert length(quatres) >= 4
   @assert length(pos1) >= 3
   @assert length(quat1) >= 4
   @assert length(pos2) >= 3
   @assert length(quat2) >= 4
   ccall((:mju_mulPose,libmujoco),Cvoid,(Vector{mjtNum},Vector{mjtNum},Vector{mjtNum},Vector{mjtNum},Vector{mjtNum},Vector{mjtNum}),posres,quatres,pos1,quat1,pos2,quat2)
end

"""Conjugate pose, corresponding to the opposite spatial transformation."""
function mju_negPose(posres::Vector{mjtNum},quatres::Vector{mjtNum},pos::Vector{mjtNum},quat::Vector{mjtNum})
   @assert length(posres) >= 3
   @assert length(quatres) >= 4
   @assert length(pos) >= 3
   @assert length(quat) >= 4
   ccall((:mju_negPose,libmujoco),Cvoid,(Vector{mjtNum},Vector{mjtNum},Vector{mjtNum},Vector{mjtNum}),posres,quatres,pos,quat)
end

"""transform vector by pose"""
function mju_trnVecPose(res::Vector{mjtNum},pos::Vector{mjtNum},quat::Vector{mjtNum},vec::Vector{mjtNum})
   @assert length(res) >= 3
   @assert length(pos) >= 3
   @assert length(quat) >= 4
   @assert length(vec) >= 3
   ccall((:mju_trnVecPose,libmujoco),Cvoid,(Vector{mjtNum},Vector{mjtNum},Vector{mjtNum},Vector{mjtNum}),res,pos,quat,vec)
end

#---------------------- Utility functions: matrix decomposition ------------------------

"""Cholesky decomposition: mat = L*L'; return rank."""
function mju_cholFactor(mat::PV{mjtNum},n::Integer,mindiag::mjtNum)
   ccall((:mju_cholFactor,libmujoco),Cint,(Ptr{mjtNum},Cint,mjtNum),mat,n,mindiag)
end

"""Cholesky backsubstitution: phase&i enables forward(i=1), backward(i=2) pass"""
function mju_cholBacksub(res::PV{mjtNum},mat::PV{mjtNum},vec::PV{mjtNum},n::Integer,nvec::Integer,phase::Integer)
   ccall((:mju_cholBacksub,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint,Cint,Cint),res,mat,vec,n,nvec,phase)
end

"""Solve mat * res = vec, where mat is Cholesky-factorized"""
function mju_cholSolve(res::PV{mjtNum},mat::PV{mjtNum},vec::PV{mjtNum},n::Integer)
   ccall((:mju_cholSolve,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint),res,mat,vec,n)
end

"""Cholesky rank-one update: L*L' +/- x*x'; return rank."""
function mju_cholUpdate(res::PV{mjtNum},mat::PV{mjtNum},vec::PV{mjtNum},n::Integer)
   ccall((:mju_cholUpdate,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},Cint,Cint),mat,x,n,flg_plus)
end

#=
"""Sparse reverse-order Cholesky decomposition: mat = L'*L; return 'rank'.
mat must have uncompressed layout; rownnz is modified to end at diagonal.
The required scratch space is 2*n.
"""
function mju_cholFactorSparse(mat::PV{mjtNum},n::Integer, 
rownnz::PV{Cint},rowadr::PV{Cint},colind::PV{Cint},
scratch::PV{mjtNum},nscratch::Integer)
ccall((:mju_cholFactorSparse,libmujoco),Cint,
(Ptr{mjtNum},Cint,Ptr{Cint},Ptr{Cint},Ptr{Cint},Ptr{mjtNum},Cint),
mat,n,rownnz,rowadr,colind,scratch,nscratch)
end

"""Solve mat * res = vec, where mat is sparse reverse-order Cholesky factorized."""
function mju_cholSolveSparse(res::PV{mjtNum},mat::PV{mjtNum},vec::PV{mjtNum},n::Integer,
rownnz::PV{Cint},rowadr::PV{Cint},colind::PV{Cint})
ccall((:mju_cholSolveSparse,libmujoco),Cvoid,
(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint,Ptr{Cint},Ptr{Cint},Ptr{Cint}),
res,mat,vec,n,rownnz,rowadr,colind)
end

"""Sparse reverse-order Cholesky rank-one update: L'*L +/- x*x'; return rank.
The vector x is sparse; changes in sparsity pattern of mat are not allowed.
The required scratch space is 2*n.
"""
function mju_cholUpdateSparse(mat::PV{mjtNum},x::PV{mjtNum},n::Integer,flg_plus::Integer,
rownnz::PV{Cint},rowadr::PV{Cint},colind::PV{Cint},x_nnz::Integer,x_ind::PV{Cint},
scratch::PV{mjtNum},nscratch::Integer)
ccall((:mju_cholUpdateSparse,libmujoco),Cint,
(Ptr{mjtNum},Ptr{mjtNum},Cint,Cint,Ptr{Cint},Ptr{Cint},Ptr{Cint},Cint,Ptr{Cint},Ptr{mjtNum},Cint),
mat,vec,n,flg_plus,rownnz,rowadr,colind,x_nnz,x_ind,scratch,nscratch)
end

# not in 2.0
=#

"""eigenvalue decomposition of symmetric 3x3 matrix"""
function mju_eig3(eigval::PV{mjtNum},eigvec::PV{mjtNum},quat::PV{mjtNum},mat::PV{mjtNum})
   ccall((:mju_eig3,libmujoco),Cint,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum}),eigval,eigvec,quat,mat)
end

#---------------------- Utility functions: miscellaneous -------------------------------

#=
"""muscle FVL curve: prm = (lminrel, lmaxrel, widthrel, vmaxrel, fmax, fvsat)"""
function mju_muscleFVL(len::mjtNum,vel::mjtNum,lmin::mjtNum,lmax::mjtNum,prm::PV{mjtNum})
ccall((:mju_muscleFVL,libmujoco),mjtNum,(mjtNum,mjtNum,mjtNum,mjtNum,Ptr{mjtNum}),len,vel,lmin,lmax,prm)
end

"""muscle passive force: prm = (lminrel, lmaxrel, fpassive)"""
function mju_musclePassive(len::mjtNum,lmin::mjtNum,lmax::mjtNum,prm::PV{mjtNum})
ccall((:mju_musclePassive,libmujoco),mjtNum,(mjtNum,mjtNum,mjtNum,Ptr{mjtNum}),len,lmin,lmax,prm)
end

"""pneumatic cylinder dynamics"""
function mju_pneumatic(len::mjtNum,len0::mjtNum,vel::mjtNum,prm::PV{mjtNum},act::mjtNum,ctrl::mjtNum,timestep::mjtNum,jac::PV{mjtNum})
ccall((:mju_pneumatic,libmujoco),mjtNum,(mjtNum,mjtNum,mjtNum,Ptr{mjtNum},mjtNum,mjtNum,mjtNum,Ptr{mjtNum}),len,len0,vel,prm,act,ctrl,timestep,jac)
end
=#

"""Muscle active force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax)."""
function mju_muscleGain(len::mjtNum,vel::mjtNum,lengthrange::PV{mjtNum},acc0::mjtNum,prm::PV{mjtNum})
   @assert length(lengthrange) >= 2
   @assert length(prm) >= 9
   ccall((:mju_muscleGain,libmujoco),mjtNum,(mjtNum,mjtNum,Ptr{mjtNum},mjtNum,Ptr{mjtNum}),len,vel,lengthrange,acc0,prm)
end

"""Muscle passive force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax)."""
function mju_muscleBias(len::mjtNum,lengthrange::PV{mjtNum},acc0::mjtNum,prm::PV{mjtNum})
   @assert length(lengthrange) >= 2
   @assert length(prm) >= 9
   ccall((:mju_muscleBias,libmujoco),mjtNum,(mjtNum,Ptr{mjtNum},mjtNum,Ptr{mjtNum}),len,lengthrange,acc0,prm)
end

"""Muscle activation dynamics, prm = (tau_act, tau_deact)."""
function mju_muscleDynamics(ctrl::mjtNum,act::mjtNum,prm::PV{mjtNum})
   @assert length(prm) >= 2
   ccall((:mju_muscleDynamics,libmujoco),mjtNum,(mjtNum,mjtNum,Ptr{mjtNum}),ctrl,act,prm)
end

"""convert contact force to pyramid representation"""
function mju_encodePyramid(pyramid::PV{mjtNum},force::PV{mjtNum},mu::PV{mjtNum},dim::Integer)
   ccall((:mju_encodePyramid,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint),pyramid,force,mu,dim)
end

"""convert pyramid representation to contact force"""
function mju_decodePyramid(force::PV{mjtNum},pyramid::PV{mjtNum},mu::PV{mjtNum},dim::Integer)
   ccall((:mju_decodePyramid,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{mjtNum},Ptr{mjtNum},Cint),force,pyramid,mu,dim)
end

"""integrate spring-damper analytically, return pos(dt)"""
function mju_springDamper(pos0::mjtNum,vel0::mjtNum,Kp::mjtNum,Kv::mjtNum,dt::mjtNum)
   ccall((:mju_springDamper,libmujoco),mjtNum,(mjtNum,mjtNum,mjtNum,mjtNum,mjtNum),pos0,vel0,Kp,Kv,dt)
end

"""min function, single evaluation of a and b"""
function mju_min(a::mjtNum,b::mjtNum)
   ccall((:mju_min,libmujoco),mjtNum,(mjtNum,mjtNum),a,b)
end

"""max function, single evaluation of a and b"""
function mju_max(a::mjtNum,b::mjtNum)
   ccall((:mju_max,libmujoco),mjtNum,(mjtNum,mjtNum),a,b)
end

"""sign function"""
function mju_sign(x::mjtNum)
   ccall((:mju_sign,libmujoco),mjtNum,(mjtNum,),x)
end

"""round to nearest integer"""
function mju_round(x::mjtNum)
   ccall((:mju_round,libmujoco),Cint,(mjtNum,),x)
end

"""convert type id (mjtObj) to type name"""
function mju_type2Str(_type::Integer)
   ccall((:mju_type2Str,libmujoco),Cstring,(Cint,),_type)
end

"""convert type name to type id (mjtObj)"""
function mju_str2Type(str::String)
   ccall((:mju_str2Type,libmujoco),Cint,(Cstring,),str)
end

"""warning text"""
function mju_warningText(warning::Integer,info::Integer)
   ccall((:mju_warningText,libmujoco),Cstring,(Cint,Cint),warning,info)
end

"""return 1 if nan or abs(x)>mjMAXVAL, 0 otherwise"""
function mju_isBad(x::mjtNum)
   ccall((:mju_isBad,libmujoco),Cint,(mjtNum,),x)
end

"""return 1 if all elements are 0"""
function mju_isZero(vec::PV{mjtNum},n::Integer)
   ccall((:mju_isZero,libmujoco),Cint,(Ptr{mjtNum},Cint),vec,n)
end

"""standard normal random number generator (optional second number)"""
function mju_standardNormal(num2::PV{mjtNum})
   ccall((:mju_standardNormal,libmujoco),mjtNum,(Ptr{mjtNum},),num2)
end

"""convert from float to mjtNum"""
function mju_f2n(res::PV{mjtNum},vec::Ptr{Cfloat},n::Integer)
   ccall((:mju_f2n,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{Cfloat},Cint),res,vec,n)
end

"""convert from mjtNum to float"""
function mju_n2f(res::Ptr{Cfloat},vec::PV{mjtNum},n::Integer)
   ccall((:mju_n2f,libmujoco),Cvoid,(Ptr{Cfloat},Ptr{mjtNum},Cint),res,vec,n)
end

"""convert from double to mjtNum"""
function mju_d2n(res::PV{mjtNum},vec::Ptr{Cdouble},n::Integer)
   ccall((:mju_d2n,libmujoco),Cvoid,(Ptr{mjtNum},Ptr{Cdouble},Cint),res,vec,n)
end

"""convert from mjtNum to double"""
function mju_n2d(res::Ptr{Cdouble},vec::PV{mjtNum},n::Integer)
   ccall((:mju_n2d,libmujoco),Cvoid,(Ptr{Cdouble},Ptr{mjtNum},Cint),res,vec,n)
end

"""insertion sort, increasing order"""
function mju_insertionSort(list::PV{mjtNum},n::Integer)
   ccall((:mju_insertionSort,libmujoco),Cvoid,(Ptr{mjtNum},Cint),list,n)
end

"""Generate Halton sequence."""
function mju_Halton(index::Integer,base::Integer)
   ccall((:mju_Halton,libmujoco),mjtNum,(Cint,Cint),index,base)
end

"""Call strncpy, then set dst[n-1] = 0."""
function mju_strncpy(dst::String,src::String,n::Integer)
   ccall((:mju_strncpy,libmujoco),String,(Cstring,Cstring,Cint),dst,src,n)
end

#---------------------- UI framework ---------------------------------------------------

"""Get builtin UI theme spacing (ind: 0-1)."""
function mjui_themeSpacing(ind::Integer)
   ccall((:mjui_themeSpacing,libmujoco),mjuiThemeSpacing,(Cint,),ind)
end

"""Get builtin UI theme color (ind: 0-3)."""
function mjui_themeColor(ind::Integer)
   ccall((:mjui_themeColor,libmujoco),mjuiThemeColor,(Cint,),ind)
end

"""Add definitions to UI."""
function mjui_add(ui::PV{mjUI},def::PV{mjuiDef})
   ccall((:mjui_add,libmujoco),Cvoid,(Ptr{mjUI},Ptr{mjuiDef}),ui,def)
end

"""Compute UI sizes."""
function mjui_resize(ui::PV{mjUI},con::PV{mjrContext})
   ccall((:mjui_resize,libmujoco),Cvoid,(Ptr{mjUI},Ptr{mjrContext}),ui,con)
end

"""Update specific section/item; -1: update all."""
function mjui_update(section::Integer,item::Integer,ui::PV{mjUI},state::PV{mjuiState},con::PV{mjrContext})
   ccall((:mjui_update,libmujoco),Cvoid,(Cint,Cint,Ptr{mjUI},Ptr{mjuiState},Ptr{mjrContext}),section,item,ui,state,con)
end

"""Handle UI event, return pointer to changed item, NULL if no change."""
function mjui_event(ui::PV{mjUI},state::PV{mjuiState},con::PV{mjrContext})
   ccall((:mjui_event,libmujoco),Ptr{mjuiItem},(Ptr{mjUI},Ptr{mjuiState},Ptr{mjrContext}),ui,state,con)
end

"""Copy UI image to current buffer."""
function mjui_render(ui::PV{mjUI},state::PV{mjuiState},con::PV{mjrContext})
   ccall((:mjui_render,libmujoco),Cvoid,(Ptr{mjUI},Ptr{mjuiState},Ptr{mjrContext}),ui,state,con)
end


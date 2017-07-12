
const mjMINVAL = 1.0e-14
const mjPI = 3.141592653589793
const mjMAXVAL = 1.0e10
const mjMINMU = 1.0e-5
const mjMINIMP = 0.0001
const mjMAXIMP = 0.9999
const mjMAXCONPAIR = 16
const mjNEQDATA = 7
const mjNDYN = 3
const mjNGAIN = 3
const mjNBIAS = 3
const mjNREF = 2
const mjNIMP = 3
const mjNTRACE = 200
const mjNGROUP = 5
const mjMAXOVERLAY = 500


@enum mjtDisableBit mjDSBL_CONSTRAINT = (UInt32)(1) mjDSBL_EQUALITY = (UInt32)(2) mjDSBL_FRICTIONLOSS = (UInt32)(4) mjDSBL_LIMIT = (UInt32)(8) mjDSBL_CONTACT = (UInt32)(16) mjDSBL_MOCAP = (UInt32)(32) mjDSBL_SENSOR = (UInt32)(64) mjDSBL_PASSIVE = (UInt32)(128) mjDSBL_GRAVITY = (UInt32)(256) mjDSBL_CLAMPCTRL = (UInt32)(512) mjDSBL_WARMSTART = (UInt32)(1024) mjDSBL_FILTERPARENT = (UInt32)(2048) mjDSBL_ACTUATION = (UInt32)(4096) mjDSBL_BROADPHASE = (UInt32)(8192) mjDSBL_REFSAFE = (UInt32)(16384) mjNDISABLE = (UInt32)(15)

@enum mjtEnableBit mjENBL_OVERRIDE = (UInt32)(1) mjENBL_BOXCONVEX = (UInt32)(2) mjENBL_ENERGY = (UInt32)(4) mjENBL_FWDINV = (UInt32)(8) mjENBL_SENSORNOISE = (UInt32)(16) mjNENABLE = (UInt32)(5) 

@enum mjtJoint mjJNT_FREE = (UInt32)(0) mjJNT_BALL = (UInt32)(1) mjJNT_SLIDE = (UInt32)(2) mjJNT_HINGE = (UInt32)(3) 

@enum mjtGeom mjGEOM_PLANE = (UInt32)(0) mjGEOM_HFIELD = (UInt32)(1) mjGEOM_SPHERE = (UInt32)(2) mjGEOM_CAPSULE = (UInt32)(3) mjGEOM_ELLIPSOID = (UInt32)(4) mjGEOM_CYLINDER = (UInt32)(5) mjGEOM_BOX = (UInt32)(6) mjGEOM_MESH = (UInt32)(7) mjNGEOMTYPES = (UInt32)(8) mjGEOM_ARROW = (UInt32)(100) mjGEOM_ARROW1 = (UInt32)(101) mjGEOM_ARROW2 = (UInt32)(102) mjGEOM_LABEL = (UInt32)(103) mjGEOM_NONE = (UInt32)(1001)  

@enum mjtCamLight  mjCAMLIGHT_FIXED = (UInt32)(0) mjCAMLIGHT_TRACK = (UInt32)(1) mjCAMLIGHT_TRACKCOM = (UInt32)(2) mjCAMLIGHT_TARGETBODY = (UInt32)(3) mjCAMLIGHT_TARGETBODYCOM = (UInt32)(4) 

@enum mjtTexture mjTEXTURE_2D = (UInt32)(0) mjTEXTURE_CUBE = (UInt32)(1) mjTEXTURE_SKYBOX = (UInt32)(2) 

@enum mjtIntegrator mjINT_EULER = (UInt32)(0) mjINT_RK4 = (UInt32)(1) 

@enum mjtCollision mjCOL_ALL = (UInt32)(0) mjCOL_PAIR = (UInt32)(1) mjCOL_DYNAMIC = (UInt32)(2) 

@enum mjtSolver mjSOL_CG = (UInt32)(0) mjSOL_CG_ELLIPTIC = (UInt32)(1) mjSOL_PCG_SPARSE = (UInt32)(2) mjSOL_PCG = (UInt32)(3) mjSOL_PGS = (UInt32)(4) mjSOL_ELLIPTIC = (UInt32)(5) 

@enum mjtImp mjIMP_CONSTANT = (UInt32)(0) mjIMP_SIGMOID = (UInt32)(1) mjIMP_LINEAR = (UInt32)(2) mjIMP_USER = (UInt32)(3) 

@enum mjtRef mjREF_SPRING = (UInt32)(0) mjREF_USER = (UInt32)(1) 

@enum mjtEq mjEQ_CONNECT = (UInt32)(0) mjEQ_WELD = (UInt32)(1) mjEQ_JOINT = (UInt32)(2) mjEQ_TENDON = (UInt32)(3) mjEQ_DISTANCE = (UInt32)(4) 

@enum mjtWrap mjWRAP_NONE = (UInt32)(0) mjWRAP_JOINT = (UInt32)(1) mjWRAP_PULLEY = (UInt32)(2) mjWRAP_SITE = (UInt32)(3) mjWRAP_SPHERE = (UInt32)(4) mjWRAP_CYLINDER = (UInt32)(5) 

@enum mjtTrn mjTRN_JOINT = (UInt32)(0) mjTRN_JOINTINPARENT = (UInt32)(1) mjTRN_SLIDERCRANK = (UInt32)(2) mjTRN_TENDON = (UInt32)(3) mjTRN_SITE = (UInt32)(4) mjTRN_UNDEFINED = (UInt32)(1000) 

@enum mjtDyn mjDYN_NONE = (UInt32)(0) mjDYN_INTEGRATOR = (UInt32)(1) mjDYN_FILTER = (UInt32)(2) mjDYN_USER = (UInt32)(3) 

@enum mjtGain mjGAIN_FIXED = (UInt32)(0) mjGAIN_USER = (UInt32)(1) 

@enum mjtBias mjBIAS_NONE = (UInt32)(0) mjBIAS_AFFINE = (UInt32)(1) mjBIAS_USER = (UInt32)(2) 

@enum mjtObj mjOBJ_UNKNOWN = (UInt32)(0) mjOBJ_BODY = (UInt32)(1) mjOBJ_XBODY = (UInt32)(2) mjOBJ_JOINT = (UInt32)(3) mjOBJ_DOF = (UInt32)(4) mjOBJ_GEOM = (UInt32)(5) mjOBJ_SITE = (UInt32)(6) mjOBJ_CAMERA = (UInt32)(7) mjOBJ_LIGHT = (UInt32)(8) mjOBJ_MESH = (UInt32)(9) mjOBJ_HFIELD = (UInt32)(10) mjOBJ_TEXTURE = (UInt32)(11) mjOBJ_MATERIAL = (UInt32)(12) mjOBJ_PAIR = (UInt32)(13) mjOBJ_EXCLUDE = (UInt32)(14) mjOBJ_EQUALITY = (UInt32)(15) mjOBJ_TENDON = (UInt32)(16) mjOBJ_ACTUATOR = (UInt32)(17) mjOBJ_SENSOR = (UInt32)(18) mjOBJ_NUMERIC = (UInt32)(19) mjOBJ_TEXT = (UInt32)(20) mjOBJ_TUPLE = (UInt32)(21) mjOBJ_KEY = (UInt32)(22) 

@enum mjtConstraint mjCNSTR_EQUALITY = (UInt32)(0) mjCNSTR_FRICTION_DOF = (UInt32)(1) mjCNSTR_FRICTION_TENDON = (UInt32)(2) mjCNSTR_LIMIT_JOINT = (UInt32)(3) mjCNSTR_LIMIT_TENDON = (UInt32)(4) mjCNSTR_CONTACT_FRICTIONLESS = (UInt32)(5) mjCNSTR_CONTACT_PYRAMIDAL = (UInt32)(6) mjCNSTR_CONTACT_ELLIPTIC = (UInt32)(7) 

@enum mjtSensor mjSENS_TOUCH = (UInt32)(0) mjSENS_ACCELEROMETER = (UInt32)(1) mjSENS_VELOCIMETER = (UInt32)(2) mjSENS_GYRO = (UInt32)(3) mjSENS_FORCE = (UInt32)(4) mjSENS_TORQUE = (UInt32)(5) mjSENS_MAGNETOMETER = (UInt32)(6) mjSENS_JOINTPOS = (UInt32)(7) mjSENS_JOINTVEL = (UInt32)(8) mjSENS_TENDONPOS = (UInt32)(9) mjSENS_TENDONVEL = (UInt32)(10) mjSENS_ACTUATORPOS = (UInt32)(11) mjSENS_ACTUATORVEL = (UInt32)(12) mjSENS_ACTUATORFRC = (UInt32)(13) mjSENS_BALLQUAT = (UInt32)(14) mjSENS_BALLANGVEL = (UInt32)(15) mjSENS_FRAMEPOS = (UInt32)(16) mjSENS_FRAMEQUAT = (UInt32)(17) mjSENS_FRAMEXAXIS = (UInt32)(18) mjSENS_FRAMEYAXIS = (UInt32)(19) mjSENS_FRAMEZAXIS = (UInt32)(20) mjSENS_FRAMELINVEL = (UInt32)(21) mjSENS_FRAMEANGVEL = (UInt32)(22) mjSENS_FRAMELINACC = (UInt32)(23) mjSENS_FRAMEANGACC = (UInt32)(24) mjSENS_SUBTREECOM = (UInt32)(25) mjSENS_SUBTREELINVEL = (UInt32)(26) mjSENS_SUBTREEANGMOM = (UInt32)(27) mjSENS_USER = (UInt32)(28)

@enum mjtStage mjSTAGE_NONE = (UInt32)(0) mjSTAGE_POS = (UInt32)(1) mjSTAGE_VEL = (UInt32)(2) mjSTAGE_ACC = (UInt32)(3) 

@enum mjtDataType mjDATATYPE_REAL = (UInt32)(0) mjDATATYPE_AXIS = (UInt32)(1) mjDATATYPE_QUAT = (UInt32)(2)

#struct _mjOption
immutable _mjOption
   timestep::mjtNum
   apirate::mjtNum
   tolerance::mjtNum
   impratio::mjtNum
   gravity::NTuple{3, mjtNum}
   wind::NTuple{3, mjtNum}
   magnetic::NTuple{3, mjtNum}
   density::mjtNum
   viscosity::mjtNum
   o_margin::mjtNum
   o_solref::NTuple{2, mjtNum}
   o_solimp::NTuple{3, mjtNum}
   mpr_tolerance::mjtNum
   mpr_iterations::Cint
   integrator::Cint
   collision::Cint
   impedance::Cint
   reference::Cint
   solver::Cint
   iterations::Cint
   disableflags::Cint
   enableflags::Cint
end

const mjOption = _mjOption

#struct _mjVisual
immutable _mjVisual
   _global::Void
   quality::Void
   headlight::Void
   map::Void
   scale::Void
   rgba::Void
end

const mjVisual = _mjVisual

#struct _mjStatistic
immutable _mjStatistic
   meanmass::mjtNum
   meansize::mjtNum
   extent::mjtNum
   center::NTuple{3, mjtNum}
end

const mjStatistic = _mjStatistic

type _mjModel
   nq::Cint
   nv::Cint
   nu::Cint
   na::Cint
   nbody::Cint
   njnt::Cint
   ngeom::Cint
   nsite::Cint
   ncam::Cint
   nlight::Cint
   nmesh::Cint
   nmeshvert::Cint
   nmeshface::Cint
   nmeshgraph::Cint
   nhfield::Cint
   nhfielddata::Cint
   ntex::Cint
   ntexdata::Cint
   nmat::Cint
   npair::Cint
   nexclude::Cint
   neq::Cint
   ntendon::Cint
   nwrap::Cint
   nsensor::Cint
   nnumeric::Cint
   nnumericdata::Cint
   ntext::Cint
   ntextdata::Cint
   ntuple::Cint
   ntupledata::Cint
   nkey::Cint
   nuser_body::Cint
   nuser_jnt::Cint
   nuser_geom::Cint
   nuser_site::Cint
   nuser_tendon::Cint
   nuser_actuator::Cint
   nuser_sensor::Cint
   nnames::Cint
   nM::Cint
   nemax::Cint
   njmax::Cint
   nconmax::Cint
   nstack::Cint
   nuserdata::Cint
   nmocap::Cint
   nsensordata::Cint
   nbuffer::Cint
   opt::mjOption
   vis::mjVisual
   stat::mjStatistic
   buffer::Ptr{Void}
   qpos0::Ptr{mjtNum}
   qpos_spring::Ptr{mjtNum}
   body_parentid::Ptr{Cint}
   body_rootid::Ptr{Cint}
   body_weldid::Ptr{Cint}
   body_mocapid::Ptr{Cint}
   body_jntnum::Ptr{Cint}
   body_jntadr::Ptr{Cint}
   body_dofnum::Ptr{Cint}
   body_dofadr::Ptr{Cint}
   body_geomnum::Ptr{Cint}
   body_geomadr::Ptr{Cint}
   body_pos::Ptr{mjtNum}
   body_quat::Ptr{mjtNum}
   body_ipos::Ptr{mjtNum}
   body_iquat::Ptr{mjtNum}
   body_mass::Ptr{mjtNum}
   body_subtreemass::Ptr{mjtNum}
   body_inertia::Ptr{mjtNum}
   body_invweight0::Ptr{mjtNum}
   body_user::Ptr{mjtNum}
   jnt_type::Ptr{Cint}
   jnt_qposadr::Ptr{Cint}
   jnt_dofadr::Ptr{Cint}
   jnt_bodyid::Ptr{Cint}
   jnt_limited::Ptr{mjtByte}
   jnt_solref::Ptr{mjtNum}
   jnt_solimp::Ptr{mjtNum}
   jnt_pos::Ptr{mjtNum}
   jnt_axis::Ptr{mjtNum}
   jnt_stiffness::Ptr{mjtNum}
   jnt_range::Ptr{mjtNum}
   jnt_margin::Ptr{mjtNum}
   jnt_user::Ptr{mjtNum}
   dof_bodyid::Ptr{Cint}
   dof_jntid::Ptr{Cint}
   dof_parentid::Ptr{Cint}
   dof_Madr::Ptr{Cint}
   dof_frictional::Ptr{mjtByte}
   dof_solref::Ptr{mjtNum}
   dof_solimp::Ptr{mjtNum}
   dof_frictionloss::Ptr{mjtNum}
   dof_armature::Ptr{mjtNum}
   dof_damping::Ptr{mjtNum}
   dof_invweight0::Ptr{mjtNum}
   geom_type::Ptr{Cint}
   geom_contype::Ptr{Cint}
   geom_conaffinity::Ptr{Cint}
   geom_condim::Ptr{Cint}
   geom_bodyid::Ptr{Cint}
   geom_dataid::Ptr{Cint}
   geom_matid::Ptr{Cint}
   geom_group::Ptr{Cint}
   geom_solmix::Ptr{mjtNum}
   geom_solref::Ptr{mjtNum}
   geom_solimp::Ptr{mjtNum}
   geom_size::Ptr{mjtNum}
   geom_rbound::Ptr{mjtNum}
   geom_pos::Ptr{mjtNum}
   geom_quat::Ptr{mjtNum}
   geom_friction::Ptr{mjtNum}
   geom_margin::Ptr{mjtNum}
   geom_gap::Ptr{mjtNum}
   geom_user::Ptr{mjtNum}
   geom_rgba::Ptr{Cfloat}
   site_type::Ptr{Cint}
   site_bodyid::Ptr{Cint}
   site_matid::Ptr{Cint}
   site_group::Ptr{Cint}
   site_size::Ptr{mjtNum}
   site_pos::Ptr{mjtNum}
   site_quat::Ptr{mjtNum}
   site_user::Ptr{mjtNum}
   site_rgba::Ptr{Cfloat}
   cam_mode::Ptr{Cint}
   cam_bodyid::Ptr{Cint}
   cam_targetbodyid::Ptr{Cint}
   cam_pos::Ptr{mjtNum}
   cam_quat::Ptr{mjtNum}
   cam_poscom0::Ptr{mjtNum}
   cam_pos0::Ptr{mjtNum}
   cam_mat0::Ptr{mjtNum}
   cam_fovy::Ptr{mjtNum}
   cam_ipd::Ptr{mjtNum}
   light_mode::Ptr{Cint}
   light_bodyid::Ptr{Cint}
   light_targetbodyid::Ptr{Cint}
   light_directional::Ptr{mjtByte}
   light_castshadow::Ptr{mjtByte}
   light_active::Ptr{mjtByte}
   light_pos::Ptr{mjtNum}
   light_dir::Ptr{mjtNum}
   light_poscom0::Ptr{mjtNum}
   light_pos0::Ptr{mjtNum}
   light_dir0::Ptr{mjtNum}
   light_attenuation::Ptr{Cfloat}
   light_cutoff::Ptr{Cfloat}
   light_exponent::Ptr{Cfloat}
   light_ambient::Ptr{Cfloat}
   light_diffuse::Ptr{Cfloat}
   light_specular::Ptr{Cfloat}
   mesh_faceadr::Ptr{Cint}
   mesh_facenum::Ptr{Cint}
   mesh_vertadr::Ptr{Cint}
   mesh_vertnum::Ptr{Cint}
   mesh_graphadr::Ptr{Cint}
   mesh_vert::Ptr{Cfloat}
   mesh_normal::Ptr{Cfloat}
   mesh_face::Ptr{Cint}
   mesh_graph::Ptr{Cint}
   hfield_size::Ptr{mjtNum}
   hfield_nrow::Ptr{Cint}
   hfield_ncol::Ptr{Cint}
   hfield_adr::Ptr{Cint}
   hfield_data::Ptr{Cfloat}
   tex_type::Ptr{Cint}
   tex_height::Ptr{Cint}
   tex_width::Ptr{Cint}
   tex_adr::Ptr{Cint}
   tex_rgb::Ptr{mjtByte}
   mat_texid::Ptr{Cint}
   mat_texuniform::Ptr{mjtByte}
   mat_texrepeat::Ptr{Cfloat}
   mat_emission::Ptr{Cfloat}
   mat_specular::Ptr{Cfloat}
   mat_shininess::Ptr{Cfloat}
   mat_reflectance::Ptr{Cfloat}
   mat_rgba::Ptr{Cfloat}
   pair_dim::Ptr{Cint}
   pair_geom1::Ptr{Cint}
   pair_geom2::Ptr{Cint}
   pair_signature::Ptr{Cint}
   pair_solref::Ptr{mjtNum}
   pair_solimp::Ptr{mjtNum}
   pair_margin::Ptr{mjtNum}
   pair_gap::Ptr{mjtNum}
   pair_friction::Ptr{mjtNum}
   exclude_signature::Ptr{Cint}
   eq_type::Ptr{Cint}
   eq_obj1id::Ptr{Cint}
   eq_obj2id::Ptr{Cint}
   eq_active::Ptr{mjtByte}
   eq_solref::Ptr{mjtNum}
   eq_solimp::Ptr{mjtNum}
   eq_data::Ptr{mjtNum}
   tendon_adr::Ptr{Cint}
   tendon_num::Ptr{Cint}
   tendon_matid::Ptr{Cint}
   tendon_limited::Ptr{mjtByte}
   tendon_frictional::Ptr{mjtByte}
   tendon_width::Ptr{mjtNum}
   tendon_solref_lim::Ptr{mjtNum}
   tendon_solimp_lim::Ptr{mjtNum}
   tendon_solref_fri::Ptr{mjtNum}
   tendon_solimp_fri::Ptr{mjtNum}
   tendon_range::Ptr{mjtNum}
   tendon_margin::Ptr{mjtNum}
   tendon_stiffness::Ptr{mjtNum}
   tendon_damping::Ptr{mjtNum}
   tendon_frictionloss::Ptr{mjtNum}
   tendon_lengthspring::Ptr{mjtNum}
   tendon_length0::Ptr{mjtNum}
   tendon_invweight0::Ptr{mjtNum}
   tendon_user::Ptr{mjtNum}
   tendon_rgba::Ptr{Cfloat}
   wrap_type::Ptr{Cint}
   wrap_objid::Ptr{Cint}
   wrap_prm::Ptr{mjtNum}
   actuator_trntype::Ptr{Cint}
   actuator_dyntype::Ptr{Cint}
   actuator_gaintype::Ptr{Cint}
   actuator_biastype::Ptr{Cint}
   actuator_trnid::Ptr{Cint}
   actuator_ctrllimited::Ptr{mjtByte}
   actuator_forcelimited::Ptr{mjtByte}
   actuator_dynprm::Ptr{mjtNum}
   actuator_gainprm::Ptr{mjtNum}
   actuator_biasprm::Ptr{mjtNum}
   actuator_ctrlrange::Ptr{mjtNum}
   actuator_forcerange::Ptr{mjtNum}
   actuator_gear::Ptr{mjtNum}
   actuator_cranklength::Ptr{mjtNum}
   actuator_invweight0::Ptr{mjtNum}
   actuator_length0::Ptr{mjtNum}
   actuator_lengthrange::Ptr{mjtNum}
   actuator_user::Ptr{mjtNum}
   sensor_type::Ptr{Cint}
   sensor_datatype::Ptr{Cint}
   sensor_needstage::Ptr{Cint}
   sensor_objtype::Ptr{Cint}
   sensor_objid::Ptr{Cint}
   sensor_dim::Ptr{Cint}
   sensor_adr::Ptr{Cint}
   sensor_noise::Ptr{mjtNum}
   sensor_user::Ptr{mjtNum}
   numeric_adr::Ptr{Cint}
   numeric_size::Ptr{Cint}
   numeric_data::Ptr{mjtNum}
   text_adr::Ptr{Cint}
   text_size::Ptr{Cint}
   text_data::Ptr{UInt8}
   tuple_adr::Ptr{Cint}
   tuple_size::Ptr{Cint}
   tuple_objtype::Ptr{Cint}
   tuple_objid::Ptr{Cint}
   tuple_objprm::Ptr{mjtNum}
   key_time::Ptr{mjtNum}
   key_qpos::Ptr{mjtNum}
   key_qvel::Ptr{mjtNum}
   key_act::Ptr{mjtNum}
   name_bodyadr::Ptr{Cint}
   name_jntadr::Ptr{Cint}
   name_geomadr::Ptr{Cint}
   name_siteadr::Ptr{Cint}
   name_camadr::Ptr{Cint}
   name_lightadr::Ptr{Cint}
   name_meshadr::Ptr{Cint}
   name_hfieldadr::Ptr{Cint}
   name_texadr::Ptr{Cint}
   name_matadr::Ptr{Cint}
   name_eqadr::Ptr{Cint}
   name_tendonadr::Ptr{Cint}
   name_actuatoradr::Ptr{Cint}
   name_sensoradr::Ptr{Cint}
   name_numericadr::Ptr{Cint}
   name_textadr::Ptr{Cint}
   name_tupleadr::Ptr{Cint}
   names::Ptr{UInt8}
end

const mjModel = _mjModel



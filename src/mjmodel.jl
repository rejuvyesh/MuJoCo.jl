
const MINVAL = 1.0e-15
const PI = 3.141592653589793
const MAXVAL = 1.0e10
const MINMU = 1.0e-5
const MINIMP = 0.0001
const MAXIMP = 0.9999
const MAXCONPAIR = 50
const MAXVFS = 200
const MAXVFSNAME = 100

const NEQDATA = 7
const NDYN = 3
const NGAIN = 3
const NBIAS = 3
const NREF = 2
const NIMP = 3
const NSOLVER = 1000


@enum mjtDisableBit DSBL_CONSTRAINT = (UInt32)(1) DSBL_EQUALITY = (UInt32)(2) DSBL_FRICTIONLOSS = (UInt32)(4) DSBL_LIMIT = (UInt32)(8) DSBL_CONTACT = (UInt32)(16) DSBL_MOCAP = (UInt32)(32) DSBL_SENSOR = (UInt32)(64) DSBL_PASSIVE = (UInt32)(128) DSBL_GRAVITY = (UInt32)(256) DSBL_CLAMPCTRL = (UInt32)(512) DSBL_WARMSTART = (UInt32)(1024) DSBL_FILTERPARENT = (UInt32)(2048) DSBL_ACTUATION = (UInt32)(4096) DSBL_BROADPHASE = (UInt32)(8192) DSBL_REFSAFE = (UInt32)(16384) NDISABLE = (UInt32)(15)

@enum mjtEnableBit ENBL_OVERRIDE = (UInt32)(1) ENBL_BOXCONVEX = (UInt32)(2) ENBL_ENERGY = (UInt32)(4) ENBL_FWDINV = (UInt32)(8) ENBL_SENSORNOISE = (UInt32)(16) NENABLE = (UInt32)(5) 

@enum mjtJoint JNT_FREE = (UInt32)(0) JNT_BALL = (UInt32)(1) JNT_SLIDE = (UInt32)(2) JNT_HINGE = (UInt32)(3) 

@enum mjtGeom GEOM_PLANE = (UInt32)(0) GEOM_HFIELD = (UInt32)(1) GEOM_SPHERE = (UInt32)(2) GEOM_CAPSULE = (UInt32)(3) GEOM_ELLIPSOID = (UInt32)(4) GEOM_CYLINDER = (UInt32)(5) GEOM_BOX = (UInt32)(6) GEOM_MESH = (UInt32)(7) NGEOMTYPES = (UInt32)(8) GEOM_ARROW = (UInt32)(100) GEOM_ARROW1 = (UInt32)(101) GEOM_ARROW2 = (UInt32)(102) GEOM_LABEL = (UInt32)(103) GEOM_NONE = (UInt32)(1001)  

@enum mjtCamLight CAMLIGHT_FIXED = (UInt32)(0) CAMLIGHT_TRACK = (UInt32)(1) CAMLIGHT_TRACKCOM = (UInt32)(2) CAMLIGHT_TARGETBODY = (UInt32)(3) CAMLIGHT_TARGETBODYCOM = (UInt32)(4) 

@enum mjtTexture mjtEXTURE_2D = (UInt32)(0) mjtEXTURE_CUBE = (UInt32)(1) mjtEXTURE_SKYBOX = (UInt32)(2) 

@enum mjtIntegrator INT_EULER = (UInt32)(0) INT_RK4 = (UInt32)(1) 

@enum mjtCollision COL_ALL = (UInt32)(0) COL_PAIR = (UInt32)(1) COL_DYNAMIC = (UInt32)(2) 

@enum mjtCone CONE_PYRAMIDAL = (UInt32)(0) CONE_ELLIPTIC = (UInt32)(1)

@enum mjtJacobian JAC_DENSE = (UInt32)(0) JAC_SPARSE = (UInt32)(1) JAC_AUTO = (UInt32)(2)

@enum mjtSolver SOL_PGS = (UInt32)(0) SOL_CG = (UInt32)(1) SOL_NEWTON = (UInt32)(2)

@enum mjtImp IMP_CONSTANT = (UInt32)(0) IMP_SIGMOID = (UInt32)(1) IMP_LINEAR = (UInt32)(2) IMP_USER = (UInt32)(3) 

@enum mjtRef REF_SPRING = (UInt32)(0) REF_USER = (UInt32)(1) 

@enum mjtEq EQ_CONNECT = (UInt32)(0) EQ_WELD = (UInt32)(1) EQ_JOINT = (UInt32)(2) EQ_TENDON = (UInt32)(3) EQ_DISTANCE = (UInt32)(4) 

@enum mjtWrap WRAP_NONE = (UInt32)(0) WRAP_JOINT = (UInt32)(1) WRAP_PULLEY = (UInt32)(2) WRAP_SITE = (UInt32)(3) WRAP_SPHERE = (UInt32)(4) WRAP_CYLINDER = (UInt32)(5) 

@enum mjtTrn mjtRN_JOINT = (UInt32)(0) mjtRN_JOINTINPARENT = (UInt32)(1) mjtRN_SLIDERCRANK = (UInt32)(2) mjtRN_TENDON = (UInt32)(3) mjtRN_SITE = (UInt32)(4) mjtRN_UNDEFINED = (UInt32)(1000) 

@enum mjtDyn DYN_NONE = (UInt32)(0) DYN_INTEGRATOR = (UInt32)(1) DYN_FILTER = (UInt32)(2) DYN_USER = (UInt32)(3) 

@enum mjtGain GAIN_FIXED = (UInt32)(0) GAIN_USER = (UInt32)(1) 

@enum mjtBias BIAS_NONE = (UInt32)(0) BIAS_AFFINE = (UInt32)(1) BIAS_USER = (UInt32)(2) 

@enum mjtObj OBJ_UNKNOWN = (UInt32)(0) OBJ_BODY = (UInt32)(1) OBJ_XBODY = (UInt32)(2) OBJ_JOINT = (UInt32)(3) OBJ_DOF = (UInt32)(4) OBJ_GEOM = (UInt32)(5) OBJ_SITE = (UInt32)(6) OBJ_CAMERA = (UInt32)(7) OBJ_LIGHT = (UInt32)(8) OBJ_MESH = (UInt32)(9) OBJ_HFIELD = (UInt32)(10) OBJ_TEXTURE = (UInt32)(11) OBJ_MATERIAL = (UInt32)(12) OBJ_PAIR = (UInt32)(13) OBJ_EXCLUDE = (UInt32)(14) OBJ_EQUALITY = (UInt32)(15) OBJ_TENDON = (UInt32)(16) OBJ_ACTUATOR = (UInt32)(17) OBJ_SENSOR = (UInt32)(18) OBJ_NUMERIC = (UInt32)(19) OBJ_TEXT = (UInt32)(20) OBJ_TUPLE = (UInt32)(21) OBJ_KEY = (UInt32)(22) 

@enum mjtConstraint CNSTR_EQUALITY = (UInt32)(0) CNSTR_FRICTION_DOF = (UInt32)(1) CNSTR_FRICTION_TENDON = (UInt32)(2) CNSTR_LIMIT_JOINT = (UInt32)(3) CNSTR_LIMIT_TENDON = (UInt32)(4) CNSTR_CONTACT_FRICTIONLESS = (UInt32)(5) CNSTR_CONTACT_PYRAMIDAL = (UInt32)(6) CNSTR_CONTACT_ELLIPTIC = (UInt32)(7) 

@enum mjtConstraintState CNSTRSTATE_SATISFIED = (UInt32)(0) CNSTRSTATE_QUADRATIC = (UInt32)(1) CNSTRSTATE_LINEARNEG = (UInt32)(2) CNSTRSTATE_LINEARPOS = (UInt32)(3) CNSTRSTATE_CONE = (UInt32)(4)

@enum mjtSensor SENS_TOUCH = (UInt32)(0) SENS_ACCELEROMETER = (UInt32)(1) SENS_VELOCIMETER = (UInt32)(2) SENS_GYRO = (UInt32)(3) SENS_FORCE = (UInt32)(4) SENS_TORQUE = (UInt32)(5) SENS_MAGNETOMETER = (UInt32)(6) SENS_RANGEFINDER = (UInt32)(7) SENS_JOINTPOS = (UInt32)(8) SENS_JOINTVEL = (UInt32)(9) SENS_TENDONPOS = (UInt32)(10) SENS_TENDONVEL = (UInt32)(11) SENS_ACTUATORPOS = (UInt32)(12) SENS_ACTUATORVEL = (UInt32)(13) SENS_ACTUATORFRC = (UInt32)(14) SENS_BALLQUAT = (UInt32)(15) SENS_BALLANGVEL = (UInt32)(16) SENS_FRAMEPOS = (UInt32)(17) SENS_FRAMEQUAT = (UInt32)(18) SENS_FRAMEXAXIS = (UInt32)(19) SENS_FRAMEYAXIS = (UInt32)(20) SENS_FRAMEZAXIS = (UInt32)(21) SENS_FRAMELINVEL = (UInt32)(22) SENS_FRAMEANGVEL = (UInt32)(23) SENS_FRAMELINACC = (UInt32)(24) SENS_FRAMEANGACC = (UInt32)(25) SENS_SUBTREECOM = (UInt32)(26) SENS_SUBTREELINVEL = (UInt32)(27) SENS_SUBTREEANGMOM = (UInt32)(28) SENS_USER = (UInt32)(29) 

@enum mjtStage STAGE_NONE = (UInt32)(0) STAGE_POS = (UInt32)(1) STAGE_VEL = (UInt32)(2) STAGE_ACC = (UInt32)(3) 

@enum mjtDataType DATATYPE_REAL = (UInt32)(0) DATATYPE_POSITIVE = (UInt32)(1) DATATYPE_AXIS = (UInt32)(2) DATATYPE_QUAT = (UInt32)(3)

immutable VFS
   nfile::Cint
	filename::SVector{MAXVFS, SVector{MAXVFSNAME, UInt8}}
   filesize::SVector{MAXVFS, Cint}
   filedata::SVector{MAXVFS, Ptr{Void}}
end

immutable Option
   timestep::mjtNum
   apirate::mjtNum

   impratio::mjtNum
   tolerance::mjtNum
   noslip_tolerance::mjtNum
   mpr_tolerance::mjtNum

   gravity::SVector{3, mjtNum}
   wind::SVector{3, mjtNum}
   magnetic::SVector{3, mjtNum}
   density::mjtNum
   viscosity::mjtNum

   o_margin::mjtNum
   o_solref::SVector{NREF, mjtNum}
   o_solimp::SVector{NIMP, mjtNum}

   integrator::Cint
   collision::Cint
   impedance::Cint
   reference::Cint
   cone::Cint
   jacobian::Cint
   solver::Cint
   iterations::Cint
   noslip_iterations::Cint
   mpr_iterations::Cint
   disableflags::Cint
   enableflags::Cint
end

immutable _global
	fovy::Float32
	ipd::Float32
	linewidth::Float32
	glow::Float32
	offwidth::Int32
	offheight::Int32
end

immutable quality
	shadowsize::Int32
	offsamples::Int32
	numslices::Int32
	numstacks::Int32
	numarrows::Int32
	numquads::Int32
end

immutable headlight
	ambient::SVector{3, Float32}
	diffuse::SVector{3, Float32}
	specular::SVector{3, Float32}
	active::Int32
end

immutable map
	stiffness::Float32
	stiffnessrot::Float32
	force::Float32
	torque::Float32
	alpha::Float32
	fogstart::Float32
	fogend::Float32
	znear::Float32
	zfar::Float32
	shadowclip::Float32
	shadowscale::Float32
end

immutable scale
	forcewidth::Float32
	contactwidth::Float32
	contactheight::Float32
	connect::Float32
	com::Float32
	camera::Float32
	light::Float32
	selectpoint::Float32
	jointlength::Float32
	jointwidth::Float32
	actuatorlength::Float32
	actuatorwidth::Float32
	framelength::Float32
	framewidth::Float32
	constraint::Float32
	slidercrank::Float32
end

immutable rgba
	fog::SVector{4, Float32}
	force::SVector{4, Float32}
	inertia::SVector{4, Float32}
	joint::SVector{4, Float32}
	actuator::SVector{4, Float32}
	com::SVector{4, Float32}
	camera::SVector{4, Float32}
	light::SVector{4, Float32}
	selectpoint::SVector{4, Float32}
	connect::SVector{4, Float32}
	contactpoint::SVector{4, Float32}
	contactforce::SVector{4, Float32}
	contactfriction::SVector{4, Float32}
	contacttorque::SVector{4, Float32}
	constraint::SVector{4, Float32}
	slidercrank::SVector{4, Float32}
	crankbroken::SVector{4, Float32}
end

immutable Visual
	_global::_global
	quality::quality
	headlight::headlight
	map::map
	scale::scale
	rgba::rgba
end

#struct _Statistic
immutable Statistic
	meaninertia::mjtNum
	meanmass::mjtNum
	meansize::mjtNum
	extent::mjtNum
	center::SVector{3, mjtNum}
end

immutable Model
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
	nuser_cam::Cint
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

	opt::Option
	vis::Visual
	stat::Statistic

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
	cam_user::Ptr{mjtNum}

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
	sensor_cutoff::Ptr{mjtNum}
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




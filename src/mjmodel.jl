
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
const NDYN = 10
const NGAIN = 10
const NBIAS = 10
const NREF = 2
const NIMP = 5
const NSOLVER = 1000

@enum mjtDisableBit begin             # disable default feature bitflags
    DSBL_CONSTRAINT   = 1<<0   # entire constraint solver
    DSBL_EQUALITY     = 1<<1   # equality constraints
    DSBL_FRICTIONLOSS = 1<<2   # joint and tendon frictionloss constraints
    DSBL_LIMIT        = 1<<3   # joint and tendon limit constraints
    DSBL_CONTACT      = 1<<4   # contact constraints
    DSBL_PASSIVE      = 1<<5   # passive forces
    DSBL_GRAVITY      = 1<<6   # gravitational forces
    DSBL_CLAMPCTRL    = 1<<7   # clamp control to specified range
    DSBL_WARMSTART    = 1<<8   # warmstart constraint solver
    DSBL_FILTERPARENT = 1<<9   # remove collisions with parent body
    DSBL_ACTUATION    = 1<<10  # apply actuation forces
    DSBL_REFSAFE      = 1<<11  # integrator safety: make ref[0]>=2*timestep

    NDISABLE          = 12     # number of disable flags
end

@enum mjtEnableBit begin              # enable optional feature bitflags
    ENBL_OVERRIDE     = 1<<0   # override contact parameters
    ENBL_ENERGY       = 1<<1   # energy computation
    ENBL_FWDINV       = 1<<2   # record solver statistics
    ENBL_SENSORNOISE  = 1<<3   # add noise to sensor data

    #NENABLE           = 4      # number of enable flags
end
const NENABLE = 4

@enum mjtJoint begin                  # type of degree of freedom
    JNT_FREE          = 0      # global position and orientation (quat)       (7)
    JNT_BALL                   # orientation (quat) relative to parent        (4)
    JNT_SLIDE                  # sliding distance along body-fixed axis       (1)
    JNT_HINGE                  # rotation angle (rad) around body-fixed axis  (1)
end

@enum mjtGeom begin                   # type of geometric shape
                                 # regular geom types
    GEOM_PLANE        = 0      # plane
    GEOM_HFIELD                # height field
    GEOM_SPHERE                # sphere
    GEOM_CAPSULE               # capsule
    GEOM_ELLIPSOID             # ellipsoid
    GEOM_CYLINDER              # cylinder
    GEOM_BOX                   # box
    GEOM_MESH                  # mesh

    NGEOMTYPES                 # number of regular geom types

                               # rendering-only geom types: not used in mjModel not counted in mjNGEOMTYPES
    GEOM_ARROW        = 100    # arrow
    GEOM_ARROW1                # arrow without wedges
    GEOM_ARROW2                # arrow in both directions
    GEOM_LINE                  # line
    GEOM_SKIN                  # skin
    GEOM_LABEL                 # text label

    GEOM_NONE         = 1001   # missing geom type
end

@enum mjtCamLight begin               # tracking mode for camera and light
    CAMLIGHT_FIXED    = 0      # pos and rot fixed in body
    CAMLIGHT_TRACK             # pos tracks body, rot fixed in global
    CAMLIGHT_TRACKCOM          # pos tracks subtree com, rot fixed in body
    CAMLIGHT_TARGETBODY        # pos fixed in body, rot tracks target body
    CAMLIGHT_TARGETBODYCOM     # pos fixed in body rot tracks target subtree com
end

@enum mjtTexture begin                # type of texture
    TEXTURE_2D        = 0      # 2d texture, suitable for planes and hfields
    TEXTURE_CUBE               # cube texture, suitable for all other geom types
    TEXTURE_SKYBOX             # cube texture used as skybox
end

@enum mjtIntegrator begin             # integrator mode
    INT_EULER         = 0      # semi-implicit Euler
    INT_RK4                    # 4th-order Runge Kutta
end

@enum mjtCollision begin              # collision mode for selecting geom pairs
    COL_ALL           = 0      # test precomputed and dynamic pairs
    COL_PAIR                   # test predefined pairs only
    COL_DYNAMIC                # test dynamic pairs only
end

@enum mjtCone begin                   # type of friction cone
    CONE_PYRAMIDAL     = 0     # pyramidal
    CONE_ELLIPTIC              # elliptic
end

@enum mjtJacobian begin               # type of constraint Jacobian
    JAC_DENSE          = 0     # dense
    JAC_SPARSE                 # sparse
    JAC_AUTO                   # dense if nv<60 sparse otherwise
end

@enum mjtSolver begin                 # constraint solver algorithm
    SOL_PGS            = 0     # PGS    (dual)
    SOL_CG                     # CG     (primal)
    SOL_NEWTON                 # Newton (primal)
end

@enum mjtEq begin                     # type of equality constraint
    EQ_CONNECT        = 0      # connect two bodies at a point (ball joint)
    EQ_WELD                    # fix relative position and orientation of two bodies
    EQ_JOINT                   # couple the values of two scalar joints with cubic
    EQ_TENDON                  # couple the lengths of two tendons with cubic
    EQ_DISTANCE                # fix the contact distance betweent two geoms
end

@enum mjtWrap begin                   # type of tendon wrap object
    WRAP_NONE         = 0      # null object
    WRAP_JOINT                 # constant moment arm
    WRAP_PULLEY                # pulley used to split tendon
    WRAP_SITE                  # pass through site
    WRAP_SPHERE                # wrap around sphere
    WRAP_CYLINDER              # wrap around (infinite) cylinder
end

@enum mjtTrn begin                    # type of actuator transmission
    TRN_JOINT         = 0      # force on joint
    TRN_JOINTINPARENT          # force on joint, expressed in parent frame
    TRN_SLIDERCRANK            # force via slider-crank linkage
    TRN_TENDON                 # force on tendon
    TRN_SITE                   # force on site

    TRN_UNDEFINED     = 1000   # undefined transmission type
end

@enum mjtDyn begin                    # type of actuator dynamics
    DYN_NONE          = 0      # no internal dynamics; ctrl specifies force
    DYN_INTEGRATOR             # integrator: da/dt = u
    DYN_FILTER                 # linear filter: da/dt = (u-a) / tau
    DYN_MUSCLE                 # piece-wise linear filter with two time constants
    DYN_USER                   # user-defined dynamics type
end

@enum mjtGain begin                   # type of actuator gain
    GAIN_FIXED        = 0      # fixed gain
    GAIN_MUSCLE                # muscle FLV curve computed by mju_muscleGain()
    GAIN_USER                  # user-defined gain type
end

@enum mjtBias begin                   # type of actuator bias
    BIAS_NONE         = 0      # no bias
    BIAS_AFFINE                # const + kp*length + kv*velocity
    BIAS_MUSCLE                # muscle passive force computed by mju_muscleBias()
    BIAS_USER                  # user-defined bias type
end

@enum mjtObj begin                    # type of MujoCo object
    OBJ_UNKNOWN       = 0      # unknown object type
    OBJ_BODY                   # body
    OBJ_XBODY                  # body, used to access regular frame instead of i-frame
    OBJ_JOINT                  # joint
    OBJ_DOF                    # dof
    OBJ_GEOM                   # geom
    OBJ_SITE                   # site
    OBJ_CAMERA                 # camera
    OBJ_LIGHT                  # light
    OBJ_MESH                   # mesh
    OBJ_SKIN                   # skin
    OBJ_HFIELD                 # heightfield
    OBJ_TEXTURE                # texture
    OBJ_MATERIAL               # material for rendering
    OBJ_PAIR                   # geom pair to include
    OBJ_EXCLUDE                # body pair to exclude
    OBJ_EQUALITY               # equality constraint
    OBJ_TENDON                 # tendon
    OBJ_ACTUATOR               # actuator
    OBJ_SENSOR                 # sensor
    OBJ_NUMERIC                # numeric
    OBJ_TEXT                   # text
    OBJ_TUPLE                  # tuple
    OBJ_KEY                    # keyframe
end

@enum mjtConstraint begin             # type of constraint
    CNSTR_EQUALITY    = 0      # equality constraint
    CNSTR_FRICTION_DOF         # dof friction
    CNSTR_FRICTION_TENDON      # tendon friction
    CNSTR_LIMIT_JOINT          # joint limit
    CNSTR_LIMIT_TENDON         # tendon limit
    CNSTR_CONTACT_FRICTIONLESS # frictionless contact
    CNSTR_CONTACT_PYRAMIDAL    # frictional contact, pyramidal friction cone
    CNSTR_CONTACT_ELLIPTIC     # frictional contact elliptic friction cone
end

@enum mjtConstraintState begin        # constraint state
    CNSTRSTATE_SATISFIED = 0   # constraint satisfied, zero cost (limit, contact)
    CNSTRSTATE_QUADRATIC       # quadratic cost (equality, friction, limit, contact)
    CNSTRSTATE_LINEARNEG       # linear cost, negative side (friction)
    CNSTRSTATE_LINEARPOS       # linear cost, positive side (friction)
    CNSTRSTATE_CONE            # squared distance to cone cost (elliptic contact)
end

@enum mjtSensor begin                 # type of sensor
                                 # common robotic sensors attached to a site
    SENS_TOUCH        = 0      # scalar contact normal forces summed over sensor zone
    SENS_ACCELEROMETER         # 3D linear acceleration, in local frame
    SENS_VELOCIMETER           # 3D linear velocity, in local frame
    SENS_GYRO                  # 3D angular velocity, in local frame
    SENS_FORCE                 # 3D force between site's body and its parent body
    SENS_TORQUE                # 3D torque between site's body and its parent body
    SENS_MAGNETOMETER          # 3D magnetometer
    SENS_RANGEFINDER           # scalar distance to nearest geom or site along z-axis

                               # sensors related to scalar joints tendons, actuators
    SENS_JOINTPOS              # scalar joint position (hinge and slide only)
    SENS_JOINTVEL              # scalar joint velocity (hinge and slide only)
    SENS_TENDONPOS             # scalar tendon position
    SENS_TENDONVEL             # scalar tendon velocity
    SENS_ACTUATORPOS           # scalar actuator position
    SENS_ACTUATORVEL           # scalar actuator velocity
    SENS_ACTUATORFRC           # scalar actuator force

                               # sensors related to ball joints
    SENS_BALLQUAT              # 4D ball joint quaterion
    SENS_BALLANGVEL            # 3D ball joint angular velocity

                               # joint and tendon limit sensors in constraint space
    SENS_JOINTLIMITPOS         # joint limit distance-margin
    SENS_JOINTLIMITVEL         # joint limit velocity
    SENS_JOINTLIMITFRC         # joint limit force
    SENS_TENDONLIMITPOS        # tendon limit distance-margin
    SENS_TENDONLIMITVEL        # tendon limit velocity
    SENS_TENDONLIMITFRC        # tendon limit force

                               # sensors attached to an object with spatial frame: (x)body geom, site, camera
    SENS_FRAMEPOS              # 3D position
    SENS_FRAMEQUAT             # 4D unit quaternion orientation
    SENS_FRAMEXAXIS            # 3D unit vector: x-axis of object's frame
    SENS_FRAMEYAXIS            # 3D unit vector: y-axis of object's frame
    SENS_FRAMEZAXIS            # 3D unit vector: z-axis of object's frame
    SENS_FRAMELINVEL           # 3D linear velocity
    SENS_FRAMEANGVEL           # 3D angular velocity
    SENS_FRAMELINACC           # 3D linear acceleration
    SENS_FRAMEANGACC           # 3D angular acceleration

                               # sensors related to kinematic subtrees; attached to a body (which is the subtree root)
    SENS_SUBTREECOM            # 3D center of mass of subtree
    SENS_SUBTREELINVEL         # 3D linear velocity of subtree
    SENS_SUBTREEANGMOM         # 3D angular momentum of subtree

                               # user-defined sensor
    SENS_USER                  # sensor data provided by mjcb_sensor callback
end

@enum mjtStage begin                  # computation stage
    STAGE_NONE        = 0      # no computations
    STAGE_POS                  # position-dependent computations
    STAGE_VEL                  # velocity-dependent computations
    STAGE_ACC                  # acceleration/force-dependent computations
end

@enum mjtDataType begin               # data type for sensors
    DATATYPE_REAL     = 0      # real values, no constraints
    DATATYPE_POSITIVE          # positive values; 0 or negative: inactive
    DATATYPE_AXIS              # 3D unit vector
    DATATYPE_QUATERNION        # unit quaternion
end

@enum mjtLRMode begin                 # mode for actuator length range computation
    LRMODE_NONE   = 0          # do not process any actuators
    LRMODE_MUSCLE              # process muscle actuators
    LRMODE_MUSCLEUSER          # process muscle and user actuators
    LRMODE_ALL                 # process all actuators
end

#------------------------------ mjLROpt ------------------------------------------------

struct mjLROpt         # options for mj_setLengthRange()
                     # flags
   mode              # which actuators to process (mjtLRMode)
   useexisting::Cint # use existing length range if available
   uselimit::Cint    # use joint and tendon limits if available

                     # algorithm parameters
   accel::mjtNum     # target acceleration used to compute force
   maxforce::mjtNum  # maximum force 0: no limit
   timeconst::mjtNum # time constant for velocity reduction min 0.01
   timestep::mjtNum  # simulation timestep 0: use mjOption.timestep
   inttotal::mjtNum  # total simulation time interval
   inteval::mjtNum   # evaluation time interval (at the end)
   tolrange::mjtNum  # convergence tolerance (relative to range)
end


struct mjVFS
   nfile::Cint
   filename::SVector{MAXVFS, SVector{MAXVFSNAME, UInt8}}
   filesize::SVector{MAXVFS, Cint}
   filedata::SVector{MAXVFS, Ptr{Cvoid}}
end

struct mjOption
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
    cone::Cint
    jacobian::Cint
    solver::Cint
    iterations::Cint
    noslip_iterations::Cint
    mpr_iterations::Cint
    disableflags::Cint
    enableflags::Cint
end

struct _global
    fovy::Cfloat
    ipd::Cfloat
    linewidth::Cfloat
    glow::Cfloat
    offwidth::Cint
    offheight::Cint
end

struct quality
    shadowsize::Cint
    offsamples::Cint
    numslices::Cint
    numstacks::Cint
    numquads::Cint
end

struct headlight
    ambient::SVector{3, Cfloat}
    diffuse::SVector{3, Cfloat}
    specular::SVector{3, Cfloat}
    active::Cint
end

struct map
    stiffness::Cfloat
    stiffnessrot::Cfloat
    force::Cfloat
    torque::Cfloat
    alpha::Cfloat
    fogstart::Cfloat
    fogend::Cfloat
    znear::Cfloat
    zfar::Cfloat
    haze::Cfloat
    shadowclip::Cfloat
    shadowscale::Cfloat
    actuatortendon::Cfloat
end

struct scale
    forcewidth::Cfloat
    contactwidth::Cfloat
    contactheight::Cfloat
    connect::Cfloat
    com::Cfloat
    camera::Cfloat
    light::Cfloat
    selectpoint::Cfloat
    jointlength::Cfloat
    jointwidth::Cfloat
    actuatorlength::Cfloat
    actuatorwidth::Cfloat
    framelength::Cfloat
    framewidth::Cfloat
    constraint::Cfloat
    slidercrank::Cfloat
end

struct rgba
    fog::SVector{4, Cfloat}
    haze::SVector{4, Cfloat}
    force::SVector{4, Cfloat}
    inertia::SVector{4, Cfloat}
    joint::SVector{4, Cfloat}
    actuator::SVector{4, Cfloat}
    actuatornegative::SVector{4, Cfloat}
    actuatorpositive::SVector{4, Cfloat}
    com::SVector{4, Cfloat}
    camera::SVector{4, Cfloat}
    light::SVector{4, Cfloat}
    selectpoint::SVector{4, Cfloat}
    connect::SVector{4, Cfloat}
    contactpoint::SVector{4, Cfloat}
    contactforce::SVector{4, Cfloat}
    contactfriction::SVector{4, Cfloat}
    contacttorque::SVector{4, Cfloat}
    contactgap::SVector{4, Cfloat}
    rangefinder::SVector{4, Cfloat}
    constraint::SVector{4, Cfloat}
    slidercrank::SVector{4, Cfloat}
    crankbroken::SVector{4, Cfloat}
end

struct mjVisual
    _global::_global
    quality::quality
    headlight::headlight
    map::map
    scale::scale
    rgba::rgba
end

struct mjStatistic
    meaninertia::mjtNum
    meanmass::mjtNum
    meansize::mjtNum
    extent::mjtNum
    center::SVector{3, mjtNum}
end

struct mjModel
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
    nmeshtexvert::Cint
    nmeshface::Cint
    nmeshgraph::Cint
    nskin::Cint
    nskinvert::Cint
    nskintexvert::Cint
    nskinface::Cint
    nskinbone::Cint
    nskinbonevert::Cint
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

    opt::mjOption
    vis::mjVisual
    stat::mjStatistic

    buffer::Ptr{Cvoid}

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
    body_simple::Ptr{mjtByte}
    body_sameframe::Ptr{mjtByte}
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
    jnt_group::Ptr{Cint}
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
    dof_simplenum::Ptr{Cint}
    dof_solref::Ptr{mjtNum}
    dof_solimp::Ptr{mjtNum}
    dof_frictionloss::Ptr{mjtNum}
    dof_armature::Ptr{mjtNum}
    dof_damping::Ptr{mjtNum}
    dof_invweight0::Ptr{mjtNum}
    dof_M0::Ptr{mjtNum}

    geom_type::Ptr{Cint}
    geom_contype::Ptr{Cint}
    geom_conaffinity::Ptr{Cint}
    geom_condim::Ptr{Cint}
    geom_bodyid::Ptr{Cint}
    geom_dataid::Ptr{Cint}
    geom_matid::Ptr{Cint}
    geom_group::Ptr{Cint}
    geom_priority::Ptr{Cint}
    geom_sameframe::Ptr{mjtByte}
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
    site_sameframe::Ptr{mjtByte}
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

    mesh_vertadr::Ptr{Cint}
    mesh_vertnum::Ptr{Cint}
    mesh_texcoordadr::Ptr{Cint}
    mesh_faceadr::Ptr{Cint}
    mesh_facenum::Ptr{Cint}
    mesh_graphadr::Ptr{Cint}
    mesh_vert::Ptr{Cfloat}
    mesh_normal::Ptr{Cfloat}
    mesh_texcoord::Ptr{Cfloat}
    mesh_face::Ptr{Cint}
    mesh_graph::Ptr{Cint}

    skin_matid::Ptr{Cint}
    skin_rgba::Ptr{Cfloat}
    skin_inflate::Ptr{Cfloat}
    skin_vertadr::Ptr{Cint}
    skin_vertnum::Ptr{Cint}
    skin_texcoordadr::Ptr{Cint}
    skin_faceadr::Ptr{Cint}
    skin_facenum::Ptr{Cint}
    skin_boneadr::Ptr{Cint}
    skin_bonenum::Ptr{Cint}
    skin_vert::Ptr{Cfloat}
    skin_texcoord::Ptr{Cfloat}
    skin_face::Ptr{Cint}
    skin_bonevertadr::Ptr{Cint}
    skin_bonevertnum::Ptr{Cint}
    skin_bonebindpos::Ptr{Cfloat}
    skin_bonebindquat::Ptr{Cfloat}
    skin_bonebodyid::Ptr{Cint}
    skin_bonevertid::Ptr{Cint}
    skin_bonevertweight::Ptr{Cfloat}

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
    tendon_group::Ptr{Cint}
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
    actuator_group::Ptr{Cint}
    actuator_ctrllimited::Ptr{mjtByte}
    actuator_forcelimited::Ptr{mjtByte}
    actuator_dynprm::Ptr{mjtNum}
    actuator_gainprm::Ptr{mjtNum}
    actuator_biasprm::Ptr{mjtNum}
    actuator_ctrlrange::Ptr{mjtNum}
    actuator_forcerange::Ptr{mjtNum}
    actuator_gear::Ptr{mjtNum}
    actuator_cranklength::Ptr{mjtNum}
    actuator_acc0::Ptr{mjtNum}
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
    name_skinadr::Ptr{Cint}
    name_hfieldadr::Ptr{Cint}
    name_texadr::Ptr{Cint}
    name_matadr::Ptr{Cint}
    name_pairadr::Ptr{Cint}
    name_excludeadr::Ptr{Cint}
    name_eqadr::Ptr{Cint}
    name_tendonadr::Ptr{Cint}
    name_actuatoradr::Ptr{Cint}
    name_sensoradr::Ptr{Cint}
    name_numericadr::Ptr{Cint}
    name_textadr::Ptr{Cint}
    name_tupleadr::Ptr{Cint}
    name_keyadr::Ptr{Cint}
    names::Ptr{UInt8}
end




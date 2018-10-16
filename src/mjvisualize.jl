
const NGROUP       = 6
const MAXOVERLAY   = 500

const MAXLINE      = 100
const MAXLINEPNT   = 1000
const MAXPLANEGRID = 200

@enum mjtCatBit begin           # bitflags for vGeom category
    CAT_STATIC        = 1  # model elements in body 0
    CAT_DYNAMIC       = 2  # model elements in all other bodies
    CAT_DECOR         = 4  # decorative geoms
    CAT_ALL           = 7  # select all categories
end

@enum mjtMouse begin            # mouse interaction mode
    MOUSE_NONE         = 0 # no action
    MOUSE_ROTATE_V         # rotate, vertical plane
    MOUSE_ROTATE_H         # rotate, horizontal plane
    MOUSE_MOVE_V           # move, vertical plane
    MOUSE_MOVE_H           # move, horizontal plane
    MOUSE_ZOOM             # zoom
    MOUSE_SELECT           # selection
end

@enum mjtPertBit begin          # mouse perturbations
    PERT_TRANSLATE    = 1  # translation
    PERT_ROTATE       = 2  # rotation
end

@enum mjtCamera begin           # abstract camera type
    CAMERA_FREE        = 0 # free camera
    CAMERA_TRACKING        # tracking camera; uses trackbodyid
    CAMERA_FIXED           # fixed camera; uses fixedcamid
    CAMERA_USER            # user is responsible for setting OpenGL camera
end

@enum mjtLabel begin            # object labeling
    LABEL_NONE        = 0  # nothing
    LABEL_BODY             # body labels
    LABEL_JOINT            # joint labels
    LABEL_GEOM             # geom labels
    LABEL_SITE             # site labels
    LABEL_CAMERA           # camera labels
    LABEL_LIGHT            # light labels
    LABEL_TENDON           # tendon labels
    LABEL_ACTUATOR         # actuator labels
    LABEL_CONSTRAINT       # constraint labels
    LABEL_SKIN             # skin labels
    LABEL_SELECTION        # selected object
    LABEL_SELPNT           # coordinates of selection point
    LABEL_CONTACTFORCE     # magnitude of contact force

    NLABEL                 # number of label types
end

@enum mjtFrame begin            # frame visualization
    FRAME_NONE        = 0  # no frames
    FRAME_BODY             # body frames
    FRAME_GEOM             # geom frames
    FRAME_SITE             # site frames
    FRAME_CAMERA           # camera frames
    FRAME_LIGHT            # light frames
    FRAME_WORLD            # world frame

    NFRAME                 # number of visualization frames
end

@enum mjtVisFlag begin          # flags enabling model element visualization
    VIS_CONVEXHULL    = 0  # mesh convex hull
    VIS_TEXTURE            # textures
    VIS_JOINT              # joints
    VIS_ACTUATOR           # actuators
    VIS_CAMERA             # cameras
    VIS_LIGHT              # lights
    VIS_TENDON             # tendons
    VIS_RANGEFINDER        # rangefinder sensors
    VIS_CONSTRAINT         # point constraints
    VIS_INERTIA            # equivalent inertia boxes
    VIS_SCLINERTIA         # scale equivalent inertia boxes with mass
    VIS_PERTFORCE          # perturbation force
    VIS_PERTOBJ            # perturbation object
    VIS_CONTACTPOINT       # contact points
    VIS_CONTACTFORCE       # contact force
    VIS_CONTACTSPLIT       # split contact force into normal and tanget
    VIS_TRANSPARENT        # make dynamic geoms more transparent
    VIS_AUTOCONNECT        # auto connect joints and body coms
    VIS_COM                # center of mass
    VIS_SELECT             # selection point
    VIS_STATIC             # static bodies
    VIS_SKIN               # skin

    NVISFLAG               # number of visualization flags
end

@enum mjtRndFlag begin          # flags enabling rendering effects
    RND_SHADOW        = 0  # shadows
    RND_WIREFRAME          # wireframe
    RND_REFLECTION         # reflections
    RND_ADDITIVE           # additive transparency
    RND_SKYBOX             # skybox
    RND_FOG                # fog
    RND_HAZE               # haze
    RND_SEGMENT            # segmentation with random color
    RND_IDCOLOR            # segmentation with segid color

    NRNDFLAG               # number of rendering flags
end

@enum mjtStereo begin           # type of stereo rendering
    STEREO_NONE       = 0  # no stereo; use left eye only
    STEREO_QUADBUFFERED    # quad buffered; revert to side-by-side if no hardware support
    STEREO_SIDEBYSIDE      # side-by-side
end

struct mjvPerturb
    select::Cint
    skinselect::Cint
    active::Cint
    refpos::SVector{3, mjtNum}
    refquat::SVector{4, mjtNum}
    localpos::SVector{3, mjtNum}
    scale::mjtNum
end

struct mjvCamera
    _type::Cint
    fixedcamid::Cint
    trackbodyid::Cint
    lookat::SVector{3, mjtNum}
    distance::mjtNum
    azimuth::mjtNum
    elevation::mjtNum
end

struct mjvGLCamera
    pos::SVector{3, Cfloat}
    forward::SVector{3, Cfloat}
    up::SVector{3, Cfloat}
    frustum_center::Cfloat
    frustum_bottom::Cfloat
    frustum_top::Cfloat
    frustum_near::Cfloat
    frustum_far::Cfloat
end

struct mjvGeom
    _type::Cint
    dataid::Cint
    objtype::Cint
    objid::Cint
    category::Cint
    texid::Cint
    texuniform::Cint
    texcoord::Cint
    segid::Cint
    texrepeat::SVector{2, Cfloat}
    size::SVector{3, Cfloat}
    pos::SVector{3, Cfloat}
    mat::SVector{9, Cfloat}
    rgba::SVector{4, Cfloat}
    emission::Cfloat
    specular::Cfloat
    shininess::Cfloat
    reflectance::Cfloat
    label::SVector{100, UInt8}
    camdist::Cfloat
    rbound::Cfloat
    transparent::mjtByte
end

struct mjvLight
    pos::SVector{3, Cfloat}
    dir::SVector{3, Cfloat}
    attenuation::SVector{3, Cfloat}
    cutoff::Cfloat
    exponent::Cfloat
    ambient::SVector{3, Cfloat}
    diffuse::SVector{3, Cfloat}
    specular::SVector{3, Cfloat}
    headlight::mjtByte
    directional::mjtByte
    castshadow::mjtByte
end

struct mjvOption
    label::Cint
    frame::Cint
    geomgroup::SVector{NGROUP, mjtByte}
    sitegroup::SVector{NGROUP, mjtByte}
    jointgroup::SVector{NGROUP, mjtByte}
    tendongroup::SVector{NGROUP, mjtByte}
    actuatorgroup::SVector{NGROUP, mjtByte}
    flags::SVector{Int(NVISFLAG), mjtByte}
end

struct mjvScene
    maxgeom::Cint
    ngeom::Cint
    geoms::Ptr{mjvGeom}
    geomorder::Ptr{Cint}
    
    nskin::Cint
    skinfacenum::Ptr{Cint}
    skinvertadr::Ptr{Cint}
    skinvertnum::Ptr{Cint}
    skinvert::Ptr{Cfloat}
    skinnormal::Ptr{Cfloat}

    nlight::Cint
    lights::SVector{8, mjvLight}
    camera::SVector{2, mjvGLCamera}
    enabletransform::mjtByte
    translate::SVector{3, Cfloat}
    rotate::SVector{4, Cfloat}
    scale::Cfloat
    stereo::Cint
    flags::SVector{5, mjtByte}
end

struct mjvFigure
    flg_legend::Cint
    flg_ticklabel::SVector{2, Cint}
    flg_extend::Cint
    flg_barplot::Cint
    flg_selection::Cint
    flg_symmetric::Cint

    legendoff::Cint
    gridsize::SVector{2, Cint}
    selection::Cint
    highlight::SVector{2, Cint}
    gridrgb::SVector{3, Cfloat}
    gridwidth::Cfloat
    figurergba::SVector{4, Cfloat}
    panergba::SVector{4, Cfloat}
    legendrgba::SVector{4, Cfloat}
    textrgb::SVector{3, Cfloat}
    range::SVector{2, SVector{2, Cfloat}}
    xlabel::SVector{100, UInt8}
    title::SVector{100, UInt8}
    xformat::SVector{20, UInt8}
    yformat::SVector{20, UInt8}
    minwidth::SVector{20, UInt8}

    linepnt::SVector{MAXLINE, Cint}
    linergb::SVector{MAXLINE, SVector{3, Cfloat}}
    linewidth::SVector{MAXLINE, Cfloat}
    linedata::SVector{MAXLINE, SVector{2*MAXLINEPNT, Cfloat}}
    linename::SVector{MAXLINE, SVector{100, UInt8}}

    xaxispixel::SVector{2, Cint}
    yaxispixel::SVector{2, Cint}
    xaxisdata::SVector{2, Cint}
    yaxisdata::SVector{2, Cint}
end

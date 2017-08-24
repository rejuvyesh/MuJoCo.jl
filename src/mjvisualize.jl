
const mjNGROUP = 5
const mjMAXOVERLAY = 500

const mjMAXLINE = 100
const mjMAXLINEPNT = 500
const  mjMAXPLANEGRID = 100

@enum mjtCatBit mjCAT_STATIC = (UInt32)(1) mjCAT_DYNAMIC = (UInt32)(2) mjCAT_DECOR = (UInt32)(4) mjCAT_ALL = (UInt32)(7)

@enum mjtMouse mjMOUSE_NONE = (UInt32)(0) mjMOUSE_ROTATE_V = (UInt32)(1) mjMOUSE_ROTATE_H = (UInt32)(2) mjMOUSE_MOVE_V = (UInt32)(3) mjMOUSE_MOVE_H = (UInt32)(4) mjMOUSE_ZOOM = (UInt32)(5) mjMOUSE_SELECT = (UInt32)(6)

@enum mjtPertBit mjPERT_TRANSLATE = (UInt32)(1) mjPERT_ROTATE = (UInt32)(2)

@enum mjtCamera mjCAMERA_FREE = (UInt32)(0) mjCAMERA_TRACKING = (UInt32)(1) mjCAMERA_FIXED = (UInt32)(2) mjCAMERA_USER = (UInt32)(3)

@enum mjtLabel mjLABEL_NONE = (UInt32)(0) mjLABEL_BODY = (UInt32)(1) mjLABEL_JOINT = (UInt32)(2) mjLABEL_GEOM = (UInt32)(3) mjLABEL_SITE = (UInt32)(4) mjLABEL_CAMERA = (UInt32)(5) mjLABEL_LIGHT = (UInt32)(6) mjLABEL_TENDON = (UInt32)(7) mjLABEL_ACTUATOR = (UInt32)(8) mjLABEL_CONSTRAINT = (UInt32)(9) mjLABEL_SELECTION = (UInt32)(10) mjLABEL_SELPNT = (UInt32)(11) mjLABEL_CONTACTFORCE = (UInt32)(12) mjNLABEL = (UInt32)(13)

@enum mjtFrame mjFRAME_NONE = (UInt32)(0) mjFRAME_BODY = (UInt32)(1) mjFRAME_GEOM = (UInt32)(2) mjFRAME_SITE = (UInt32)(3) mjFRAME_CAMERA = (UInt32)(4) mjFRAME_LIGHT = (UInt32)(5) mjFRAME_WORLD = (UInt32)(6) mjNFRAME = (UInt32)(7)

@enum mjtVisFlag mjVIS_CONVEXHULL = (UInt32)(0) mjVIS_TEXTURE = (UInt32)(1) mjVIS_JOINT = (UInt32)(2) mjVIS_ACTUATOR = (UInt32)(3) mjVIS_CAMERA = (UInt32)(4) mjVIS_LIGHT = (UInt32)(5) mjVIS_CONSTRAINT = (UInt32)(6) mjVIS_INERTIA = (UInt32)(7) mjVIS_PERTFORCE = (UInt32)(8) mjVIS_PERTOBJ = (UInt32)(9) mjVIS_CONTACTPOINT = (UInt32)(10) mjVIS_CONTACTFORCE = (UInt32)(11) mjVIS_CONTACTSPLIT = (UInt32)(12) mjVIS_TRANSPARENT = (UInt32)(13) mjVIS_AUTOCONNECT = (UInt32)(14) mjVIS_COM = (UInt32)(15) mjVIS_SELECT = (UInt32)(16) mjVIS_STATIC = (UInt32)(17) mjNVISFLAG = (UInt32)(18)

@enum mjtRndFlag mjRND_SHADOW = (UInt32)(0) mjRND_WIREFRAME = (UInt32)(1) mjRND_REFLECTION = (UInt32)(2) mjRND_FOG = (UInt32)(3) mjRND_SKYBOX = (UInt32)(4) mjNRNDFLAG = (UInt32)(5)

@enum mjtStereo mjSTEREO_NONE = (UInt32)(0) mjSTEREO_QUADBUFFERED = (UInt32)(1) mjSTEREO_SIDEBYSIDE = (UInt32)(2)

type _mjvPerturb
   select::Cint
   active::Cint
   refpos::SVector{3, mjtNum}
   refquat::SVector{4, mjtNum}
   localpos::SVector{3, mjtNum}
   scale::mjtNum
end

const mjvPerturb = _mjvPerturb

type _mjvCamera
   _type::Cint
   fixedcamid::Cint
   trackbodyid::Cint
   lookat::SVector{3, mjtNum}
   distance::mjtNum
   azimuth::mjtNum
   elevation::mjtNum
end

const mjvCamera = _mjvCamera

type _mjvGLCamera
   pos::SVector{3, Cfloat}
   forward::SVector{3, Cfloat}
   up::SVector{3, Cfloat}
   frustum_center::Cfloat
   frustum_bottom::Cfloat
   frustum_top::Cfloat
   frustum_near::Cfloat
   frustum_far::Cfloat
end

const mjvGLCamera = _mjvGLCamera

type _mjvGeom
   _type::Cint
   dataid::Cint
   objtype::Cint
   objid::Cint
   category::Cint
   texid::Cint
   texuniform::Cint
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

const mjvGeom = _mjvGeom

type _mjvLight
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

const mjvLight = _mjvLight

type _mjvOption
   label::Cint
   frame::Cint
   geomgroup::SVector{5, mjtByte}
   sitegroup::SVector{5, mjtByte}
   flags::SVector{18, mjtByte}
end

const mjvOption = _mjvOption

type _mjvScene
   maxgeom::Cint
   ngeom::Cint
   geoms::Ptr{mjvGeom}
   geomorder::Ptr{Cint}
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

const mjvScene = _mjvScene

type _mjvFigure
   flg_legend::Cint
   flg_ticklabel::SVector{2, Cint}
   flg_extend::Cint
   flg_barplot::Cint

   gridsize::SVector{2, Cint}
   gridrgb::SVector{3, Cfloat}
   gridwidth::Cfloat
   figurergba::SVector{4, Cfloat}
   legendrgba::SVector{4, Cfloat}
   textrgb::SVector{3, Cfloat}
   range::SMatrix{2, 2, Cfloat}
   xlabel::SVector{100, UInt8}
   title::SVector{100, UInt8}
   xformat::SVector{20, UInt8}
   yformat::SVector{20, UInt8}
   minwidth::SVector{20, UInt8}

   linepnt::SVector{mjMAXLINE, Cint}
	linergb::SMatrix{mjMAXLINE, 3, Cfloat}
   linewidth::SVector{mjMAXLINE, Cfloat}
   linedata::SMatrix{mjMAXLINE, 2*mjMAXLINEPNT, Cfloat}
   linename::SMatrix{mjMAXLINE, 100, UInt8}
end
const mjvFigure = _mjvFigure

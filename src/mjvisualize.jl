
const NGROUP = 5
const MAXOVERLAY = 500

const MAXLINE = 100
const MAXLINEPNT = 500
const MAXPLANEGRID = 100

@enum mjtCatBit CAT_STATIC = (UInt32)(1) CAT_DYNAMIC = (UInt32)(2) CAT_DECOR = (UInt32)(4) CAT_ALL = (UInt32)(7)

@enum mjtMouse MOUSE_NONE = (UInt32)(0) MOUSE_ROTATE_V = (UInt32)(1) MOUSE_ROTATE_H = (UInt32)(2) MOUSE_MOVE_V = (UInt32)(3) MOUSE_MOVE_H = (UInt32)(4) MOUSE_ZOOM = (UInt32)(5) MOUSE_SELECT = (UInt32)(6)

@enum mjtPertBit PERT_TRANSLATE = (UInt32)(1) PERT_ROTATE = (UInt32)(2)

@enum mjtCamera CAMERA_FREE = (UInt32)(0) CAMERA_TRACKING = (UInt32)(1) CAMERA_FIXED = (UInt32)(2) CAMERA_USER = (UInt32)(3)

@enum mjtLabel LABEL_NONE = (UInt32)(0) LABEL_BODY = (UInt32)(1) LABEL_JOINT = (UInt32)(2) LABEL_GEOM = (UInt32)(3) LABEL_SITE = (UInt32)(4) LABEL_CAMERA = (UInt32)(5) LABEL_LIGHT = (UInt32)(6) LABEL_TENDON = (UInt32)(7) LABEL_ACTUATOR = (UInt32)(8) LABEL_CONSTRAINT = (UInt32)(9) LABEL_SELECTION = (UInt32)(10) LABEL_SELPNT = (UInt32)(11) LABEL_CONTACTFORCE = (UInt32)(12) NLABEL = (UInt32)(13)

@enum mjtFrame FRAME_NONE = (UInt32)(0) FRAME_BODY = (UInt32)(1) FRAME_GEOM = (UInt32)(2) FRAME_SITE = (UInt32)(3) FRAME_CAMERA = (UInt32)(4) FRAME_LIGHT = (UInt32)(5) FRAME_WORLD = (UInt32)(6) NFRAME = (UInt32)(7)

@enum mjtVisFlag VIS_CONVEXHULL = (UInt32)(0) VIS_TEXTURE = (UInt32)(1) VIS_JOINT = (UInt32)(2) VIS_ACTUATOR = (UInt32)(3) VIS_CAMERA = (UInt32)(4) VIS_LIGHT = (UInt32)(5) VIS_CONSTRAINT = (UInt32)(6) VIS_INERTIA = (UInt32)(7) VIS_PERTFORCE = (UInt32)(8) VIS_PERTOBJ = (UInt32)(9) VIS_CONTACTPOINT = (UInt32)(10) VIS_CONTACTFORCE = (UInt32)(11) VIS_CONTACTSPLIT = (UInt32)(12) VIS_TRANSPARENT = (UInt32)(13) VIS_AUTOCONNECT = (UInt32)(14) VIS_COM = (UInt32)(15) VIS_SELECT = (UInt32)(16) VIS_STATIC = (UInt32)(17) NVISFLAG = (UInt32)(18)

@enum mjtRndFlag RND_SHADOW = (UInt32)(0) RND_WIREFRAME = (UInt32)(1) RND_REFLECTION = (UInt32)(2) RND_FOG = (UInt32)(3) RND_SKYBOX = (UInt32)(4) NRNDFLAG = (UInt32)(5)

@enum mjtStereo STEREO_NONE = (UInt32)(0) STEREO_QUADBUFFERED = (UInt32)(1) STEREO_SIDEBYSIDE = (UInt32)(2)

struct mjvPerturb
   select::Cint
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
   geomgroup::SVector{5, mjtByte}
   sitegroup::SVector{5, mjtByte}
   flags::SVector{18, mjtByte}
end

struct mjvScene
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

struct mjvFigure
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
end

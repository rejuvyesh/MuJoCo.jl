
@enum mjtGridPos mjGRID_TOPLEFT = (UInt32)(0) mjGRID_TOPRIGHT = (UInt32)(1) mjGRID_BOTTOMLEFT = (UInt32)(2) mjGRID_BOTTOMRIGHT = (UInt32)(3)

@enum mjtFramebuffer mjFB_WINDOW = (UInt32)(0) mjFB_OFFSCREEN = (UInt32)(1)

@enum mjtFontScale mjFONTSCALE_100 = (UInt32)(100) mjFONTSCALE_150 = (UInt32)(150) mjFONTSCALE_200 = (UInt32)(200)

@enum mjtFont mjFONT_NORMAL = (UInt32)(0) mjFONT_SHADOW = (UInt32)(1) mjFONT_BIG = (UInt32)(2)

type _mjrRect
   left::Cint
   bottom::Cint
   width::Cint
   height::Cint
end

const mjrRect = _mjrRect

immutable _mjrContext
   lineWidth::Cfloat
   shadowClip::Cfloat
   shadowScale::Cfloat
   shadowSize::Cint
   offWidth::Cint
   offHeight::Cint
   offSamples::Cint
   offFBO::UInt32
   offFBO_r::UInt32
   offColor::UInt32
   offColor_r::UInt32
   offDepthStencil::UInt32
   offDepthStencil_r::UInt32
   shadowFBO::UInt32
   shadowTex::UInt32
   ntexture::Cint
   textureType::SVector{100, Cint}
   texture::SVector{100, UInt32}
   basePlane::UInt32
   baseMesh::UInt32
   baseHField::UInt32
   baseBuiltin::UInt32
   baseFontNormal::UInt32
   baseFontShadow::UInt32
   baseFontBig::UInt32
   rangePlane::Cint
   rangeMesh::Cint
   rangeHField::Cint
   rangeBuiltin::Cint
   rangeFont::Cint
   charWidth::SVector{127, Cint}
   charWidthBig::SVector{127, Cint}
   charHeight::Cint
   charHeightBig::Cint
   glewInitialized::Cint
   windowAvailable::Cint
   windowSamples::Cint
   windowStereo::Cint
   windowDoublebuffer::Cint
   currentBuffer::Cint
end

const mjrContext = _mjrContext


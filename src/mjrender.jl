
@enum mjtGridPos GRID_TOPLEFT = (UInt32)(0) GRID_TOPRIGHT = (UInt32)(1) GRID_BOTTOMLEFT = (UInt32)(2) GRID_BOTTOMRIGHT = (UInt32)(3)

@enum mjtFramebuffer FB_WINDOW = (UInt32)(0) FB_OFFSCREEN = (UInt32)(1)

@enum mjtFontScale FONTSCALE_100 = (UInt32)(100) FONTSCALE_150 = (UInt32)(150) FONTSCALE_200 = (UInt32)(200)

@enum mjtFont FONT_NORMAL = (UInt32)(0) FONT_SHADOW = (UInt32)(1) FONT_BIG = (UInt32)(2)

immutable mjrRect
   left::Cint
   bottom::Cint
   width::Cint
   height::Cint
end

immutable mjrContext
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


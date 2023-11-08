#version 120

//Setting window number
uniform int windowNumber;

//per eye texture to warp for lens distortion
uniform sampler2D warpTexture;
uniform sampler2D warpTexture2;

//Position of lens center in m (usually eye_w/2, eye_h/2)
uniform vec2 LensCenterLeft;
uniform vec2 LensCenterLeft2;
//Position of lens center in m (usually eye_w/2, eye_h/2)
uniform vec2 LensCenterRight;
uniform vec2 LensCenterRight2;
//Scale from texture co-ords to m (usually eye_w, eye_h)
uniform vec2 ViewportScale;
//Distortion overall scale in m (usually ~eye_w/2)
uniform float WarpScale;
//Distoriton coefficients (PanoTools model) [a,b,c,d]
uniform vec4 HmdWarpParam;
uniform vec4 HmdWarpParam2;


//chromatic distortion post scaling
uniform vec3 aberr;
uniform vec3 aberr2;

void main()
{
    if (windowNumber==1){
    //output_loc is the fragment location on screen from [0,1]x[0,1]
    vec2 output_loc = gl_TexCoord[0].xy;
    vec2 LensCenter;
    float offset;
    if (output_loc[0] <= 0.5){
      offset = 0.0;
      LensCenter = LensCenterLeft;
    }
    else{
      offset = 0.5;
      LensCenter = LensCenterRight;
    }

    output_loc[0] = (output_loc[0] - offset) * 2.0;
    //Compute fragment location in lens-centered coordinates at world scale
	  vec2 r = output_loc * ViewportScale - LensCenter;
    //scale for distortion model
    //distortion model has r=1 being the largest circle inscribed (e.g. eye_w/2)
    r /= WarpScale;

    //|r|**2
    float r_mag = length(r);
    //offset for which fragment is sourced
    vec2 r_displaced = r * (HmdWarpParam.w + HmdWarpParam.z * r_mag +
    HmdWarpParam.y * r_mag * r_mag +
    HmdWarpParam.x * r_mag * r_mag * r_mag);
    //back to world scale
    r_displaced *= WarpScale;
    //back to viewport co-ord
    vec2 tc_r = (LensCenter + aberr.r * r_displaced) / ViewportScale;
    vec2 tc_g = (LensCenter + aberr.g * r_displaced) / ViewportScale;
    vec2 tc_b = (LensCenter + aberr.b * r_displaced) / ViewportScale;

    tc_r[0] = (tc_r[0] / 2.0 ) + offset;
    tc_g[0] = (tc_g[0] / 2.0 ) + offset;
    tc_b[0] = (tc_b[0] / 2.0 ) + offset;

    float red = texture2D(warpTexture, tc_r).r;
    float green = texture2D(warpTexture, tc_g).g;
    float blue = texture2D(warpTexture, tc_b).b;

    //Black edges off the texture
    gl_FragColor = ((tc_g.x - offset < 0.0) || (tc_g.x - offset > 0.5) || (tc_g.y < 0.0) || (tc_g.y > 1.0)) ? vec4(0.0, 0.0, 0.0, 1.0) : vec4(red, green, blue, 1.0);
    }



    else if (windowNumber==2){
    //output_loc is the fragment location on screen from [0,1]x[0,1]
    vec2 output_loc = gl_TexCoord[0].xy;
    vec2 LensCenter;
    float offset;
    if (output_loc[0] <= 0.5){
      offset = 0.0;
      LensCenter = LensCenterLeft;
    }
    else{
      offset = 0.5;
      LensCenter = LensCenterRight;
    }

    output_loc[0] = (output_loc[0] - offset) * 2.0;
    //Compute fragment location in lens-centered coordinates at world scale
	  vec2 r = output_loc * ViewportScale - LensCenter;
    //scale for distortion model
    //distortion model has r=1 being the largest circle inscribed (e.g. eye_w/2)
    r /= WarpScale;

    //|r|**2
    float r_mag = length(r);
    //offset for which fragment is sourced
    vec2 r_displaced = r * (HmdWarpParam.w + HmdWarpParam.z * r_mag +
    HmdWarpParam.y * r_mag * r_mag +
    HmdWarpParam.x * r_mag * r_mag * r_mag);
    //back to world scale
    r_displaced *= WarpScale;
    //back to viewport co-ord
    vec2 tc_r = (LensCenter + aberr.r * r_displaced) / ViewportScale;
    vec2 tc_g = (LensCenter + aberr.g * r_displaced) / ViewportScale;
    vec2 tc_b = (LensCenter + aberr.b * r_displaced) / ViewportScale;

    tc_r[0] = (tc_r[0] / 2.0 ) + offset;
    tc_g[0] = (tc_g[0] / 2.0 ) + offset;
    tc_b[0] = (tc_b[0] / 2.0 ) + offset;

    float red = texture2D(warpTexture2, tc_r).r;
    float green = texture2D(warpTexture2, tc_g).g;
    float blue = texture2D(warpTexture2, tc_b).b;

    //Black edges off the texture
    gl_FragColor = ((tc_g.x - offset < 0.0) || (tc_g.x - offset > 0.5) || (tc_g.y < 0.0) || (tc_g.y > 1.0)) ? vec4(0.0, 0.0, 0.0, 1.0) : vec4(red, green, blue, 1.0);
    }




    else if (windowNumber==3){
      //output_loc is the fragment location on screen from [0,1]x[0,1]
      vec2 output_loc = gl_TexCoord[0].xy;
      vec2 LensCenter;
      float offset;
      if (output_loc[0] <= 0.5){
        offset = 0.0;
        LensCenter = LensCenterLeft;
      }
      else{
        offset = 0.5;
        LensCenter = LensCenterRight;
      }

      output_loc[0] = (output_loc[0] - offset) * 2.0;
      //Compute fragment location in lens-centered coordinates at world scale
      vec2 r = output_loc * ViewportScale - LensCenter;
      //scale for distortion model
      //distortion model has r=1 being the largest circle inscribed (e.g. eye_w/2)
      r /= WarpScale;

      //|r|**2
      float r_mag = length(r);
      //offset for which fragment is sourced
      vec2 r_displaced = r * (HmdWarpParam.w + HmdWarpParam.z * r_mag +
      HmdWarpParam.y * r_mag * r_mag +
      HmdWarpParam.x * r_mag * r_mag * r_mag);
      //back to world scale
      r_displaced *= WarpScale;
      //back to viewport co-ord
      vec2 tc_r = (LensCenter + aberr.r * r_displaced) / ViewportScale;
      vec2 tc_g = (LensCenter + aberr.g * r_displaced) / ViewportScale;
      vec2 tc_b = (LensCenter + aberr.b * r_displaced) / ViewportScale;

      tc_r[0] = (tc_r[0] / 2.0 ) + offset;
      tc_g[0] = (tc_g[0] / 2.0 ) + offset;
      tc_b[0] = (tc_b[0] / 2.0 ) + offset;

      float red = texture2D(warpTexture, tc_r).r;
      float green = texture2D(warpTexture, tc_g).g;
      float blue = texture2D(warpTexture, tc_b).b;
      
      //For video 2
      vec2 output_loc2 = gl_TexCoord[0].xy;
      vec2 LensCenter2;
      float offset2;
      if (output_loc2[0] <= 0.5){
        offset2 = 0.0;
        LensCenter2 = LensCenterLeft2;
      }
      else{
        offset2 = 0.5;
        LensCenter2 = LensCenterRight2;
      }

      output_loc2[0] = (output_loc2[0] - offset2) * 2.0;
      //Compute fragment location in lens-centered coordinates at world scale
      vec2 r2 = output_loc2 * ViewportScale - LensCenter2;
      //scale for distortion model
      //distortion model has r=1 being the largest circle inscribed (e.g. eye_w/2)
      r2 /= WarpScale;

      //|r|**2
      float r_mag2 = length(r2);
      //offset for which fragment is sourced
      vec2 r_displaced2 = r2 * (HmdWarpParam2.w + HmdWarpParam2.z * r_mag2 +
      HmdWarpParam2.y * r_mag2 * r_mag2 +
      HmdWarpParam2.x * r_mag2 * r_mag2 * r_mag2);
      //back to world scale
      r_displaced2 *= WarpScale;
      //back to viewport co-ord
      vec2 tc_r2 = (LensCenter2 + aberr2.r * r_displaced2) / ViewportScale;
      vec2 tc_g2 = (LensCenter2 + aberr2.g * r_displaced2) / ViewportScale;
      vec2 tc_b2 = (LensCenter2 + aberr2.b * r_displaced2) / ViewportScale;

      tc_r2[0] = (tc_r2[0] / 2.0 ) + offset2;
      tc_g2[0] = (tc_g2[0] / 2.0 ) + offset2;
      tc_b2[0] = (tc_b2[0] / 2.0 ) + offset2;

      float red2 = texture2D(warpTexture2, tc_r2).r;
      float green2 = texture2D(warpTexture2, tc_g2).g;
      float blue2 = texture2D(warpTexture2, tc_b2).b;
      //Black edges off the texture

      if ((tc_g2.x - offset2 < 0.0) || (tc_g2.x - offset2 > 0.5) || (tc_g2.y < 0.0) || (tc_g2.y > 1.0))
      {
        if ((tc_g.x - offset2 < 0.0) || (tc_g.x - offset2 > 0.5) || (tc_g.y < 0.0) || (tc_g.y > 1.0))
          gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
        else
          gl_FragColor = vec4(red, green, blue, 1.0);
      }
      else
        gl_FragColor = vec4(red2, green2, blue2, 1.0);
    }



    else if (windowNumber==4){
      //output_loc is the fragment location on screen from [0,1]x[0,1]
      vec2 output_loc = gl_TexCoord[0].xy;
      vec2 LensCenter;
      float offset;
      if (output_loc[0] <= 0.5){
        offset = 0.0;
        LensCenter = LensCenterLeft;
      }
      else{
        offset = 0.5;
        LensCenter = LensCenterRight;
      }

      output_loc[0] = (output_loc[0] - offset) * 2.0;
      //Compute fragment location in lens-centered coordinates at world scale
      vec2 r = output_loc * ViewportScale - LensCenter;
      //scale for distortion model
      //distortion model has r=1 being the largest circle inscribed (e.g. eye_w/2)
      r /= WarpScale;

      //|r|**2
      float r_mag = length(r);
      //offset for which fragment is sourced
      vec2 r_displaced = r * (HmdWarpParam.w + HmdWarpParam.z * r_mag +
      HmdWarpParam.y * r_mag * r_mag +
      HmdWarpParam.x * r_mag * r_mag * r_mag);
      //back to world scale
      r_displaced *= WarpScale;
      //back to viewport co-ord
      vec2 tc_r = (LensCenter + aberr.r * r_displaced) / ViewportScale;
      vec2 tc_g = (LensCenter + aberr.g * r_displaced) / ViewportScale;
      vec2 tc_b = (LensCenter + aberr.b * r_displaced) / ViewportScale;

      tc_r[0] = (tc_r[0] / 2.0 ) + offset;
      tc_g[0] = (tc_g[0] / 2.0 ) + offset;
      tc_b[0] = (tc_b[0] / 2.0 ) + offset;

      float red = texture2D(warpTexture2, tc_r).r;
      float green = texture2D(warpTexture2, tc_g).g;
      float blue = texture2D(warpTexture2, tc_b).b;
      
      //For video 2
      vec2 output_loc2 = gl_TexCoord[0].xy;
      vec2 LensCenter2;
      float offset2;
      if (output_loc2[0] <= 0.5){
        offset2 = 0.0;
        LensCenter2 = LensCenterLeft2;
      }
      else{
        offset2 = 0.5;
        LensCenter2 = LensCenterRight2;
      }

      output_loc2[0] = (output_loc2[0] - offset2) * 2.0;
      //Compute fragment location in lens-centered coordinates at world scale
      vec2 r2 = output_loc2 * ViewportScale - LensCenter2;
      //scale for distortion model
      //distortion model has r=1 being the largest circle inscribed (e.g. eye_w/2)
      r2 /= WarpScale;

      //|r|**2
      float r_mag2 = length(r2);
      //offset for which fragment is sourced
      vec2 r_displaced2 = r2 * (HmdWarpParam2.w + HmdWarpParam2.z * r_mag2 +
      HmdWarpParam2.y * r_mag2 * r_mag2 +
      HmdWarpParam2.x * r_mag2 * r_mag2 * r_mag2);
      //back to world scale
      r_displaced2 *= WarpScale;
      //back to viewport co-ord
      vec2 tc_r2 = (LensCenter2 + aberr2.r * r_displaced2) / ViewportScale;
      vec2 tc_g2 = (LensCenter2 + aberr2.g * r_displaced2) / ViewportScale;
      vec2 tc_b2 = (LensCenter2 + aberr2.b * r_displaced2) / ViewportScale;

      tc_r2[0] = (tc_r2[0] / 2.0 ) + offset2;
      tc_g2[0] = (tc_g2[0] / 2.0 ) + offset2;
      tc_b2[0] = (tc_b2[0] / 2.0 ) + offset2;

      float red2 = texture2D(warpTexture, tc_r2).r;
      float green2 = texture2D(warpTexture, tc_g2).g;
      float blue2 = texture2D(warpTexture, tc_b2).b;
      //Black edges off the texture

      if ((tc_g2.x - offset2 < 0.0) || (tc_g2.x - offset2 > 0.5) || (tc_g2.y < 0.0) || (tc_g2.y > 1.0))
      {
        if ((tc_g.x - offset2 < 0.0) || (tc_g.x - offset2 > 0.5) || (tc_g.y < 0.0) || (tc_g.y > 1.0))
          gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
        else
          gl_FragColor = vec4(red, green, blue, 1.0);
      }
      else
        gl_FragColor = vec4(red2, green2, blue2, 1.0);
    }
};

//
// Implementation for Yocto/Grade.
//

//
// LICENSE:
//
// Copyright (c) 2020 -- 2020 Fabio Pellacini
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include "yocto_colorgrade.h"

#include <yocto/yocto_color.h>
#include <yocto/yocto_sampling.h>

// -----------------------------------------------------------------------------
// COLOR GRADING FUNCTIONS
// -----------------------------------------------------------------------------
namespace yocto {
int PixelsNeededForSigma(float sigma) {
  // returns the number of pixels needed to represent a gaussian kernel that has
  // values down to the threshold amount.  A gaussian function technically has
  // values everywhere on the image, but the threshold lets us cut it off where
  // the pixels contribute to only small amounts that aren't as noticeable.
  const float c_threshold = 0.05f;  // 5%
  return int(floor(
             1.0f + 2.0f * sqrtf(-2.0f * sigma * sigma * log(c_threshold)))) +
         1;
}
float Gaussian(float sigma, float x) {
  return expf(-(x * x) / (2.0f * sigma * sigma));
}
float GaussianSimpsonIntegration(float sigma, float a, float b) {
  return ((b - a) / 6.0f) *
         (Gaussian(sigma, a) + 4.0f * Gaussian(sigma, (a + b) / 2.0f) +
             Gaussian(sigma, b));
}
std::vector<float> GaussianKernelIntegrals(float sigma, int taps) {
  std::vector<float> ret;
  float              total = 0.0f;
  for (int i = 0; i < taps; ++i) {
    float x     = float(i) - float(taps / 2);
    float value = GaussianSimpsonIntegration(sigma, x - 0.5f, x + 0.5f);
    ret.push_back(value);
    total += value;
  }
  // normalize it
  for (unsigned int i = 0; i < ret.size(); ++i) {
    ret[i] /= total;
  }
  return ret;
}
const vec3f GetPixelOrBlack(const color_image image, int i, int j) {
  static const vec3f black = {0.0, 0.0, 0.0};
  if (i < 0 || i >= image.width || j < 0 || j >= image.height) {
    return black;
  }

  return xyz(get_pixel(image, i, j));
}

color_image gaussianBlur(color_image image, grade_params params) {
  /*
  sigma = quanto blur ci sta
  radius = la grandezza del kernel in pixel
  formula per calcolare kernel gaussiano = e^(-(x^2)/2*sigma^2)
  formula per calcolare grandezza kernel
  1+2 * rad(-2 * sigma^2 * ln(0.005))
  */
  int xblursize = PixelsNeededForSigma(params.xblursigma) | 1;
  int yblursize = PixelsNeededForSigma(params.yblursigma) | 1;

  // allocate space for copying the image for destImage and tmpImage
  auto tmpImage  = make_image(image.width, image.height, false);
  auto destImage = make_image(image.width, image.height, false);

  // horizontal blur from srcImage into tmpImage
  {
    auto row = GaussianKernelIntegrals(params.xblursigma, xblursize);

    int startOffset = -1 * int(row.size() / 2);
    for (auto j : range(image.height)) {
      for (auto i : range(image.width)) {
        vec3f blurredPixel = {0.0f, 0.0f, 0.0f};
        for (auto x : range(row.size())) {
          auto pixel = GetPixelOrBlack(image, j, i + startOffset + x);
          blurredPixel += pixel * row[x];
        }

        set_pixel(tmpImage, i, j, rgb_to_rgba(blurredPixel));
      }
    }
  }
  // vertical blur from tmpImage into destImage
  {
    auto row = GaussianKernelIntegrals(params.yblursigma, yblursize);

    int startOffset = -1 * int(row.size() / 2);

    for (auto j : range(image.height)) {
      for (auto i : range(image.width)) {
        vec3f blurredPixel = {0.0f, 0.0f, 0.0f};
        for (auto x : range(row.size())) {
          auto pixel = GetPixelOrBlack(tmpImage, i, j + startOffset + x);
          blurredPixel += pixel * row[x];
        }

        set_pixel(destImage, j, i, rgb_to_rgba(blurredPixel));
      }
    }
  }
  return destImage;
}

color_image boxBlur(const color_image image, float sigma) {

  auto kernelSizeEdge = PixelsNeededForSigma(sigma);
  int  startOffset    = -1 * int(kernelSizeEdge / 2);
  auto destImage      = make_image(image.width, image.height, false);
  //printf("kernel size = %d\n", kernelSizeEdge);
  for (auto y : range(image.height)) {
    for (auto x : range(image.width)) {
      vec3f blurredPixel = {0.0f, 0.0f, 0.0f};
      //vec3f blurredPixel = {0.5f,0.5f,0.5f};
      for (auto j : range(kernelSizeEdge)) {
        for (auto i : range(kernelSizeEdge)) {
          if (!(x + i + startOffset < 0 || x + i + startOffset >= image.width ||
                  y + j + startOffset < 0 ||
                  y + j + startOffset >= image.height))
            blurredPixel += xyz(
                get_pixel(image, x + i + startOffset, y + j + startOffset));
        }
      }
      blurredPixel /= kernelSizeEdge * kernelSizeEdge;
      set_pixel(destImage, x, y, rgb_to_rgba(blurredPixel));
    }
  }
  return destImage;
}
color_image deg_to_color(const color_image image, vector<vector<float>> angles, grade_params params) {
  /*
  C = (V/100)*(S/100)

  X = C*(1 – | ((H/60)mod 2 )-1 |)

  m = (V/100) – C
*/
  auto desImage = image; 
  for(auto j : range(image.height)){
    for(auto i : range(image.width))
    {
    if(get_pixel(image,i,j).x>params.edgeDetection || (params.cannyMin!=0 & params.cannyMax!=0 && get_pixel(image,i,j).x == 1))
    {
      auto H = angles[j][i]+180.0;
      float X = (1 - abs(fmod(H / 60.0, 2) - 1));
      vec3f c;
      if (H >= 0 && H < 60) {
        c.x = 1, c.y = X, c.z = 0;
      } else if (H >= 60 && H < 120) {
        c.x = X, c.y = 1, c.z = 0;

      } else if (H >= 120 && H < 180) {
        c.x = 0, c.y = 1, c.z = X;

      } else if (H >= 180 && H < 240) {
        c.x = 0, c.y = X, c.z = 1;

      } else if (H >= 240 && H < 300) {
        c.x = X, c.y = 0, c.z = 1;

      } else {
        c.x = 1, c.y = 0, c.z = X;
      }
      set_pixel(desImage, i, j, rgb_to_rgba(c));
      }
    }
  }
  return desImage;
}
color_image grayScale(color_image image) {
  for (auto y : range(image.height)) {
    for (auto x : range(image.width)) {
      auto  c    = xyz(get_pixel(image, x, y));
      float gray = (c.x * 0.2126 + c.y * 0.7152 + c.z * 0.0722);
      set_pixel(image, x, y, rgb_to_rgba({gray, gray, gray}));
    }
  }
  return image;
}

color_image nonMaxSupp(const color_image image, vector<vector<float>> angles) {
  auto destImage = image;
  for (auto y : range(1, image.height - 1)) {
    for (auto x : range(1, image.width - 1)) {
      auto Tangent = angles[y][x];
      //Horizontal edge
      if (((-22.5 < Tangent) && (Tangent <= 22.5)) ||
          ((157.5 < Tangent) && (Tangent <= -157.5))){
            if ((get_pixel(image,x,y).x < get_pixel(image,x,y+1).x ||get_pixel(image,x,y).x < get_pixel(image,x,y-1).x ))
                    set_pixel(destImage,x,y,rgb_to_rgba({0,0,0}));
          }

        // Vertical Edge
        if (((-112.5 < Tangent) && (Tangent <= -67.5)) ||
            ((67.5 < Tangent) && (Tangent <= 112.5))) {
              if ((get_pixel(image,x,y).x < get_pixel(image,x+1,y).x ||get_pixel(image,x,y).x < get_pixel(image,x-1,y).x ))
                    set_pixel(destImage,x,y,rgb_to_rgba({0,0,0}));
        }

      //-45 Degree Edge
      if (((-67.5 < Tangent) && (Tangent <= -22.5)) ||
          ((112.5 < Tangent) && (Tangent <= 157.5))) {
            if ((get_pixel(image,x,y).x < get_pixel(image,x+1,y-1).x ||get_pixel(image,x,y).x < get_pixel(image,x-1,y+1).x ))
                    set_pixel(destImage,x,y,rgb_to_rgba({0,0,0}));
      }

      // 45 Degree Edge
      if (((-157.5 < Tangent) && (Tangent <= -112.5)) ||
          ((22.5 < Tangent) && (Tangent <= 67.5))) {
            if ((get_pixel(image,x,y).x < get_pixel(image,x+1,y+1).x ||get_pixel(image,x,y).x < get_pixel(image,x-1,y-1).x ))
                    set_pixel(destImage,x,y,rgb_to_rgba({0,0,0}));
      }
    }
  }
  return destImage;
}
color_image cannyThreshold(const color_image image,int low, int high){
  auto destImage = image;
  vec4f White = rgb_to_rgba({1,1,1});
  vec4f Black = rgb_to_rgba({0,0,0});

  for (auto j : range(image.height)) {
    for (auto i : range(image.width)) {
      auto p = float_to_byte(get_pixel(image,i,j));
      if(p.x > high){
        set_pixel(destImage,i,j,White);
      }
      else if (p.x < low) 
        set_pixel(destImage,i,j,Black);
      else
      {
        bool anyHigh = false;
        bool anyBetween = false;
        for (int x=i-1; x < i+2; x++) 
        {
          for (int y = j-1; y<j+2; y++) 
          {
            if(x <= 0 || y <= 0 || x > image.width || y > image.height) //Out of bounds
                continue;
            else
            {
              p = float_to_byte(get_pixel(image,x,y));
              if( p.x > high)
              {
                set_pixel(destImage,i,j,White);
                anyHigh = true;
                break;
              }
              else if(p.x <= high && p.x >= low)
                anyBetween = true;
            }
          }
          if(anyHigh)
            break;
        }
        if(!anyHigh && anyBetween)
            for (int x=i-2; x < i+3; x++) 
            {
                for (int y = j-2; y<j+3; y++) 
                {
                  if(x <= 0 || y <= 0 || x > image.width || y > image.height) //Out of bounds
                    continue;
                  else
                  {
                    p = float_to_byte(get_pixel(image,x,y));
                     if( p.x > high)
                    {
                      set_pixel(destImage,i,j,White);
                      anyHigh = true;
                      break;
                    }
                  }
                }
                if(anyHigh)
                    break;
            }
        if(!anyHigh)
          set_pixel(destImage,i,j,Black);
      }
    }
  }
  return destImage;
}
color_image cannyEdgeDetection(
    const color_image image, const grade_params params) {
  const int kernelSize                             = 3;
  int       verticalKernel[kernelSize][kernelSize] = {
      {-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}};
  int horizontalKernel[kernelSize][kernelSize] = {
      {-1, -2, -1}, {0, 0, 0}, {1, 2, 1}};
  auto  startOffset = -1;
  auto  blurImage   = boxBlur(image, 0.3);
  auto  grayImage   = grayScale(blurImage);
  auto  destImage   = make_image(image.width, image.height, false);
  vector<vector<float>> angles(image.height,vector<float> (image.width,0));

  for (auto y : range(1, image.height - 1)) {
    for (auto x : range(1, image.width - 1)) {
      auto Gx = zero3f;
      auto Gy = zero3f;
      for (auto j : range(kernelSize)) {
        for (auto i : range(kernelSize)) {
          if (!(x + i + startOffset < 0 || x + i + startOffset >= image.width ||
                  y + j + startOffset < 0 ||
                  y + j + startOffset >= image.height)) {
            auto p = xyz(
                get_pixel(grayImage, x + i + startOffset, y + j + startOffset));

            Gx += p * horizontalKernel[j][i];
            Gy += p * verticalKernel[j][i];
          }
        }
      }
      auto G = sqrt(pow(Gx, 2) + pow(Gy, 2));
      // printf("G = %f\n", G.x);
      auto direction = degrees(std::atan2(Gy.x, Gx.x));
      angles[y][x] = direction;
      if (G.x > 1) G.x = 1;

        set_pixel(destImage, x, y, rgb_to_rgba(G));
    }
  }
  if(params.cannyMin!=0 && params.cannyMax != 0 && params.cannyMin<=255 && params.cannyMax <=255) {
    destImage = nonMaxSupp(destImage,angles);
    destImage = cannyThreshold(destImage,params.cannyMin,params.cannyMax);
  }
  if(params.edgeColor)
    destImage = deg_to_color(destImage,angles,params); 
  return destImage;
}

color_image mosaicGrid(color_image image, grade_params params) {
  auto graded = make_image(image.width, image.height, false);
  for (auto j : range(image.height)) {
    for (auto i : range(image.width)) {
      auto newPixel = xyz(get_pixel(image, i, j));

      // mosaic
      if (params.mosaic != 0) {
        newPixel = xyz(
            get_pixel(image, i - (i % params.mosaic), j - (j % params.mosaic)));
      }
      // grid
      if (params.grid != 0) {
        if (i % params.grid == 0 || j % params.grid == 0)
          operator*=(newPixel, 0.5);
      }
      set_pixel(graded, i, j, rgb_to_rgba(newPixel));
    }
  }
  return graded;
}

color_image applyFilters(color_image image, grade_params params) {
  auto graded = make_image(image.width, image.height, false);
  for (auto j : range(image.height)) {
    for (auto i : range(image.width)) {
      auto newPixel = xyz(get_pixel(image, i, j));
      set_pixel(graded, i, j, rgb_to_rgba(newPixel));
      // Tone mapping
      /*
    expoure compensioation: c = c * 2^exposure
    filmic correction: c *= 0.6; c = (c^2 * 2.51 + c * 0.03) / (c^2 * 2.43 +
    c * 0.59 + 0.14) che è un fit del tonemapping cinematografico ACES srgb
    color space: c = c ^ 1/2.2 clamp result: c = clamp(c, 0, 1) con clamp(c,
    m, M) = max(min(c, M), m)
      */
      newPixel *= pow(2, params.exposure);

      // prima parentesi
      if (params.filmic) {
        newPixel *= 0.6;
        newPixel = operator/(
            operator+(operator*(pow(newPixel, 2), 2.51), operator*(
                                                             newPixel, 0.03)),

            // seconda parentesi
            operator+(operator+(operator*(pow(newPixel, 2), 2.43), operator*(
                                                                       newPixel,
                                                                       0.59)),
                0.14));
      }
      newPixel = pow(newPixel, 1.0 / 2.2);
      clamp(newPixel, 0, 1);

      // colorTint
      newPixel = operator*(newPixel, params.tint);

      // saturation c = g + (c - g) * (saturation * 2)
      if (params.saturation != 0.5f) {
        auto           g = ((newPixel.x + newPixel.y + newPixel.z) / 3);
        newPixel = g + operator*(operator-(newPixel, g), params.saturation * 2);
      }
      // contrast
      if (params.contrast != 0.5f)
        newPixel = gain(newPixel, 1 - params.contrast);

      // vignette
      if (params.vignette != 0.0f) {
        auto vr = 1 - params.vignette;
        // r = length(ij - size/2) / length(size/2)
        auto ij   = vec2f{static_cast<float>(i), static_cast<float>(j)};
        auto size = vec2f{
            static_cast<float>(image.width), static_cast<float>(image.height)};
        auto r = length(ij - operator/(size, 2)) / length(operator/(size, 2));
        // c = c * (1 - smoothstep(vr, 2 * vr, r))
        newPixel = operator*(newPixel, (1 - smoothstep(vr, 2 * vr, r)));
      }
      // film grain c = c + (rand1f(rng) - 0.5) * grain
      u_int64_t  seed = random();
      rng_state  rng  = make_rng(seed);
      newPixel        = operator+(newPixel, (rand1f(rng) - 0.5) * params.grain);

      set_pixel(graded, i, j, rgb_to_rgba(newPixel));
    }
  }

  return graded;
}

color_image grade_image(const color_image &image, const grade_params &params) {
  // PUT YOUR CODE HERE
  // con const passo gli oggetti per referenza, non per puntatore, così
  // evito di avere copie superflue
  /*
  for (auto& pixel : image.pixels) {  // sto iterando per referenza
  }
  */
  auto graded = applyFilters(image, params);
  // if (params.mosaic && params.grid != 0)
  graded = mosaicGrid(graded, params);

  if (params.xblursigma != 0 || params.yblursigma != 0)
    graded = gaussianBlur(graded, params);
  if (params.boxblur != 0)
    for (auto x : range(3)) graded = boxBlur(graded, params.boxblur);
  if (params.edgeDetection != 0) graded = cannyEdgeDetection(image, params);
  return graded;
}

}  // namespace yocto
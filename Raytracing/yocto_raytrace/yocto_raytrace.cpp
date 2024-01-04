//
// Implementation for Yocto/RayTrace.
//

//
// LICENSE:
//
// Copyright (c) 2016 -- 2021 Fabio Pellacini
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "yocto_raytrace.h"

#include <yocto/yocto_cli.h>
#include <yocto/yocto_geometry.h>
#include <yocto/yocto_parallel.h>
#include <yocto/yocto_sampling.h>
#include <yocto/yocto_shading.h>
#include <yocto/yocto_shape.h>

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR SCENE EVALUATION
// -----------------------------------------------------------------------------
namespace yocto {

// Generates a ray from a camera for yimg::image plane coordinate uv and
// the lens coordinates luv.
static ray3f eval_camera(const camera_data& camera, const vec2f& uv) {
  auto film = camera.aspect >= 1
                  ? vec2f{camera.film, camera.film / camera.aspect}
                  : vec2f{camera.film * camera.aspect, camera.film};
  auto q    = transform_point(camera.frame,
      {film.x * (0.5f - uv.x), film.y * (uv.y - 0.5f), camera.lens});
  auto e    = transform_point(camera.frame, {0, 0, 0});
  return {e, normalize(e - q)};
}

}  // namespace yocto

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR PATH TRACING
// -----------------------------------------------------------------------------
namespace yocto {

// Raytrace renderer.
static vec4f shade_raytrace(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  // YOUR CODE GOES HERE ----
  auto isec = intersect_bvh(bvh, scene, ray, false, true);

  if (!isec.hit) {
    return rgb_to_rgba(eval_environment(scene, ray.d));
  }
  auto& istance  = scene.instances[isec.instance];
  auto  material = eval_material(scene, istance, isec.element, isec.uv);
  auto& shape    = scene.shapes[istance.shape];
  auto  normal   = transform_direction(
      istance.frame, eval_normal(shape, isec.element, isec.uv));
  auto position = transform_point(
      istance.frame, eval_position(shape, isec.element, isec.uv));
  auto radiance = material.emission;

  auto color = rgb_to_rgba(material.color);

  // formula pazza
  if (!shape.points.empty()) {
    normal = -ray.d;
  } else if (!shape.lines.empty()) {
    normal = orthonormalize(-ray.d, normal);
  } else if (!shape.triangles.empty()) {
    if (dot(-ray.d, normal) < 0) {
      normal = -normal;
    }
  }

  // opacity
  if (rand1f(rng) < 1 - material.opacity) {
    return shade_raytrace(
        scene, bvh, ray3f{position, ray.d}, bounce + 1, rng, params);
  }

  if (bounce >= params.bounces) return rgb_to_rgba(radiance);
  switch (material.type) {
    case material_type::matte: {
      auto incoming = sample_hemisphere_cos(normal, rand2f(rng));
      radiance += rgba_to_rgb(
          color * shade_raytrace(scene, bvh, ray3f{position, incoming},
                      bounce + 1, rng, params));
      break;
    }
    case material_type::reflective: {
      // polished
      if (!material.roughness) {
        auto incoming = reflect(-ray.d, normal);
        radiance += fresnel_schlick(rgba_to_rgb(color), normal, -ray.d) *
                    rgba_to_rgb(shade_raytrace(scene, bvh,
                        ray3f{position, incoming}, bounce + 1, rng, params));
      } else {
        auto halfway = sample_hemisphere_cospower(
            4 / (material.roughness * material.roughness), normal, rand2f(rng));
        auto incoming = reflect(-ray.d, halfway);
        radiance += fresnel_schlick(rgba_to_rgb(color), halfway, -ray.d) *
                    rgba_to_rgb(shade_raytrace(scene, bvh,
                        ray3f{position, -incoming}, bounce + 1, rng, params));
      }
      break;
    }
    case material_type::glossy: {
      auto exponent = 4 / (material.roughness * material.roughness);
      auto mnormal  = sample_hemisphere_cospower(exponent, normal, rand2f(rng));
      auto fresnel_value = fresnel_schlick({0.04, 0.04, 0.04}, mnormal, -ray.d);
      if (rand1f(rng) < fresnel_value.x) {
        auto incoming = reflect(-ray.d, mnormal);
        radiance += rgba_to_rgb(shade_raytrace(
            scene, bvh, ray3f{position, incoming}, bounce + 1, rng, params));
      } else {
        auto incoming = sample_hemisphere_cos(mnormal, rand2f(rng));
        radiance += rgba_to_rgb(
            color * shade_raytrace(scene, bvh, ray3f{position, incoming},
                        bounce + 1, rng, params));
      }
      break;
    }
    case material_type::transparent: {
      if (rand1f(rng) < fresnel_schlick({0.04, 0.04, 0.04}, normal, -ray.d).x) {
        auto incoming = reflect(-ray.d, normal);
        radiance += rgba_to_rgb(shade_raytrace(
            scene, bvh, ray3f{position, incoming}, bounce + 1, rng, params));
      } else {
        auto incoming = ray.d;
        radiance += rgba_to_rgb(
            color * shade_raytrace(scene, bvh, ray3f{position, incoming},
                        bounce + 1, rng, params));
      }
      break;
    }
    default: {
      break;
    }
  }

  return rgb_to_rgba(radiance);
}

// Matte renderer.
static vec4f shade_matte(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  // YOUR CODE GOES HERE ----
  return {0, 0, 0, 0};
}

// Eyelight renderer.
static vec4f shade_eyelight(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto point = intersect_bvh(bvh, scene, ray, false, true);
  if (!point.hit) return {0, 0, 0, 0};
  auto& istance  = scene.instances[point.instance];
  auto& material = scene.materials[istance.material];
  auto& shape    = scene.shapes[istance.shape];
  auto  normal   = transform_direction(
      istance.frame, eval_normal(shape, point.element, point.uv));
  return rgb_to_rgba(material.color * dot(normal, -ray.d));
}

static vec4f shade_normal(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto point = intersect_bvh(bvh, scene, ray, false, true);
  if (!point.hit)
    return {0, 0, 0, 0};
  else {
    auto& istance   = scene.instances[point.instance];
    auto& materials = scene.materials[istance.material];
    auto  v         = eval_normalmap(scene, istance, point.element, point.uv);
    v *= 0.5;
    v += 0.5;
    return rgb_to_rgba(v);
  }
}

static vec4f shade_texcoord(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto point = intersect_bvh(bvh, scene, ray, false, true);
  if (!point.hit)
    return {0, 0, 0, 0};
  else {
    auto& istance = scene.instances[point.instance];
    auto& shape   = scene.shapes[istance.shape];
    auto  element = point.element;
    auto  uv      = point.uv;
    auto  coord   = eval_texcoord(shape, element, uv);
    return {fmod(coord.x, 1), fmod(coord.y, 1), 0, 1};
  }
}

static vec4f shade_color(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto point = intersect_bvh(bvh, scene, ray, false, true);
  if (!point.hit)
    return {0, 0, 0, 0};
  else {
    auto& istance   = scene.instances[point.instance];
    auto& materials = scene.materials[istance.material];
    return rgb_to_rgba(materials.color);
  }
}

static bool shadow_testing(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& shadowRay, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto isec = intersect_bvh(bvh, scene, shadowRay, false, true);

  if (!isec.hit) {
    return false;
  }
  auto& instance = scene.instances[isec.instance];
  auto  material = eval_material(scene, instance, isec.element, isec.uv);
  auto  radiance = material.emission;

  if ((!radiance.x > 0) || (!radiance.y > 0) || (!radiance.z > 0)) return true;
  return false;
}
// FARE QUA
static vec4f shade_toon(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto isec = intersect_bvh(bvh, scene, ray, false, true);

  if (!isec.hit) {
    return rgb_to_rgba(eval_environment(scene, ray.d));
  }

  auto& instance = scene.instances[isec.instance];
  auto  material = eval_material(scene, instance, isec.element, isec.uv);
  auto& shape    = scene.shapes[instance.shape];
  auto  normal   = transform_direction(
      instance.frame, eval_normal(shape, isec.element, isec.uv));
  auto position = transform_point(
      instance.frame, eval_position(shape, isec.element, isec.uv));
  // aggiungo tutte le fonti di luce
  instance_data lights;
  auto          color  = material.color;
  double        shadow = 1;

  for (int i = 0; i < scene.instances.size(); i++) {
    auto& lightInstance   = scene.instances[i];
    auto& currentMaterial = scene.materials[lightInstance.material];
    if (currentMaterial.emission.x > 0 || currentMaterial.emission.y > 0 ||
        currentMaterial.emission.z > 0) {
      lights = lightInstance;
    }
  }
  auto   lightDir = transform_direction(instance.frame, lights.frame.o);
  double NdotL    = dot(normal, lightDir);
  // controllo se è shadow, una volta che è 0, non cambia più
  if (shadow_testing(
          scene, bvh, ray3f{position, lightDir}, bounce, rng, params) == true ||
      shadow == 0)
    shadow = 0;

  auto   lightIntensity          = smoothstep(0.0001, 0.15, NdotL * shadow);
  auto   halfVector              = normalize(lightDir + (-ray.d));
  auto   NdotH                   = dot(normal, halfVector);
  auto   specularIntensity       = pow(NdotH * lightIntensity, 32 * 32);
  auto   specularIntensitySmooth = smoothstep(0.005, 0.01, specularIntensity);
  double rimDot                  = 1 - dot(-ray.d, normal);
  auto   rimAmount               = 0.716;
  double rimIntensity            = rimDot * pow(NdotL, 0.1);
  rimIntensity = smoothstep(rimAmount - 0.01, rimAmount + 0.01, rimIntensity);

  return rgb_to_rgba(
      color * (lightIntensity + 0.4 + specularIntensitySmooth + rimIntensity));
}

// Trace a single ray from the camera using the given algorithm.
using raytrace_shader_func = vec4f (*)(const scene_data& scene,
    const bvh_scene& bvh, const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params);
static raytrace_shader_func get_shader(const raytrace_params& params) {
  switch (params.shader) {
    case raytrace_shader_type::raytrace: return shade_raytrace;
    case raytrace_shader_type::matte: return shade_matte;
    case raytrace_shader_type::eyelight: return shade_eyelight;
    case raytrace_shader_type::normal: return shade_normal;
    case raytrace_shader_type::texcoord: return shade_texcoord;
    case raytrace_shader_type::color: return shade_color;
    case raytrace_shader_type::toon: return shade_toon;
    default: {
      throw std::runtime_error("sampler unknown");
      return nullptr;
    }
  }
}

// Build the bvh acceleration structure.
bvh_scene make_bvh(const scene_data& scene, const raytrace_params& params) {
  return make_bvh(scene, false, false, params.noparallel);
}

// Init a sequence of random number generators.
raytrace_state make_state(
    const scene_data& scene, const raytrace_params& params) {
  auto& camera = scene.cameras[params.camera];
  auto  state  = raytrace_state{};
  if (camera.aspect >= 1) {
    state.width  = params.resolution;
    state.height = (int)round(params.resolution / camera.aspect);
  } else {
    state.height = params.resolution;
    state.width  = (int)round(params.resolution * camera.aspect);
  }
  state.samples = 0;
  state.image.assign(state.width * state.height, {0, 0, 0, 0});
  state.hits.assign(state.width * state.height, 0);
  state.rngs.assign(state.width * state.height, {});
  auto rng_ = make_rng(1301081);
  for (auto& rng : state.rngs) {
    rng = make_rng(961748941ull, rand1i(rng_, 1 << 31) / 2 + 1);
  }
  return state;
}

// Progressively compute an image by calling trace_samples multiple times.
void raytrace_samples(raytrace_state& state, const scene_data& scene,
    const bvh_scene& bvh, const raytrace_params& params) {
  if (state.samples >= params.samples) return;
  auto& camera = scene.cameras[params.camera];
  auto  shader = get_shader(params);
  state.samples += 1;
  if (params.samples == 1) {
    for (auto idx = 0; idx < state.width * state.height; idx++) {
      auto i = idx % state.width, j = idx / state.width;
      auto u = (i + 0.5f) / state.width, v = (j + 0.5f) / state.height;
      auto ray      = eval_camera(camera, {u, v});
      auto radiance = shader(scene, bvh, ray, 0, state.rngs[idx], params);
      if (!isfinite(radiance)) radiance = {0, 0, 0};
      state.image[idx] += radiance;
      state.hits[idx] += 1;
    }
  } else if (params.noparallel) {
    for (auto idx = 0; idx < state.width * state.height; idx++) {
      auto i = idx % state.width, j = idx / state.width;
      auto u        = (i + rand1f(state.rngs[idx])) / state.width,
           v        = (j + rand1f(state.rngs[idx])) / state.height;
      auto ray      = eval_camera(camera, {u, v});
      auto radiance = shader(scene, bvh, ray, 0, state.rngs[idx], params);
      if (!isfinite(radiance)) radiance = {0, 0, 0};
      state.image[idx] += radiance;
      state.hits[idx] += 1;
    }
  } else {
    parallel_for(state.width * state.height, [&](int idx) {
      auto i = idx % state.width, j = idx / state.width;
      auto u        = (i + rand1f(state.rngs[idx])) / state.width,
           v        = (j + rand1f(state.rngs[idx])) / state.height;
      auto ray      = eval_camera(camera, {u, v});
      auto radiance = shader(scene, bvh, ray, 0, state.rngs[idx], params);
      if (!isfinite(radiance)) radiance = {0, 0, 0};
      state.image[idx] += radiance;
      state.hits[idx] += 1;
    });
  }
}

// Check image type
static void check_image(
    const color_image& image, int width, int height, bool linear) {
  if (image.width != width || image.height != height)
    throw std::invalid_argument{"image should have the same size"};
  if (image.linear != linear)
    throw std::invalid_argument{
        linear ? "expected linear image" : "expected srgb image"};
}

// Get resulting render
color_image get_render(const raytrace_state& state) {
  auto image = make_image(state.width, state.height, true);
  get_render(image, state);
  return image;
}
void get_render(color_image& image, const raytrace_state& state) {
  check_image(image, state.width, state.height, true);
  auto scale = 1.0f / (float)state.samples;
  for (auto idx = 0; idx < state.width * state.height; idx++) {
    image.pixels[idx] = state.image[idx] * scale;
  }
}

}  // namespace yocto

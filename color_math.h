#pragma once
#include <math.h>

struct XYZ { float X, Y, Z; };
struct xy  { float x, y; };
struct Lab { float L, a, b; };

inline xy xyz_to_xy(const XYZ& v) {
  float sum = v.X + v.Y + v.Z;
  if (sum <= 0.000001f) return {0, 0};
  return { v.X / sum, v.Y / sum };
}

// McCamy approximation (fast embedded CCT from x,y)
inline float cct_from_xy_mccamy(const xy& p) {
  // guard
  float denom = (0.1858f - p.y);
  if (fabsf(denom) < 1e-6f) return NAN;

  float n = (p.x - 0.3320f) / denom;
  float n2 = n * n;
  float n3 = n2 * n;
  return 449.0f*n3 + 3525.0f*n2 + 6823.3f*n + 5520.33f;
}

inline float lab_f(float t) {
  const float d = 6.0f/29.0f;
  const float d3 = d*d*d;
  if (t > d3) return cbrtf(t);
  return (t/(3.0f*d*d)) + (4.0f/29.0f);
}

inline Lab xyz_to_lab(const XYZ& xyz, const XYZ& whiteRef) {
  // guard
  if (whiteRef.X <= 0 || whiteRef.Y <= 0 || whiteRef.Z <= 0) return {NAN, NAN, NAN};

  float xr = xyz.X / whiteRef.X;
  float yr = xyz.Y / whiteRef.Y;
  float zr = xyz.Z / whiteRef.Z;

  float fx = lab_f(xr);
  float fy = lab_f(yr);
  float fz = lab_f(zr);

  Lab out;
  out.L = 116.0f * fy - 16.0f;
  out.a = 500.0f * (fx - fy);
  out.b = 200.0f * (fy - fz);
  return out;
}

#pragma once
#include "color_math.h"

struct Cal2PointXYZ {
  bool  valid = false;
  XYZ   offset = {0,0,0}; // raw black
  XYZ   gain   = {1,1,1}; // refWhite / (rawWhite-rawBlack)
  XYZ   refWhiteSpectro = {1,1,1}; // save for Lab reference

  XYZ apply(const XYZ& raw) const {
    XYZ out;
    out.X = (raw.X - offset.X) * gain.X;
    out.Y = (raw.Y - offset.Y) * gain.Y;
    out.Z = (raw.Z - offset.Z) * gain.Z;
    return out;
  }
};

inline Cal2PointXYZ make_cal_2pt_xyz(
  const XYZ& rawBlack,
  const XYZ& rawWhite,
  const XYZ& refWhiteSpectro
) {
  Cal2PointXYZ cal;
  cal.offset = rawBlack;
  cal.refWhiteSpectro = refWhiteSpectro;

  // Avoid divide by ~0
  float dx = rawWhite.X - rawBlack.X;
  float dy = rawWhite.Y - rawBlack.Y;
  float dz = rawWhite.Z - rawBlack.Z;

  const float eps = 1e-6f;
  if (fabsf(dx) < eps || fabsf(dy) < eps || fabsf(dz) < eps) {
    cal.valid = false;
    return cal;
  }

  cal.gain = {
    refWhiteSpectro.X / dx,
    refWhiteSpectro.Y / dy,
    refWhiteSpectro.Z / dz
  };

  cal.valid = true;
  return cal;
}

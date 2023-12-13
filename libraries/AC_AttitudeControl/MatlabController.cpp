//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.cpp
//
// Code generated for Simulink model 'ArduCopter_Fast_Descent'.
//
// Model version                  : 1.468
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Tue Oct 10 17:35:11 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM 7
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "MatlabController.h"

// Exported data definition

// Const memory section
// Definition for custom storage class: Const
const logConfigBus log_config[5] = { {
    14U,

    { 117U, 49U, 1U, 117U, 50U, 1U, 117U, 51U, 1U, 117U, 52U, 1U, 101U, 120U,
      103U, 101U, 121U, 103U, 101U, 122U, 103U, 101U, 117U, 103U, 101U, 118U,
      103U, 101U, 119U, 103U, 101U, 97U, 120U, 101U, 97U, 121U, 101U, 97U, 122U,
      104U, 49U, 1U },

    { 77U, 76U, 49U, 0U }
  }, { 13U,
    { 104U, 50U, 1U, 120U, 103U, 114U, 121U, 103U, 114U, 122U, 103U, 114U, 120U,
      103U, 109U, 121U, 103U, 109U, 122U, 103U, 109U, 120U, 100U, 49U, 121U,
      100U, 49U, 122U, 100U, 49U, 120U, 100U, 50U, 121U, 100U, 50U, 122U, 100U,
      50U, 0U, 0U, 0U },

    { 77U, 76U, 50U, 0U }
  }, { 11U,
    { 68U, 118U, 49U, 68U, 118U, 50U, 68U, 118U, 51U, 68U, 118U, 52U, 112U, 1U,
      1U, 113U, 1U, 1U, 114U, 1U, 1U, 119U, 49U, 1U, 119U, 50U, 1U, 119U, 51U,
      1U, 119U, 52U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U },

    { 77U, 76U, 51U, 0U }
  }, { 12U,
    { 113U, 49U, 1U, 113U, 50U, 1U, 113U, 51U, 1U, 113U, 52U, 1U, 112U, 100U,
      116U, 113U, 100U, 116U, 114U, 100U, 116U, 97U, 102U, 49U, 97U, 102U, 50U,
      97U, 102U, 51U, 97U, 102U, 52U, 97U, 102U, 53U, 0U, 0U, 0U, 0U, 0U, 0U },

    { 77U, 76U, 52U, 0U }
  }, { 8U,
    { 119U, 112U, 117U, 105U, 115U, 103U, 116U, 115U, 49U, 116U, 115U, 50U, 116U,
      115U, 51U, 116U, 115U, 52U, 116U, 115U, 53U, 116U, 115U, 54U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U },

    { 77U, 76U, 53U, 0U }
  } } ;

extern real32_T rt_roundf(real32_T u);
extern real_T rt_urand_Upu32_Yd_f_pw(uint32_T *u);
extern real_T rt_nrand_Upu32_Yd_f_pw(uint32_T *u);
extern real32_T rt_hypotf(real32_T u0, real32_T u1);
static real32_T look1_iflf_binlx(real32_T u0, const real32_T bp0[], const
  real32_T table[], uint32_T maxIndex);
static void Throttlefinished(boolean_T rtu_Enable, real32_T *rty_throttle);
static void DCM2LeanVector(const real32_T rtu_M_bg[9], real32_T rty_n_g[3]);
static void nrefnorm(const real32_T rtu_y_dt2[3], const real32_T rtu_y_dt[3],
                     const real32_T rtu_y[3], real32_T rty_n_dt2[3], real32_T
                     rty_n_dt[3], real32_T rty_n[3]);
static void DCMtoquaternions(const real32_T rtu_M_bg[9], real32_T rty_q_bg[4]);
static void QuaternionReduced(const real32_T rtu_q_bg[4], real32_T rty_q_red[4],
  real32_T *rty_yaw);
static void wrapangle(real32_T rtu_angle, real32_T *rty_angle_0_2pi);
static void LeanVectorDerivativeTrafo(const real32_T rtu_n_g[3], const real32_T
  rtu_n_g_dt[3], const real32_T rtu_n_g_dt2[3], const real32_T rtu_M_bg[9],
  const real32_T rtu_omega_Kb[3], const real32_T rtu_omega_Kb_dt[3], real32_T
  rty_n_b[3], real32_T rty_n_b_dt[3], real32_T rty_n_b_dt2[3]);

// Forward declaration for local functions
static void leanVectorNormDeriv2(const real32_T nn[3], const real32_T nn_dt[3],
  const real32_T nn_dt2[3], real_T n_dt2[3]);

// Forward declaration for local functions
static void quatNormalize(const real32_T q[4], real32_T q_out[4]);
static real32_T look1_iflf_binlx(real32_T u0, const real32_T bp0[], const
  real32_T table[], uint32_T maxIndex)
{
  real32_T frac;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;

  // Column-major Lookup 1-D
  // Search method: 'binary'
  // Use previous index: 'off'
  // Interpolation method: 'Linear point-slope'
  // Extrapolation method: 'Linear'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'off'

  // Prelookup - Index and Fraction
  // Index Search method: 'binary'
  // Extrapolation method: 'Linear'
  // Use previous index: 'off'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'off'

  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    // Binary Search
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  // Column-major Interpolation 1-D
  // Interpolation method: 'Linear point-slope'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Overflow mode: 'wrapping'

  return (table[iLeft + 1U] - table[iLeft]) * frac + table[iLeft];
}

//
// Output and update for enable system:
//    '<S11>/Throttle finished'
//    '<S11>/Throttle slowing'
//
static void Throttlefinished(boolean_T rtu_Enable, real32_T *rty_throttle)
{
  // Outputs for Enabled SubSystem: '<S11>/Throttle finished' incorporates:
  //   EnablePort: '<S22>/Enable'

  if (rtu_Enable) {
    // SignalConversion: '<S22>/OutportBuffer_InsertedFor_throttle_at_inport_0' incorporates:
    //   Constant: '<S22>/throttle finished'

    *rty_throttle = 0.0F;
  }

  // End of Outputs for SubSystem: '<S11>/Throttle finished'
}

//
// Output and update for atomic system:
//    '<S33>/DCM 2 Lean Vector'
//    '<S34>/MATLAB Function'
//
static void DCM2LeanVector(const real32_T rtu_M_bg[9], real32_T rty_n_g[3])
{
  int32_T i;
  for (i = 0; i < 3; i++) {
    rty_n_g[i] = 0.0F;
    rty_n_g[i] += -rtu_M_bg[3 * i + 2];
  }
}

// Function for MATLAB Function: '<S89>/n ref norm'
static void leanVectorNormDeriv2(const real32_T nn[3], const real32_T nn_dt[3],
  const real32_T nn_dt2[3], real_T n_dt2[3])
{
  real32_T xyz;
  real32_T a_tmp;
  real32_T n_dt2_tmp;
  real32_T xyz_tmp;
  real32_T xyz_tmp_0;
  real32_T xyz_tmp_1;
  real32_T a_tmp_tmp;
  real32_T a_tmp_tmp_0;
  real32_T a_tmp_tmp_1;
  real32_T n_dt2_tmp_0;
  real32_T n_dt2_tmp_1;
  real32_T n_dt2_tmp_2;
  real32_T n_dt2_tmp_3;
  real32_T n_dt2_tmp_4;
  real32_T n_dt2_tmp_5;
  xyz_tmp = nn[1] * nn[1];
  xyz_tmp_0 = nn[0] * nn[0];
  xyz_tmp_1 = nn[2] * nn[2];
  xyz = (xyz_tmp_0 + xyz_tmp) + xyz_tmp_1;
  a_tmp_tmp = 2.0F * nn[1] * nn_dt[1];
  a_tmp_tmp_0 = 2.0F * nn[0] * nn_dt[0];
  a_tmp_tmp_1 = 2.0F * nn[2] * nn_dt[2];
  a_tmp = (a_tmp_tmp_0 + a_tmp_tmp) + a_tmp_tmp_1;
  n_dt2_tmp = std::pow(xyz, 1.5F);
  n_dt2_tmp_0 = 2.0F * nn[1] * nn_dt2[1];
  n_dt2_tmp_1 = nn_dt[1] * nn_dt[1] * 2.0F;
  n_dt2_tmp_2 = 2.0F * nn[0] * nn_dt2[0];
  n_dt2_tmp_3 = nn_dt[0] * nn_dt[0] * 2.0F;
  n_dt2_tmp_4 = 2.0F * nn[2] * nn_dt2[2];
  n_dt2_tmp_5 = nn_dt[2] * nn_dt[2] * 2.0F;
  n_dt2[0] = (a_tmp * a_tmp * 3.0F / (4.0F * std::pow(xyz, 2.5F)) -
              (((((n_dt2_tmp_2 + n_dt2_tmp_3) + n_dt2_tmp_0) + n_dt2_tmp_1) +
                n_dt2_tmp_4) + n_dt2_tmp_5) / (2.0F * n_dt2_tmp)) * nn[0] +
    (nn_dt2[0] / std::sqrt(xyz) - a_tmp * nn_dt[0] / n_dt2_tmp);
  xyz = (xyz_tmp + xyz_tmp_0) + xyz_tmp_1;
  a_tmp = (a_tmp_tmp + a_tmp_tmp_0) + a_tmp_tmp_1;
  n_dt2_tmp = std::pow(xyz, 1.5F);
  n_dt2[1] = (a_tmp * a_tmp * 3.0F / (4.0F * std::pow(xyz, 2.5F)) -
              (((((n_dt2_tmp_0 + n_dt2_tmp_1) + n_dt2_tmp_2) + n_dt2_tmp_3) +
                n_dt2_tmp_4) + n_dt2_tmp_5) / (2.0F * n_dt2_tmp)) * nn[1] +
    (nn_dt2[1] / std::sqrt(xyz) - a_tmp * nn_dt[1] / n_dt2_tmp);
  xyz = (xyz_tmp_1 + xyz_tmp_0) + xyz_tmp;
  a_tmp = (a_tmp_tmp_1 + a_tmp_tmp_0) + a_tmp_tmp;
  n_dt2_tmp = std::pow(xyz, 1.5F);
  n_dt2[2] = (a_tmp * a_tmp * 3.0F / (4.0F * std::pow(xyz, 2.5F)) -
              (((((n_dt2_tmp_4 + n_dt2_tmp_5) + n_dt2_tmp_2) + n_dt2_tmp_3) +
                n_dt2_tmp_0) + n_dt2_tmp_1) / (2.0F * n_dt2_tmp)) * nn[2] +
    (nn_dt2[2] / std::sqrt(xyz) - a_tmp * nn_dt[2] / n_dt2_tmp);
}

//
// Output and update for atomic system:
//    '<S89>/n ref norm'
//    '<S122>/n ref norm'
//
static void nrefnorm(const real32_T rtu_y_dt2[3], const real32_T rtu_y_dt[3],
                     const real32_T rtu_y[3], real32_T rty_n_dt2[3], real32_T
                     rty_n_dt[3], real32_T rty_n[3])
{
  real32_T norm_n;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  real_T tmp[3];
  real32_T scale_0[9];
  int32_T i;
  real32_T norm_n_tmp;
  real32_T scale_tmp;
  real32_T norm_n_tmp_0;
  real32_T tmp_0;
  real32_T scale_tmp_tmp;
  scale = 1.29246971E-26F;
  absxk = std::abs(rtu_y[0]);
  if (absxk > 1.29246971E-26F) {
    norm_n = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    norm_n = t * t;
  }

  absxk = std::abs(rtu_y[1]);
  if (absxk > scale) {
    t = scale / absxk;
    norm_n = norm_n * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    norm_n += t * t;
  }

  absxk = std::abs(rtu_y[2]);
  if (absxk > scale) {
    t = scale / absxk;
    norm_n = norm_n * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    norm_n += t * t;
  }

  norm_n = scale * std::sqrt(norm_n);
  if (norm_n < 2.22044605E-16F) {
    norm_n = 2.22044605E-16F;
  }

  rty_n[0] = rtu_y[0] / norm_n;
  rty_n[1] = rtu_y[1] / norm_n;
  rty_n[2] = rtu_y[2] / norm_n;
  norm_n_tmp = rtu_y[1] * rtu_y[1];
  absxk = rtu_y[2] * rtu_y[2];
  t = rtu_y[0] * rtu_y[0];
  norm_n_tmp_0 = t + norm_n_tmp;
  norm_n = std::pow(norm_n_tmp_0 + absxk, 1.5F);
  scale = norm_n;
  tmp_0 = std::abs(norm_n);
  if (tmp_0 < 2.22044605E-16F) {
    scale = 2.22044605E-16F;
  }

  scale = 1.0F / scale;
  scale_0[0] = (norm_n_tmp + absxk) * scale;
  norm_n_tmp = -rtu_y[0] * rtu_y[1] * scale;
  scale_0[3] = norm_n_tmp;
  scale_tmp_tmp = -rtu_y[0] * rtu_y[2];
  scale_tmp = scale_tmp_tmp * scale;
  scale_0[6] = scale_tmp;
  scale_0[1] = norm_n_tmp;
  scale_0[4] = (t + absxk) * scale;
  norm_n_tmp = -rtu_y[1] * rtu_y[2] * scale;
  scale_0[7] = norm_n_tmp;
  scale_0[2] = scale_tmp;
  scale_0[5] = norm_n_tmp;
  scale_0[8] = norm_n_tmp_0 * scale;
  for (i = 0; i < 3; i++) {
    rty_n_dt[i] = 0.0F;
    rty_n_dt[i] += scale_0[i] * rtu_y_dt[0];
    rty_n_dt[i] += scale_0[i + 3] * rtu_y_dt[1];
    rty_n_dt[i] += scale_0[i + 6] * rtu_y_dt[2];
  }

  if (tmp_0 < 2.22044605E-16F) {
    norm_n = 2.22044605E-16F;
  }

  rty_n_dt[2] = ((scale_tmp_tmp * rtu_y_dt[0] + t * rtu_y_dt[2]) + (rtu_y[1] *
    rtu_y_dt[2] - rtu_y[2] * rtu_y_dt[1]) * rtu_y[1]) * (1.0F / norm_n);
  leanVectorNormDeriv2(rtu_y, rtu_y_dt, rtu_y_dt2, tmp);
  rty_n_dt2[0] = (real32_T)tmp[0];
  rty_n_dt2[1] = (real32_T)tmp[1];
  rty_n_dt2[2] = (real32_T)tmp[2];
}

//
// Output and update for atomic system:
//    '<S99>/DCM to quaternions'
//    '<S104>/DCM to quaternions'
//
static void DCMtoquaternions(const real32_T rtu_M_bg[9], real32_T rty_q_bg[4])
{
  real32_T q_0;
  real32_T q_1;
  real32_T q_2;
  real32_T q_3;
  real32_T ex;
  int32_T idx;
  q_0 = ((1.0F + rtu_M_bg[0]) + rtu_M_bg[4]) + rtu_M_bg[8];
  if (0.0F >= q_0) {
    q_0 = 0.0F;
  }

  q_0 = 0.5F * std::sqrt(q_0);
  q_1 = ((1.0F + rtu_M_bg[0]) - rtu_M_bg[4]) - rtu_M_bg[8];
  if (0.0F >= q_1) {
    q_1 = 0.0F;
  }

  q_1 = 0.5F * std::sqrt(q_1);
  q_2 = ((1.0F - rtu_M_bg[0]) + rtu_M_bg[4]) - rtu_M_bg[8];
  if (0.0F >= q_2) {
    q_2 = 0.0F;
  }

  q_2 = 0.5F * std::sqrt(q_2);
  q_3 = ((1.0F - rtu_M_bg[0]) - rtu_M_bg[4]) + rtu_M_bg[8];
  if (0.0F >= q_3) {
    q_3 = 0.0F;
  }

  q_3 = 0.5F * std::sqrt(q_3);
  ex = q_0;
  idx = -1;
  if (q_0 < q_1) {
    ex = q_1;
    idx = 0;
  }

  if (ex < q_2) {
    ex = q_2;
    idx = 1;
  }

  if (ex < q_3) {
    idx = 2;
  }

  switch (idx + 1) {
   case 0:
    ex = rtu_M_bg[7] - rtu_M_bg[5];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else {
      if (ex > 0.0F) {
        ex = 1.0F;
      }
    }

    q_1 *= ex;
    ex = rtu_M_bg[2] - rtu_M_bg[6];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else {
      if (ex > 0.0F) {
        ex = 1.0F;
      }
    }

    q_2 *= ex;
    ex = rtu_M_bg[3] - rtu_M_bg[1];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else {
      if (ex > 0.0F) {
        ex = 1.0F;
      }
    }

    q_3 *= ex;
    break;

   case 1:
    ex = rtu_M_bg[7] - rtu_M_bg[5];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else {
      if (ex > 0.0F) {
        ex = 1.0F;
      }
    }

    q_0 *= ex;
    ex = rtu_M_bg[3] + rtu_M_bg[1];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else {
      if (ex > 0.0F) {
        ex = 1.0F;
      }
    }

    q_2 *= ex;
    ex = rtu_M_bg[2] + rtu_M_bg[6];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else {
      if (ex > 0.0F) {
        ex = 1.0F;
      }
    }

    q_3 *= ex;
    break;

   case 2:
    ex = rtu_M_bg[2] - rtu_M_bg[6];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else {
      if (ex > 0.0F) {
        ex = 1.0F;
      }
    }

    q_0 *= ex;
    ex = rtu_M_bg[3] + rtu_M_bg[1];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else {
      if (ex > 0.0F) {
        ex = 1.0F;
      }
    }

    q_1 *= ex;
    ex = rtu_M_bg[7] + rtu_M_bg[5];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else {
      if (ex > 0.0F) {
        ex = 1.0F;
      }
    }

    q_3 *= ex;
    break;

   case 3:
    ex = rtu_M_bg[3] - rtu_M_bg[1];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else {
      if (ex > 0.0F) {
        ex = 1.0F;
      }
    }

    q_0 *= ex;
    ex = rtu_M_bg[2] + rtu_M_bg[6];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else {
      if (ex > 0.0F) {
        ex = 1.0F;
      }
    }

    q_1 *= ex;
    ex = rtu_M_bg[7] + rtu_M_bg[5];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else {
      if (ex > 0.0F) {
        ex = 1.0F;
      }
    }

    q_2 *= ex;
    break;
  }

  rty_q_bg[0] = q_0;
  rty_q_bg[1] = q_1;
  rty_q_bg[2] = q_2;
  rty_q_bg[3] = q_3;
  q_1 = 1.29246971E-26F;
  q_2 = std::abs(rty_q_bg[0]);
  if (q_2 > 1.29246971E-26F) {
    q_0 = 1.0F;
    q_1 = q_2;
  } else {
    q_3 = q_2 / 1.29246971E-26F;
    q_0 = q_3 * q_3;
  }

  q_2 = std::abs(rty_q_bg[1]);
  if (q_2 > q_1) {
    q_3 = q_1 / q_2;
    q_0 = q_0 * q_3 * q_3 + 1.0F;
    q_1 = q_2;
  } else {
    q_3 = q_2 / q_1;
    q_0 += q_3 * q_3;
  }

  q_2 = std::abs(rty_q_bg[2]);
  if (q_2 > q_1) {
    q_3 = q_1 / q_2;
    q_0 = q_0 * q_3 * q_3 + 1.0F;
    q_1 = q_2;
  } else {
    q_3 = q_2 / q_1;
    q_0 += q_3 * q_3;
  }

  q_2 = std::abs(rty_q_bg[3]);
  if (q_2 > q_1) {
    q_3 = q_1 / q_2;
    q_0 = q_0 * q_3 * q_3 + 1.0F;
    q_1 = q_2;
  } else {
    q_3 = q_2 / q_1;
    q_0 += q_3 * q_3;
  }

  q_0 = q_1 * std::sqrt(q_0);
  if (2.22044605E-16F >= q_0) {
    q_0 = 2.22044605E-16F;
  }

  rty_q_bg[0] /= q_0;
  rty_q_bg[1] /= q_0;
  rty_q_bg[2] /= q_0;
  rty_q_bg[3] /= q_0;
}

// Function for MATLAB Function: '<S99>/Quaternion Reduced'
static void quatNormalize(const real32_T q[4], real32_T q_out[4])
{
  real32_T y;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  scale = 1.29246971E-26F;
  absxk = std::abs(q[0]);
  if (absxk > 1.29246971E-26F) {
    y = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    y = t * t;
  }

  absxk = std::abs(q[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = std::abs(q[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = std::abs(q[3]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  y = scale * std::sqrt(y);
  if (2.22044605E-16F >= y) {
    y = 2.22044605E-16F;
  }

  q_out[0] = q[0] / y;
  q_out[1] = q[1] / y;
  q_out[2] = q[2] / y;
  q_out[3] = q[3] / y;
}

//
// Output and update for atomic system:
//    '<S99>/Quaternion Reduced'
//    '<S104>/Quaternion Reduced'
//
static void QuaternionReduced(const real32_T rtu_q_bg[4], real32_T rty_q_red[4],
  real32_T *rty_yaw)
{
  real32_T q_yaw[4];
  real32_T q1_q1;
  real32_T q_yaw_0[4];
  real32_T M_bg_idx_2;
  real32_T M_bg_idx_8;
  int32_T tmp;
  quatNormalize(rtu_q_bg, q_yaw);
  M_bg_idx_2 = (q_yaw[1] * q_yaw[3] + q_yaw[0] * q_yaw[2]) * 2.0F;
  M_bg_idx_8 = ((q_yaw[0] * q_yaw[0] - q_yaw[1] * q_yaw[1]) - q_yaw[2] * q_yaw[2])
    + q_yaw[3] * q_yaw[3];
  if (1.0F <= M_bg_idx_8) {
    M_bg_idx_8 = 1.0F;
  }

  if (-1.0F >= M_bg_idx_8) {
    M_bg_idx_8 = -1.0F;
  }

  M_bg_idx_8 = std::acos(M_bg_idx_8);
  q1_q1 = std::sin(M_bg_idx_8);
  q1_q1 = q1_q1 * q1_q1 - M_bg_idx_2 * M_bg_idx_2;
  if ((q_yaw[2] * q_yaw[3] - q_yaw[0] * q_yaw[1]) * 2.0F >= 0.0F) {
    tmp = -1;
  } else {
    tmp = 1;
  }

  if (0.0F >= q1_q1) {
    q1_q1 = 0.0F;
  }

  q1_q1 = std::atan2((real32_T)tmp * std::sqrt(q1_q1), -M_bg_idx_2);
  M_bg_idx_2 = std::sin(M_bg_idx_8 / 2.0F);
  rty_q_red[0] = std::cos(M_bg_idx_8 / 2.0F);
  rty_q_red[1] = std::sin(q1_q1) * M_bg_idx_2;
  rty_q_red[2] = -std::cos(q1_q1) * M_bg_idx_2;
  rty_q_red[3] = 0.0F;
  M_bg_idx_8 = ((rty_q_red[0] * rty_q_red[0] + rty_q_red[1] * rty_q_red[1]) +
                rty_q_red[2] * rty_q_red[2]) + rty_q_red[3] * rty_q_red[3];
  if (2.22044605E-16F >= M_bg_idx_8) {
    M_bg_idx_8 = 2.22044605E-16F;
  }

  q_yaw[0] = rty_q_red[0] / M_bg_idx_8;
  q_yaw[1] = -rty_q_red[1] / M_bg_idx_8;
  q_yaw[2] = -rty_q_red[2] / M_bg_idx_8;
  q_yaw[3] = -rty_q_red[3] / M_bg_idx_8;
  q_yaw_0[0] = ((q_yaw[0] * rtu_q_bg[0] - q_yaw[1] * rtu_q_bg[1]) - q_yaw[2] *
                rtu_q_bg[2]) - q_yaw[3] * rtu_q_bg[3];
  q_yaw_0[1] = (q_yaw[0] * rtu_q_bg[1] + rtu_q_bg[0] * q_yaw[1]) + (q_yaw[2] *
    rtu_q_bg[3] - q_yaw[3] * rtu_q_bg[2]);
  q_yaw_0[2] = (q_yaw[0] * rtu_q_bg[2] + rtu_q_bg[0] * q_yaw[2]) + (q_yaw[3] *
    rtu_q_bg[1] - q_yaw[1] * rtu_q_bg[3]);
  q_yaw_0[3] = (q_yaw[0] * rtu_q_bg[3] + rtu_q_bg[0] * q_yaw[3]) + (q_yaw[1] *
    rtu_q_bg[2] - q_yaw[2] * rtu_q_bg[1]);
  quatNormalize(q_yaw_0, q_yaw);
  if (q_yaw[3] < 0.0F) {
    M_bg_idx_8 = -1.0F;
  } else if (q_yaw[3] > 0.0F) {
    M_bg_idx_8 = 1.0F;
  } else {
    M_bg_idx_8 = q_yaw[3];
  }

  if (M_bg_idx_8 >= 0.0F) {
    if (1.0F > q_yaw[0]) {
      M_bg_idx_8 = q_yaw[0];
    } else {
      M_bg_idx_8 = 1.0F;
    }

    if (-1.0F >= M_bg_idx_8) {
      M_bg_idx_8 = -1.0F;
    }

    *rty_yaw = 2.0F * std::acos(M_bg_idx_8);
  } else {
    if (1.0F > -q_yaw[0]) {
      M_bg_idx_8 = -q_yaw[0];
    } else {
      M_bg_idx_8 = 1.0F;
    }

    if (-1.0F >= M_bg_idx_8) {
      M_bg_idx_8 = -1.0F;
    }

    *rty_yaw = 2.0F * std::acos(M_bg_idx_8);
  }
}

//
// Output and update for atomic system:
//    '<S103>/wrap angle'
//    '<S103>/wrap angle1'
//
static void wrapangle(real32_T rtu_angle, real32_T *rty_angle_0_2pi)
{
  real32_T x;
  x = std::abs(rtu_angle);
  x -= std::floor(x / 6.28318548F) * 6.28318548F;
  if (rtu_angle >= 0.0F) {
    *rty_angle_0_2pi = x;
  } else {
    *rty_angle_0_2pi = 6.28318548F - x;
  }
}

//
// Output and update for atomic system:
//    '<S108>/Lean Vector Derivative Trafo'
//    '<S108>/Lean Vector Derivative Trafo Delay'
//
static void LeanVectorDerivativeTrafo(const real32_T rtu_n_g[3], const real32_T
  rtu_n_g_dt[3], const real32_T rtu_n_g_dt2[3], const real32_T rtu_M_bg[9],
  const real32_T rtu_omega_Kb[3], const real32_T rtu_omega_Kb_dt[3], real32_T
  rty_n_b[3], real32_T rty_n_b_dt[3], real32_T rty_n_b_dt2[3])
{
  real32_T rtu_omega_Kb_0[3];
  real32_T rtu_omega_Kb_dt_0[3];
  real32_T tmp[9];
  real32_T rtu_M_bg_0[9];
  int32_T i;
  int32_T i_0;
  real32_T tmp_0;
  int32_T rtu_M_bg_tmp;
  int32_T rtu_M_bg_tmp_0;
  for (i = 0; i < 3; i++) {
    rty_n_b[i] = 0.0F;
    rty_n_b[i] += rtu_M_bg[i] * rtu_n_g[0];
    rty_n_b[i] += rtu_M_bg[i + 3] * rtu_n_g[1];
    rty_n_b[i] += rtu_M_bg[i + 6] * rtu_n_g[2];
  }

  rtu_omega_Kb_0[0] = -(rtu_omega_Kb[1] * rty_n_b[2] - rtu_omega_Kb[2] *
                        rty_n_b[1]);
  rtu_omega_Kb_0[1] = -(rtu_omega_Kb[2] * rty_n_b[0] - rtu_omega_Kb[0] *
                        rty_n_b[2]);
  rtu_omega_Kb_0[2] = -(rtu_omega_Kb[0] * rty_n_b[1] - rtu_omega_Kb[1] *
                        rty_n_b[0]);
  for (i = 0; i < 3; i++) {
    rty_n_b_dt[i] = rtu_omega_Kb_0[i] + (rtu_M_bg[i + 6] * rtu_n_g_dt[2] +
      (rtu_M_bg[i + 3] * rtu_n_g_dt[1] + rtu_M_bg[i] * rtu_n_g_dt[0]));
  }

  rtu_omega_Kb_dt_0[0] = -(rtu_omega_Kb_dt[1] * rty_n_b[2] - rtu_omega_Kb_dt[2] *
    rty_n_b[1]);
  rtu_omega_Kb_dt_0[1] = -(rtu_omega_Kb_dt[2] * rty_n_b[0] - rtu_omega_Kb_dt[0] *
    rty_n_b[2]);
  rtu_omega_Kb_dt_0[2] = -(rtu_omega_Kb_dt[0] * rty_n_b[1] - rtu_omega_Kb_dt[1] *
    rty_n_b[0]);
  rtu_omega_Kb_0[0] = rtu_omega_Kb[1] * rty_n_b_dt[2] - rtu_omega_Kb[2] *
    rty_n_b_dt[1];
  rtu_omega_Kb_0[1] = rtu_omega_Kb[2] * rty_n_b_dt[0] - rtu_omega_Kb[0] *
    rty_n_b_dt[2];
  rtu_omega_Kb_0[2] = rtu_omega_Kb[0] * rty_n_b_dt[1] - rtu_omega_Kb[1] *
    rty_n_b_dt[0];
  tmp[0] = 0.0F;
  tmp[3] = -rtu_omega_Kb[2];
  tmp[6] = rtu_omega_Kb[1];
  tmp[1] = rtu_omega_Kb[2];
  tmp[4] = 0.0F;
  tmp[7] = -rtu_omega_Kb[0];
  tmp[2] = -rtu_omega_Kb[1];
  tmp[5] = rtu_omega_Kb[0];
  tmp[8] = 0.0F;
  for (i = 0; i < 3; i++) {
    tmp_0 = 0.0F;
    for (i_0 = 0; i_0 < 3; i_0++) {
      rtu_M_bg_tmp = i + 3 * i_0;
      rtu_M_bg_0[rtu_M_bg_tmp] = 0.0F;
      rtu_M_bg_tmp_0 = 3 * i_0 + i;
      rtu_M_bg_0[rtu_M_bg_tmp] = rtu_M_bg_0[rtu_M_bg_tmp_0] + rtu_M_bg[3 * i_0] *
        tmp[3 * i];
      rtu_M_bg_0[rtu_M_bg_tmp] = rtu_M_bg[3 * i_0 + 1] * tmp[3 * i + 1] +
        rtu_M_bg_0[rtu_M_bg_tmp_0];
      rtu_M_bg_0[rtu_M_bg_tmp] = rtu_M_bg[3 * i_0 + 2] * tmp[3 * i + 2] +
        rtu_M_bg_0[rtu_M_bg_tmp_0];
      tmp_0 += rtu_M_bg_0[rtu_M_bg_tmp_0] * rtu_n_g_dt[i_0];
    }

    rty_n_b_dt2[i] = ((rtu_omega_Kb_dt_0[i] - rtu_omega_Kb_0[i]) + tmp_0) +
      (rtu_M_bg[i + 6] * rtu_n_g_dt2[2] + (rtu_M_bg[i + 3] * rtu_n_g_dt2[1] +
        rtu_M_bg[i] * rtu_n_g_dt2[0]));
  }
}

real32_T rt_roundf(real32_T u)
{
  real32_T y;
  if (std::abs(u) < 8.388608E+6F) {
    if (u >= 0.5F) {
      y = std::floor(u + 0.5F);
    } else if (u > -0.5F) {
      y = 0.0F;
    } else {
      y = std::ceil(u - 0.5F);
    }
  } else {
    y = u;
  }

  return y;
}

// Function for MATLAB Function: '<S75>/trajFromWaypoints'
void MatlabControllerClass::trajSectionGetPos(const real32_T traj_section_pos_x
  [6], const real32_T traj_section_pos_y[6], const real32_T traj_section_pos_z[6],
  real_T varargin_1, real32_T pos[3])
{
  real32_T px;
  real32_T py;
  real32_T pz;
  int32_T i;
  px = traj_section_pos_x[0];
  py = traj_section_pos_y[0];
  pz = traj_section_pos_z[0];
  for (i = 0; i < 5; i++) {
    px = (real32_T)varargin_1 * px + traj_section_pos_x[i + 1];
    py = (real32_T)varargin_1 * py + traj_section_pos_y[i + 1];
    pz = (real32_T)varargin_1 * pz + traj_section_pos_z[i + 1];
  }

  pos[0] = px;
  pos[1] = py;
  pos[2] = pz;
}

// Function for MATLAB Function: '<S75>/trajFromWaypoints'
void MatlabControllerClass::polyder_p(const real32_T u[6], real32_T a_data[],
  int32_T a_size[2])
{
  int32_T nlead0;
  int32_T b_k;
  nlead0 = 0;
  b_k = 0;
  while ((b_k < 4) && (u[b_k] == 0.0F)) {
    nlead0++;
    b_k++;
  }

  a_size[0] = 1;
  a_size[1] = 5 - nlead0;
  for (b_k = 0; b_k <= 4 - nlead0; b_k++) {
    a_data[b_k] = u[b_k + nlead0];
  }

  nlead0 = a_size[1] - 2;
  for (b_k = 0; b_k <= nlead0; b_k++) {
    a_data[b_k] *= (real32_T)((nlead0 - b_k) + 1) + 1.0F;
  }
}

// Function for MATLAB Function: '<S75>/trajFromWaypoints'
real32_T MatlabControllerClass::polyVal(const real32_T p_data[], const int32_T
  p_size[2], real_T x)
{
  real32_T y;
  int32_T i;
  y = 0.0F;
  if (p_size[1] > 0) {
    y = p_data[0];
  }

  for (i = 0; i <= p_size[1] - 2; i++) {
    y = (real32_T)x * y + p_data[i + 1];
  }

  return y;
}

// Function for MATLAB Function: '<S75>/trajFromWaypoints'
void MatlabControllerClass::trajSetArcLength(dtoSgl_trajectoryStructBus *traj)
{
  real_T total_arc_length;
  real_T total_distance;
  real32_T distance;
  int32_T k;
  real_T section_idx;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  int32_T b_k;
  real32_T dx_data[5];
  real32_T dy_data[5];
  real32_T dz_data[5];
  real32_T d_a;
  real32_T e_a;
  real32_T f_a;
  real32_T l_a;
  real32_T tmp[3];
  real32_T tmp_0[3];
  int32_T dx_size[2];
  int32_T dy_size[2];
  int32_T dz_size[2];
  total_arc_length = 0.0;
  total_distance = 0.0;
  for (k = 0; k < (int32_T)traj->num_sections_set; k++) {
    section_idx = 1.0F + (real32_T)k;
    if (1.0F + (real32_T)k > traj->num_sections_set) {
      section_idx = 1.0;
    }

    trajSectionGetPos(traj->sections[(int32_T)section_idx - 1].pos_x,
                      traj->sections[(int32_T)section_idx - 1].pos_y,
                      traj->sections[(int32_T)section_idx - 1].pos_z, 1.0, tmp);
    trajSectionGetPos(traj->sections[(int32_T)section_idx - 1].pos_x,
                      traj->sections[(int32_T)section_idx - 1].pos_y,
                      traj->sections[(int32_T)section_idx - 1].pos_z, 0.0, tmp_0);
    scale = 1.29246971E-26F;
    absxk = std::abs(tmp[0] - tmp_0[0]);
    if (absxk > 1.29246971E-26F) {
      distance = 1.0F;
      scale = absxk;
    } else {
      t = absxk / 1.29246971E-26F;
      distance = t * t;
    }

    absxk = std::abs(tmp[1] - tmp_0[1]);
    if (absxk > scale) {
      t = scale / absxk;
      distance = distance * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      distance += t * t;
    }

    absxk = std::abs(tmp[2] - tmp_0[2]);
    if (absxk > scale) {
      t = scale / absxk;
      distance = distance * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      distance += t * t;
    }

    distance = scale * std::sqrt(distance);
    polyder_p(traj->sections[(int32_T)section_idx - 1].pos_x, dx_data, dx_size);
    polyder_p(traj->sections[(int32_T)section_idx - 1].pos_y, dy_data, dy_size);
    polyder_p(traj->sections[(int32_T)section_idx - 1].pos_z, dz_data, dz_size);
    section_idx = 0.0;
    scale = polyVal(dx_data, dx_size, 0.0);
    absxk = polyVal(dy_data, dy_size, 0.0);
    t = polyVal(dz_data, dz_size, 0.0);
    d_a = polyVal(dx_data, dx_size, 1.0);
    e_a = polyVal(dy_data, dy_size, 1.0);
    f_a = polyVal(dz_data, dz_size, 1.0);
    scale = std::sqrt((scale * scale + absxk * absxk) + t * t) * 0.5F - std::
      sqrt((d_a * d_a + e_a * e_a) + f_a * f_a) * 0.5F;
    for (b_k = 0; b_k < 15; b_k++) {
      section_idx += 0.066666666666666666;
      absxk = polyVal(dx_data, dx_size, section_idx - 0.033333333333333333);
      t = polyVal(dy_data, dy_size, section_idx - 0.033333333333333333);
      d_a = polyVal(dz_data, dz_size, section_idx - 0.033333333333333333);
      e_a = polyVal(dx_data, dx_size, section_idx);
      f_a = polyVal(dy_data, dy_size, section_idx);
      l_a = polyVal(dz_data, dz_size, section_idx);
      scale = (std::sqrt((absxk * absxk + t * t) + d_a * d_a) * 2.0F + scale) +
        std::sqrt((e_a * e_a + f_a * f_a) + l_a * l_a);
    }

    scale *= 0.0222222228F;
    b_k = (int32_T)(1.0F + (real32_T)k) - 1;
    traj->sections[b_k].arc_length = scale;
    traj->sections[b_k].distance = distance;
    total_arc_length = (real32_T)total_arc_length + scale;
    total_distance = (real32_T)total_distance + distance;
  }

  traj->distance = (real32_T)total_distance;
  traj->arc_length = (real32_T)total_arc_length;
}

// Function for MATLAB Function: '<S75>/trajFromWaypoints'
void MatlabControllerClass::polyder(const real_T u_data[], const int32_T u_size
  [2], real_T a_data[], int32_T a_size[2])
{
  int32_T nlead0;
  int32_T ny;
  int32_T c_k;
  nlead0 = 0;
  ny = 0;
  while ((ny <= u_size[1] - 3) && (u_data[ny] == 0.0)) {
    nlead0++;
    ny++;
  }

  ny = (u_size[1] - nlead0) - 1;
  a_size[0] = 1;
  a_size[1] = ny;
  for (c_k = 0; c_k < ny; c_k++) {
    a_data[c_k] = u_data[c_k + nlead0];
  }

  nlead0 = ny - 2;
  for (ny = 0; ny <= nlead0; ny++) {
    a_data[ny] *= (real_T)((nlead0 - ny) + 1) + 1.0;
  }
}

// Function for MATLAB Function: '<S75>/trajFromWaypoints'
void MatlabControllerClass::polyInterpolationAx_f(real32_T num_of_splines,
  boolean_T cycle, const real32_T x_data[], const int32_T *x_size, real32_T
  b_data[], int32_T *b_size)
{
  real_T pp[36];
  real_T point_0[36];
  real32_T intermediate_size;
  int32_T bnd_left;
  int32_T l;
  int32_T n_data[60];
  int32_T w_data[57];
  real_T pp_data[6];
  real_T tmp_data[5];
  real32_T b[3];
  real32_T b_data_0[60];
  real32_T x[6];
  real32_T b_0[4];
  real32_T b_data_1[57];
  real32_T point_0_0;
  int32_T loop_ub;
  int32_T pp_size[2];
  int32_T tmp_size[2];
  int8_T c_x_idx_0;
  int32_T tmp;
  int8_T c_idx_0;
  real32_T intermediate_size_tmp;
  int32_T n_size_idx_1_tmp;
  int32_T b_data_tmp;
  int32_T j_tmp;
  c_x_idx_0 = (int8_T)*x_size;
  c_idx_0 = (int8_T)*x_size;
  *b_size = c_x_idx_0;
  if (0 <= c_x_idx_0 - 1) {
    memset(&b_data[0], 0, c_x_idx_0 * sizeof(real32_T));
  }

  for (b_data_tmp = 0; b_data_tmp < 36; b_data_tmp++) {
    pp[b_data_tmp] = 1.0;
  }

  for (b_data_tmp = 0; b_data_tmp < 6; b_data_tmp++) {
    for (j_tmp = 0; j_tmp < 5; j_tmp++) {
      pp[(j_tmp + 6 * b_data_tmp) + 1] = 0.0;
    }
  }

  for (bnd_left = 0; bnd_left < 5; bnd_left++) {
    loop_ub = 6 - bnd_left;
    pp_size[0] = 1;
    pp_size[1] = loop_ub;
    for (b_data_tmp = 0; b_data_tmp < loop_ub; b_data_tmp++) {
      pp_data[b_data_tmp] = pp[6 * b_data_tmp + bnd_left];
    }

    polyder(pp_data, pp_size, tmp_data, tmp_size);
    loop_ub = tmp_size[1];
    for (b_data_tmp = 0; b_data_tmp < loop_ub; b_data_tmp++) {
      pp[(bnd_left + 6 * b_data_tmp) + 1] = tmp_data[b_data_tmp];
    }
  }

  memcpy(&point_0[0], &pp[0], 36U * sizeof(real_T));
  for (bnd_left = 0; bnd_left < 5; bnd_left++) {
    tmp = 5 - bnd_left;
    for (b_data_tmp = 0; b_data_tmp < tmp; b_data_tmp++) {
      point_0[bnd_left + 6 * b_data_tmp] = 0.0;
    }
  }

  intermediate_size_tmp = (num_of_splines - 1.0F) * 6.0F;
  if (!cycle) {
    bnd_left = 3;
    *b_size = c_idx_0;
    if (0 <= c_idx_0 - 1) {
      memset(&b_data[0], 0, c_idx_0 * sizeof(real32_T));
    }

    for (b_data_tmp = 0; b_data_tmp < 6; b_data_tmp++) {
      x[b_data_tmp] = x_data[b_data_tmp];
    }

    for (b_data_tmp = 0; b_data_tmp < 3; b_data_tmp++) {
      point_0_0 = 0.0F;
      for (j_tmp = 0; j_tmp < 6; j_tmp++) {
        point_0_0 += (real32_T)point_0[6 * j_tmp + b_data_tmp] * x[j_tmp];
      }

      b_data[b_data_tmp] = point_0_0;
    }

    if (4.0F + intermediate_size_tmp > ((4.0F + intermediate_size_tmp) + 3.0F) -
        1.0F) {
      tmp = 0;
      j_tmp = 0;
      loop_ub = 0;
      l = 0;
    } else {
      tmp = (int32_T)(4.0F + intermediate_size_tmp) - 1;
      j_tmp = (int32_T)(((4.0F + intermediate_size_tmp) + 3.0F) - 1.0F);
      loop_ub = tmp;
      l = j_tmp;
    }

    n_size_idx_1_tmp = l - loop_ub;
    for (b_data_tmp = 0; b_data_tmp < n_size_idx_1_tmp; b_data_tmp++) {
      n_data[b_data_tmp] = loop_ub + b_data_tmp;
    }

    loop_ub = j_tmp - tmp;
    for (b_data_tmp = 0; b_data_tmp < loop_ub; b_data_tmp++) {
      b_data_0[b_data_tmp] = b_data[tmp + b_data_tmp];
    }

    for (b_data_tmp = 0; b_data_tmp < 6; b_data_tmp++) {
      x[b_data_tmp] = x_data[(int32_T)((1.0F + intermediate_size_tmp) +
        (real32_T)b_data_tmp) - 1];
    }

    for (b_data_tmp = 0; b_data_tmp < 3; b_data_tmp++) {
      point_0_0 = 0.0F;
      for (j_tmp = 0; j_tmp < 6; j_tmp++) {
        point_0_0 += (real32_T)pp[6 * j_tmp + b_data_tmp] * x[j_tmp];
      }

      b[b_data_tmp] = b_data_0[b_data_tmp] + point_0_0;
    }

    for (b_data_tmp = 0; b_data_tmp < n_size_idx_1_tmp; b_data_tmp++) {
      b_data[n_data[b_data_tmp]] = b[b_data_tmp];
    }
  } else {
    bnd_left = 2;
    point_0_0 = 0.0F;
    for (b_data_tmp = 0; b_data_tmp < 6; b_data_tmp++) {
      point_0_0 += (real32_T)point_0[6 * b_data_tmp] * x_data[b_data_tmp];
    }

    b_data[0] = point_0_0;
    point_0_0 = 0.0F;
    for (b_data_tmp = 0; b_data_tmp < 6; b_data_tmp++) {
      point_0_0 += x_data[(int32_T)((intermediate_size_tmp + 1.0F) + (real32_T)
        b_data_tmp) - 1] * (real32_T)pp[6 * b_data_tmp];
    }

    b_data[1] = point_0_0;
    if ((intermediate_size_tmp + 2.0F) + 1.0F > (((intermediate_size_tmp + 2.0F)
          + 1.0F) + 5.0F) - 2.0F) {
      tmp = 0;
      j_tmp = 0;
      loop_ub = 0;
      l = 0;
    } else {
      tmp = (int32_T)((intermediate_size_tmp + 2.0F) + 1.0F) - 1;
      j_tmp = (int32_T)((((intermediate_size_tmp + 2.0F) + 1.0F) + 5.0F) - 2.0F);
      loop_ub = tmp;
      l = j_tmp;
    }

    n_size_idx_1_tmp = l - loop_ub;
    for (b_data_tmp = 0; b_data_tmp < n_size_idx_1_tmp; b_data_tmp++) {
      n_data[b_data_tmp] = loop_ub + b_data_tmp;
    }

    loop_ub = j_tmp - tmp;
    for (b_data_tmp = 0; b_data_tmp < loop_ub; b_data_tmp++) {
      b_data_0[b_data_tmp] = b_data[tmp + b_data_tmp];
    }

    for (b_data_tmp = 0; b_data_tmp < 6; b_data_tmp++) {
      x[b_data_tmp] = x_data[b_data_tmp];
    }

    for (b_data_tmp = 0; b_data_tmp < 4; b_data_tmp++) {
      point_0_0 = 0.0F;
      for (j_tmp = 0; j_tmp < 6; j_tmp++) {
        point_0_0 += (real32_T)point_0[(6 * j_tmp + b_data_tmp) + 1] * x[j_tmp];
      }

      b_0[b_data_tmp] = b_data_0[b_data_tmp] + point_0_0;
    }

    for (b_data_tmp = 0; b_data_tmp < n_size_idx_1_tmp; b_data_tmp++) {
      b_data[n_data[b_data_tmp]] = b_0[b_data_tmp];
    }

    if ((intermediate_size_tmp + 2.0F) + 1.0F > (((intermediate_size_tmp + 2.0F)
          + 1.0F) + 5.0F) - 2.0F) {
      tmp = 0;
      j_tmp = 0;
      loop_ub = 0;
      l = 0;
    } else {
      tmp = (int32_T)((intermediate_size_tmp + 2.0F) + 1.0F) - 1;
      j_tmp = (int32_T)((((intermediate_size_tmp + 2.0F) + 1.0F) + 5.0F) - 2.0F);
      loop_ub = (int32_T)((intermediate_size_tmp + 2.0F) + 1.0F) - 1;
      l = (int32_T)((((intermediate_size_tmp + 2.0F) + 1.0F) + 5.0F) - 2.0F);
    }

    n_size_idx_1_tmp = l - loop_ub;
    for (b_data_tmp = 0; b_data_tmp < n_size_idx_1_tmp; b_data_tmp++) {
      n_data[b_data_tmp] = loop_ub + b_data_tmp;
    }

    loop_ub = j_tmp - tmp;
    for (b_data_tmp = 0; b_data_tmp < loop_ub; b_data_tmp++) {
      b_data_0[b_data_tmp] = b_data[tmp + b_data_tmp];
    }

    for (b_data_tmp = 0; b_data_tmp < 6; b_data_tmp++) {
      x[b_data_tmp] = x_data[(int32_T)((intermediate_size_tmp + 1.0F) +
        (real32_T)b_data_tmp) - 1];
    }

    for (b_data_tmp = 0; b_data_tmp < 4; b_data_tmp++) {
      point_0_0 = 0.0F;
      for (j_tmp = 0; j_tmp < 6; j_tmp++) {
        point_0_0 += (real32_T)pp[(6 * j_tmp + b_data_tmp) + 1] * x[j_tmp];
      }

      b_0[b_data_tmp] = b_data_0[b_data_tmp] - point_0_0;
    }

    for (b_data_tmp = 0; b_data_tmp < n_size_idx_1_tmp; b_data_tmp++) {
      b_data[n_data[b_data_tmp]] = b_0[b_data_tmp];
    }
  }

  for (tmp = 0; tmp < (int32_T)(num_of_splines - 1.0F); tmp++) {
    intermediate_size_tmp = ((1.0F + (real32_T)tmp) - 1.0F) * 6.0F;
    intermediate_size = intermediate_size_tmp + (real32_T)bnd_left;
    point_0_0 = 0.0F;
    for (b_data_tmp = 0; b_data_tmp < 6; b_data_tmp++) {
      point_0_0 += x_data[(int32_T)((intermediate_size_tmp + 1.0F) + (real32_T)
        b_data_tmp) - 1] * (real32_T)pp[6 * b_data_tmp];
    }

    b_data_tmp = (int32_T)(intermediate_size + 1.0F) - 1;
    b_data[b_data_tmp] += point_0_0;
    point_0_0 = 0.0F;
    for (b_data_tmp = 0; b_data_tmp < 6; b_data_tmp++) {
      point_0_0 += x_data[(int32_T)(((intermediate_size_tmp + 1.0F) + 6.0F) +
        (real32_T)b_data_tmp) - 1] * (real32_T)point_0[6 * b_data_tmp];
    }

    b_data_tmp = (int32_T)(intermediate_size + 2.0F) - 1;
    b_data[b_data_tmp] += point_0_0;
    if (intermediate_size + 3.0F > ((intermediate_size + 3.0F) + 5.0F) - 2.0F) {
      j_tmp = 0;
      loop_ub = 0;
      l = 0;
      b_data_tmp = 0;
    } else {
      j_tmp = (int32_T)(intermediate_size + 3.0F) - 1;
      loop_ub = (int32_T)(((intermediate_size + 3.0F) + 5.0F) - 2.0F);
      l = j_tmp;
      b_data_tmp = loop_ub;
    }

    n_size_idx_1_tmp = b_data_tmp - l;
    for (b_data_tmp = 0; b_data_tmp < n_size_idx_1_tmp; b_data_tmp++) {
      w_data[b_data_tmp] = l + b_data_tmp;
    }

    loop_ub -= j_tmp;
    for (b_data_tmp = 0; b_data_tmp < loop_ub; b_data_tmp++) {
      b_data_1[b_data_tmp] = b_data[j_tmp + b_data_tmp];
    }

    for (b_data_tmp = 0; b_data_tmp < 6; b_data_tmp++) {
      x[b_data_tmp] = x_data[(int32_T)((intermediate_size_tmp + 1.0F) +
        (real32_T)b_data_tmp) - 1];
    }

    for (b_data_tmp = 0; b_data_tmp < 4; b_data_tmp++) {
      point_0_0 = 0.0F;
      for (j_tmp = 0; j_tmp < 6; j_tmp++) {
        point_0_0 += (real32_T)pp[(6 * j_tmp + b_data_tmp) + 1] * x[j_tmp];
      }

      b_0[b_data_tmp] = b_data_1[b_data_tmp] + point_0_0;
    }

    for (b_data_tmp = 0; b_data_tmp < n_size_idx_1_tmp; b_data_tmp++) {
      b_data[w_data[b_data_tmp]] = b_0[b_data_tmp];
    }

    if (intermediate_size + 3.0F > ((intermediate_size + 3.0F) + 5.0F) - 2.0F) {
      j_tmp = 0;
      loop_ub = 0;
      l = 0;
      b_data_tmp = 0;
    } else {
      j_tmp = (int32_T)(intermediate_size + 3.0F) - 1;
      loop_ub = (int32_T)(((intermediate_size + 3.0F) + 5.0F) - 2.0F);
      l = (int32_T)(intermediate_size + 3.0F) - 1;
      b_data_tmp = (int32_T)(((intermediate_size + 3.0F) + 5.0F) - 2.0F);
    }

    n_size_idx_1_tmp = b_data_tmp - l;
    for (b_data_tmp = 0; b_data_tmp < n_size_idx_1_tmp; b_data_tmp++) {
      w_data[b_data_tmp] = l + b_data_tmp;
    }

    loop_ub -= j_tmp;
    for (b_data_tmp = 0; b_data_tmp < loop_ub; b_data_tmp++) {
      b_data_1[b_data_tmp] = b_data[j_tmp + b_data_tmp];
    }

    for (b_data_tmp = 0; b_data_tmp < 6; b_data_tmp++) {
      x[b_data_tmp] = x_data[(int32_T)(((intermediate_size_tmp + 1.0F) + 6.0F) +
        (real32_T)b_data_tmp) - 1];
    }

    for (b_data_tmp = 0; b_data_tmp < 4; b_data_tmp++) {
      point_0_0 = 0.0F;
      for (j_tmp = 0; j_tmp < 6; j_tmp++) {
        point_0_0 += (real32_T)point_0[(6 * j_tmp + b_data_tmp) + 1] * x[j_tmp];
      }

      b_0[b_data_tmp] = b_data_1[b_data_tmp] - point_0_0;
    }

    for (b_data_tmp = 0; b_data_tmp < n_size_idx_1_tmp; b_data_tmp++) {
      b_data[w_data[b_data_tmp]] = b_0[b_data_tmp];
    }
  }
}

// Function for MATLAB Function: '<S75>/trajFromWaypoints'
real32_T MatlabControllerClass::norm(const real32_T x_data[], const int32_T
  *x_size)
{
  real32_T y;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  int32_T k;
  if (*x_size == 0) {
    y = 0.0F;
  } else {
    y = 0.0F;
    if (*x_size == 1) {
      y = std::abs(x_data[0]);
    } else {
      scale = 1.29246971E-26F;
      for (k = 0; k < *x_size; k++) {
        absxk = std::abs(x_data[k]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0F;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  return y;
}

// Function for MATLAB Function: '<S75>/trajFromWaypoints'
void MatlabControllerClass::polyInterpolationAx(real32_T num_of_splines,
  boolean_T cycle, const real32_T x_data[], const int32_T *x_size, real32_T
  b_data[], int32_T *b_size)
{
  real_T pp[36];
  real_T point_0[36];
  real32_T intermediate_size;
  int32_T bnd_left;
  int32_T g;
  int32_T l;
  int32_T b_tmp[6];
  real_T pp_data[6];
  real_T tmp_data[5];
  real32_T b[6];
  real32_T x_data_0[60];
  real32_T x;
  real32_T x_data_1[57];
  int32_T loop_ub;
  int32_T pp_size[2];
  int32_T tmp_size[2];
  int8_T b_x_idx_0;
  int32_T tmp;
  int8_T c_idx_0;
  real32_T intermediate_size_tmp;
  b_x_idx_0 = (int8_T)*x_size;
  c_idx_0 = (int8_T)*x_size;
  *b_size = b_x_idx_0;
  if (0 <= b_x_idx_0 - 1) {
    memset(&b_data[0], 0, b_x_idx_0 * sizeof(real32_T));
  }

  for (l = 0; l < 36; l++) {
    pp[l] = 1.0;
  }

  for (l = 0; l < 6; l++) {
    for (bnd_left = 0; bnd_left < 5; bnd_left++) {
      pp[(bnd_left + 6 * l) + 1] = 0.0;
    }
  }

  for (bnd_left = 0; bnd_left < 5; bnd_left++) {
    loop_ub = 6 - bnd_left;
    pp_size[0] = 1;
    pp_size[1] = loop_ub;
    for (l = 0; l < loop_ub; l++) {
      pp_data[l] = pp[6 * l + bnd_left];
    }

    polyder(pp_data, pp_size, tmp_data, tmp_size);
    loop_ub = tmp_size[1];
    for (l = 0; l < loop_ub; l++) {
      pp[(bnd_left + 6 * l) + 1] = tmp_data[l];
    }
  }

  memcpy(&point_0[0], &pp[0], 36U * sizeof(real_T));
  for (bnd_left = 0; bnd_left < 5; bnd_left++) {
    tmp = 5 - bnd_left;
    for (l = 0; l < tmp; l++) {
      point_0[bnd_left + 6 * l] = 0.0;
    }
  }

  intermediate_size_tmp = (num_of_splines - 1.0F) * 6.0F;
  if (!cycle) {
    bnd_left = 3;
    *b_size = c_idx_0;
    if (0 <= c_idx_0 - 1) {
      memset(&b_data[0], 0, c_idx_0 * sizeof(real32_T));
    }

    for (l = 0; l < 6; l++) {
      b_data[l] = ((real32_T)point_0[6 * l + 1] * x_data[1] + (real32_T)point_0
                   [6 * l] * x_data[0]) + (real32_T)point_0[6 * l + 2] * x_data
        [2];
    }

    if (4.0F + intermediate_size_tmp > ((4.0F + intermediate_size_tmp) + 3.0F) -
        1.0F) {
      tmp = 0;
      g = 0;
    } else {
      tmp = (int32_T)(4.0F + intermediate_size_tmp) - 1;
      g = (int32_T)(((4.0F + intermediate_size_tmp) + 3.0F) - 1.0F);
    }

    for (l = 0; l < 6; l++) {
      b_tmp[l] = (int32_T)((1.0F + intermediate_size_tmp) + (real32_T)l);
    }

    loop_ub = g - tmp;
    for (l = 0; l < loop_ub; l++) {
      x_data_0[l] = x_data[tmp + l];
    }

    for (l = 0; l < 6; l++) {
      b[l] = (((real32_T)pp[6 * l + 1] * x_data_0[1] + (real32_T)pp[6 * l] *
               x_data_0[0]) + (real32_T)pp[6 * l + 2] * x_data_0[2]) +
        b_data[b_tmp[l] - 1];
    }

    for (l = 0; l < 6; l++) {
      b_data[b_tmp[l] - 1] = b[l];
    }
  } else {
    bnd_left = 2;
    for (l = 0; l < 6; l++) {
      b_data[l] = (real32_T)point_0[6 * l] * x_data[0];
      b_tmp[l] = (int32_T)((intermediate_size_tmp + 1.0F) + (real32_T)l);
    }

    for (l = 0; l < 6; l++) {
      b[l] = (real32_T)pp[6 * l] * x_data[1] + b_data[b_tmp[l] - 1];
    }

    for (l = 0; l < 6; l++) {
      b_data[b_tmp[l] - 1] = b[l];
    }

    if ((intermediate_size_tmp + 2.0F) + 1.0F > (((intermediate_size_tmp + 2.0F)
          + 1.0F) + 5.0F) - 2.0F) {
      tmp = 0;
      g = 0;
    } else {
      tmp = (int32_T)((intermediate_size_tmp + 2.0F) + 1.0F) - 1;
      g = (int32_T)((((intermediate_size_tmp + 2.0F) + 1.0F) + 5.0F) - 2.0F);
    }

    loop_ub = g - tmp;
    for (l = 0; l < loop_ub; l++) {
      x_data_0[l] = x_data[tmp + l];
    }

    for (l = 0; l < 6; l++) {
      b_data[l] += (((real32_T)point_0[6 * l + 1] * x_data_0[0] + (real32_T)
                     point_0[6 * l + 2] * x_data_0[1]) + (real32_T)point_0[6 * l
                    + 3] * x_data_0[2]) + (real32_T)point_0[6 * l + 4] *
        x_data_0[3];
    }

    if ((intermediate_size_tmp + 2.0F) + 1.0F > (((intermediate_size_tmp + 2.0F)
          + 1.0F) + 5.0F) - 2.0F) {
      tmp = 0;
      g = 0;
    } else {
      tmp = (int32_T)((intermediate_size_tmp + 2.0F) + 1.0F) - 1;
      g = (int32_T)((((intermediate_size_tmp + 2.0F) + 1.0F) + 5.0F) - 2.0F);
    }

    loop_ub = g - tmp;
    for (l = 0; l < loop_ub; l++) {
      x_data_0[l] = x_data[tmp + l];
    }

    for (l = 0; l < 6; l++) {
      b[l] = b_data[(int32_T)((intermediate_size_tmp + 1.0F) + (real32_T)l) - 1]
        - ((((real32_T)pp[6 * l + 1] * x_data_0[0] + (real32_T)pp[6 * l + 2] *
             x_data_0[1]) + (real32_T)pp[6 * l + 3] * x_data_0[2]) + (real32_T)
           pp[6 * l + 4] * x_data_0[3]);
    }

    for (l = 0; l < 6; l++) {
      b_data[(int32_T)((intermediate_size_tmp + 1.0F) + (real32_T)l) - 1] = b[l];
    }
  }

  for (tmp = 0; tmp < (int32_T)(num_of_splines - 1.0F); tmp++) {
    intermediate_size_tmp = ((1.0F + (real32_T)tmp) - 1.0F) * 6.0F;
    intermediate_size = intermediate_size_tmp + (real32_T)bnd_left;
    for (l = 0; l < 6; l++) {
      b_tmp[l] = (int32_T)((intermediate_size_tmp + 1.0F) + (real32_T)l);
    }

    x = x_data[(int32_T)(intermediate_size + 1.0F) - 1];
    for (l = 0; l < 6; l++) {
      b[l] = (real32_T)pp[6 * l] * x + b_data[b_tmp[l] - 1];
    }

    for (l = 0; l < 6; l++) {
      b_data[b_tmp[l] - 1] = b[l];
    }

    for (l = 0; l < 6; l++) {
      b_tmp[l] = (int32_T)(((intermediate_size_tmp + 1.0F) + 6.0F) + (real32_T)l);
    }

    x = x_data[(int32_T)(intermediate_size + 2.0F) - 1];
    for (l = 0; l < 6; l++) {
      b[l] = (real32_T)point_0[6 * l] * x + b_data[b_tmp[l] - 1];
    }

    for (l = 0; l < 6; l++) {
      b_data[b_tmp[l] - 1] = b[l];
    }

    if (intermediate_size + 3.0F > ((intermediate_size + 3.0F) + 5.0F) - 2.0F) {
      g = 0;
      l = 0;
    } else {
      g = (int32_T)(intermediate_size + 3.0F) - 1;
      l = (int32_T)(((intermediate_size + 3.0F) + 5.0F) - 2.0F);
    }

    loop_ub = l - g;
    for (l = 0; l < loop_ub; l++) {
      x_data_1[l] = x_data[g + l];
    }

    for (l = 0; l < 6; l++) {
      b[l] = ((((real32_T)pp[6 * l + 1] * x_data_1[0] + (real32_T)pp[6 * l + 2] *
                x_data_1[1]) + (real32_T)pp[6 * l + 3] * x_data_1[2]) +
              (real32_T)pp[6 * l + 4] * x_data_1[3]) + b_data[(int32_T)
        ((intermediate_size_tmp + 1.0F) + (real32_T)l) - 1];
    }

    for (l = 0; l < 6; l++) {
      b_data[(int32_T)((intermediate_size_tmp + 1.0F) + (real32_T)l) - 1] = b[l];
    }

    if (intermediate_size + 3.0F > ((intermediate_size + 3.0F) + 5.0F) - 2.0F) {
      g = 0;
      l = 0;
    } else {
      g = (int32_T)(intermediate_size + 3.0F) - 1;
      l = (int32_T)(((intermediate_size + 3.0F) + 5.0F) - 2.0F);
    }

    loop_ub = l - g;
    for (l = 0; l < loop_ub; l++) {
      x_data_1[l] = x_data[g + l];
    }

    for (l = 0; l < 6; l++) {
      b[l] = b_data[(int32_T)(((intermediate_size_tmp + 1.0F) + 6.0F) +
        (real32_T)l) - 1] - ((((real32_T)point_0[6 * l + 1] * x_data_1[0] +
        (real32_T)point_0[6 * l + 2] * x_data_1[1]) + (real32_T)point_0[6 * l +
        3] * x_data_1[2]) + (real32_T)point_0[6 * l + 4] * x_data_1[3]);
    }

    for (l = 0; l < 6; l++) {
      b_data[(int32_T)(((intermediate_size_tmp + 1.0F) + 6.0F) + (real32_T)l) -
        1] = b[l];
    }
  }
}

// Function for MATLAB Function: '<S75>/trajFromWaypoints'
real32_T MatlabControllerClass::norm_c(const real32_T x[2])
{
  real32_T y;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  scale = 1.29246971E-26F;
  absxk = std::abs(x[0]);
  if (absxk > 1.29246971E-26F) {
    y = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    y = t * t;
  }

  absxk = std::abs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * std::sqrt(y);
}

// Function for MATLAB Function: '<S75>/trajFromWaypoints'
void MatlabControllerClass::ladacLsqrIterate(real32_T A_tunableEnvironment_f1,
  boolean_T A_tunableEnvironment_f3, real32_T x_data[], int32_T *x_size,
  real32_T w_data[], int32_T *w_size, real32_T u_data[], int32_T *u_size,
  real32_T v_data[], int32_T *v_size, real32_T *Anorm, real32_T *alfa, real32_T *
  rhobar, real32_T *phibar)
{
  real32_T beta;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  real32_T y_tmp_data[60];
  real32_T rhobar_0[2];
  real32_T tmp_data[60];
  int32_T i;
  int32_T tmp_size;
  polyInterpolationAx_f(A_tunableEnvironment_f1, A_tunableEnvironment_f3, v_data,
                        v_size, tmp_data, &tmp_size);
  *u_size = tmp_size;
  for (i = 0; i < tmp_size; i++) {
    u_data[i] = tmp_data[i] - *alfa * u_data[i];
  }

  beta = norm(u_data, u_size);
  if (beta > 0.0F) {
    scale = 1.0F / beta;
    tmp_size = *u_size;
    for (i = 0; i < tmp_size; i++) {
      y_tmp_data[i] = scale * u_data[i];
    }

    if (0 <= *u_size - 1) {
      memcpy(&u_data[0], &y_tmp_data[0], *u_size * sizeof(real32_T));
    }

    absxk = *Anorm;
    *Anorm = 0.0F;
    scale = 1.29246971E-26F;
    absxk = std::abs(absxk);
    if (absxk > 1.29246971E-26F) {
      t = 1.29246971E-26F / absxk;
      *Anorm = *Anorm * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / 1.29246971E-26F;
      *Anorm += t * t;
    }

    absxk = std::abs(*alfa);
    if (absxk > scale) {
      t = scale / absxk;
      *Anorm = *Anorm * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      *Anorm += t * t;
    }

    if (beta > scale) {
      t = scale / beta;
      *Anorm = *Anorm * t * t + 1.0F;
      scale = beta;
    } else {
      t = beta / scale;
      *Anorm += t * t;
    }

    *Anorm = scale * std::sqrt(*Anorm);
    polyInterpolationAx(A_tunableEnvironment_f1, A_tunableEnvironment_f3,
                        y_tmp_data, u_size, tmp_data, &tmp_size);
    *v_size = tmp_size;
    for (i = 0; i < tmp_size; i++) {
      v_data[i] = tmp_data[i] - beta * v_data[i];
    }

    *alfa = norm(v_data, v_size);
    if (*alfa > 0.0F) {
      scale = 1.0F / *alfa;
      tmp_size = *v_size;
      for (i = 0; i < tmp_size; i++) {
        v_data[i] *= scale;
      }
    }

    rhobar_0[0] = *rhobar;
    rhobar_0[1] = 0.0F;
    absxk = norm_c(rhobar_0);
    *phibar *= *rhobar / absxk;
    rhobar_0[0] = absxk;
    rhobar_0[1] = beta;
    scale = norm_c(rhobar_0);
    absxk /= scale;
    beta /= scale;
    *rhobar = -absxk * *alfa;
    absxk *= *phibar;
    *phibar *= beta;
    absxk /= scale;
    beta = -(beta * *alfa) / scale;
    tmp_size = *x_size;
    for (i = 0; i < tmp_size; i++) {
      x_data[i] += absxk * w_data[i];
    }

    *w_size = *v_size;
    tmp_size = *v_size;
    for (i = 0; i < tmp_size; i++) {
      w_data[i] = beta * w_data[i] + v_data[i];
    }
  }
}

// Function for MATLAB Function: '<S75>/trajFromWaypoints'
void MatlabControllerClass::polyInterpolationb(const real32_T points_data[],
  const int32_T points_size[2], boolean_T cycle, real32_T b_data[], int32_T
  *b_size, real32_T *num_of_splines)
{
  real32_T points_new_data[11];
  int32_T size_A_mat;
  int32_T bnd_left;
  int32_T itm_row;
  int32_T points_new_size_idx_1;
  real32_T b_data_tmp;
  if (cycle) {
    points_new_size_idx_1 = points_size[1] + 1;
    size_A_mat = points_size[0] * points_size[1];
    if (0 <= size_A_mat - 1) {
      memcpy(&points_new_data[0], &points_data[0], size_A_mat * sizeof(real32_T));
    }

    points_new_data[size_A_mat] = points_data[0];
  } else {
    points_new_size_idx_1 = points_size[1];
    size_A_mat = points_size[0] * points_size[1] - 1;
    if (0 <= size_A_mat) {
      memcpy(&points_new_data[0], &points_data[0], (size_A_mat + 1) * sizeof
             (real32_T));
    }
  }

  *num_of_splines = (real32_T)points_new_size_idx_1 - 1.0F;
  size_A_mat = (points_new_size_idx_1 - 1) * 6;
  *b_size = size_A_mat;
  if (0 <= size_A_mat - 1) {
    memset(&b_data[0], 0, size_A_mat * sizeof(real32_T));
  }

  if (!cycle) {
    bnd_left = 3;
    b_data[0] = points_new_data[0];
    b_data[size_A_mat - 3] = points_new_data[points_new_size_idx_1 - 1];
    b_data[1] = points_new_data[1] - points_new_data[0];
    b_data[size_A_mat - 2] = points_new_data[points_new_size_idx_1 - 1] -
      points_new_data[points_new_size_idx_1 - 2];
  } else {
    bnd_left = 2;
    b_data[0] = points_new_data[0];
    b_data[1] = points_new_data[points_new_size_idx_1 - 1];
  }

  for (size_A_mat = 0; size_A_mat <= points_new_size_idx_1 - 3; size_A_mat++) {
    itm_row = 6 * size_A_mat + bnd_left;
    b_data_tmp = points_new_data[size_A_mat + 1];
    b_data[itm_row] = b_data_tmp;
    b_data[itm_row + 1] = b_data_tmp;
  }
}

// Function for MATLAB Function: '<S75>/trajFromWaypoints'
void MatlabControllerClass::ladacLsqrInit(real32_T A_tunableEnvironment_f1,
  boolean_T A_tunableEnvironment_f3, const real32_T b_data[], const int32_T
  *b_size, real32_T x_data[], int32_T *x_size, real32_T w_data[], int32_T
  *w_size, real32_T u_data[], int32_T *u_size, real32_T v_data[], int32_T
  *v_size, real32_T *alfa, real32_T *rhobar, real32_T *phibar)
{
  real32_T beta;
  real32_T z;
  int32_T loop_ub;
  int32_T i;
  *x_size = *b_size;
  if (0 <= *b_size - 1) {
    memset(&x_data[0], 0, *b_size * sizeof(real32_T));
  }

  *u_size = *b_size;
  if (0 <= *b_size - 1) {
    memcpy(&u_data[0], &b_data[0], *b_size * sizeof(real32_T));
  }

  *v_size = *b_size;
  if (0 <= *b_size - 1) {
    memset(&v_data[0], 0, *b_size * sizeof(real32_T));
  }

  *w_size = *b_size;
  if (0 <= *b_size - 1) {
    memset(&w_data[0], 0, *b_size * sizeof(real32_T));
  }

  *alfa = 0.0F;
  beta = norm(b_data, b_size);
  *rhobar = 0.0F;
  *phibar = 0.0F;
  if (beta > 0.0F) {
    z = 1.0F / beta;
    *u_size = *b_size;
    loop_ub = *b_size;
    for (i = 0; i < loop_ub; i++) {
      u_data[i] = z * b_data[i];
    }

    polyInterpolationAx(A_tunableEnvironment_f1, A_tunableEnvironment_f3, u_data,
                        u_size, v_data, v_size);
    *alfa = norm(v_data, v_size);
  }

  if (*alfa > 0.0F) {
    z = 1.0F / *alfa;
    loop_ub = *v_size;
    for (i = 0; i < loop_ub; i++) {
      v_data[i] *= z;
    }

    *w_size = *v_size;
    if (0 <= *v_size - 1) {
      memcpy(&w_data[0], &v_data[0], *v_size * sizeof(real32_T));
    }
  }

  if (*alfa * beta != 0.0F) {
    *rhobar = *alfa;
    *phibar = beta;
  }
}

// Function for MATLAB Function: '<S75>/trajFromWaypoints'
boolean_T MatlabControllerClass::trajValidateWaypoints(uint16_T num_wp)
{
  boolean_T is_valid;
  is_valid = false;
  if (num_wp >= 3) {
    is_valid = true;
  }

  return is_valid;
}

// Function for MATLAB Function: '<S78>/flight path matching'
real32_T MatlabControllerClass::norm_e(const real32_T x[3])
{
  real32_T y;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  scale = 1.29246971E-26F;
  absxk = std::abs(x[0]);
  if (absxk > 1.29246971E-26F) {
    y = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    y = t * t;
  }

  absxk = std::abs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = std::abs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * std::sqrt(y);
}

// Function for MATLAB Function: '<S78>/flight path matching'
void MatlabControllerClass::trajSectionGetPos_i(const real32_T
  traj_section_pos_x[6], const real32_T traj_section_pos_y[6], const real32_T
  traj_section_pos_z[6], real32_T varargin_1, real32_T pos[3])
{
  real32_T px;
  real32_T py;
  real32_T pz;
  int32_T i;
  px = traj_section_pos_x[0];
  py = traj_section_pos_y[0];
  pz = traj_section_pos_z[0];
  for (i = 0; i < 5; i++) {
    px = varargin_1 * px + traj_section_pos_x[i + 1];
    py = varargin_1 * py + traj_section_pos_y[i + 1];
    pz = varargin_1 * pz + traj_section_pos_z[i + 1];
  }

  pos[0] = px;
  pos[1] = py;
  pos[2] = pz;
}

// Function for MATLAB Function: '<S78>/flight path matching'
void MatlabControllerClass::polyder_mj(const real32_T u_data[], const int32_T
  u_size[2], real32_T a_data[], int32_T a_size[2])
{
  int32_T nymax;
  int32_T nlead0;
  int32_T ny;
  if (u_size[1] < 2) {
    nymax = 1;
  } else {
    nymax = u_size[1] - 1;
  }

  a_size[0] = 1;
  a_size[1] = nymax;
  switch (u_size[1]) {
   case 0:
    a_data[0] = 0.0F;
    break;

   case 1:
    a_data[0] = 0.0F;
    break;

   default:
    nlead0 = 0;
    ny = 0;
    while ((ny <= nymax - 2) && (u_data[ny] == 0.0F)) {
      nlead0++;
      ny++;
    }

    ny = nymax - nlead0;
    a_size[0] = 1;
    a_size[1] = ny;
    for (nymax = 0; nymax < ny; nymax++) {
      a_data[nymax] = u_data[nymax + nlead0];
    }
    break;
  }

  nlead0 = a_size[1] - 2;
  for (ny = 0; ny <= nlead0; ny++) {
    a_data[ny] *= (real32_T)((nlead0 - ny) + 1) + 1.0F;
  }
}

// Function for MATLAB Function: '<S78>/flight path matching'
void MatlabControllerClass::trajGetMatchEnhanced(const
  dtoSgl_trajectoryStructBus *traj, const real32_T position[3], real32_T
  *section_idx, real32_T *error, real32_T *t)
{
  real32_T px[12];
  real32_T py[12];
  real32_T pz[12];
  real32_T curr_pos[36];
  real32_T trajSection_pos_x[6];
  real32_T trajSection_pos_y[6];
  real32_T trajSection_pos_z[6];
  real32_T f[3];
  real32_T eval_fun;
  real32_T b_px;
  real32_T b_py;
  real32_T b_pz;
  int32_T b_i;
  real_T b_section_idx;
  real32_T dx_data[5];
  real32_T dy_data[5];
  real32_T dz_data[5];
  int32_T b_k;
  static const real32_T x[12] = { 0.0F, 0.0909090936F, 0.181818187F,
    0.272727281F, 0.363636374F, 0.454545468F, 0.545454562F, 0.636363626F,
    0.727272749F, 0.818181813F, 0.909090936F, 1.0F };

  static const real32_T b[12] = { 0.0F, 0.0909090936F, 0.181818187F,
    0.272727281F, 0.363636374F, 0.454545468F, 0.545454562F, 0.636363626F,
    0.727272749F, 0.818181813F, 0.909090936F, 1.0F };

  real32_T tmp_data[4];
  real32_T tmp_data_0[4];
  real32_T tmp_data_1[4];
  int32_T dx_size[2];
  int32_T dy_size[2];
  int32_T dz_size[2];
  int32_T tmp_size[2];
  real32_T eval_fun_tmp;
  real32_T eval_fun_tmp_0;
  real32_T eval_fun_tmp_1;
  int32_T curr_pos_tmp;
  int32_T curr_pos_tmp_0;
  boolean_T exitg1;
  *t = 0.0F;
  *section_idx = 1.0F;
  b_px = traj->sections[0].pos_x[0];
  b_py = traj->sections[0].pos_y[0];
  b_pz = traj->sections[0].pos_z[0];
  for (b_i = 0; b_i < 5; b_i++) {
    b_px = traj->sections[0].pos_x[b_i + 1];
    b_py = traj->sections[0].pos_y[b_i + 1];
    b_pz = traj->sections[0].pos_z[b_i + 1];
  }

  f[0] = position[0] - b_px;
  f[1] = position[1] - b_py;
  f[2] = position[2] - b_pz;
  *error = norm_e(f);
  for (b_i = 0; b_i < (int32_T)traj->num_sections_set; b_i++) {
    for (curr_pos_tmp = 0; curr_pos_tmp < 12; curr_pos_tmp++) {
      px[curr_pos_tmp] = traj->sections[(int32_T)(1.0F + (real32_T)b_i) - 1].
        pos_x[0];
    }

    for (b_k = 0; b_k < 5; b_k++) {
      eval_fun = traj->sections[(int32_T)(1.0F + (real32_T)b_i) - 1].pos_x[b_k +
        1];
      for (curr_pos_tmp = 0; curr_pos_tmp < 12; curr_pos_tmp++) {
        px[curr_pos_tmp] = x[curr_pos_tmp] * px[curr_pos_tmp] + eval_fun;
      }
    }

    for (curr_pos_tmp = 0; curr_pos_tmp < 12; curr_pos_tmp++) {
      py[curr_pos_tmp] = traj->sections[(int32_T)(1.0F + (real32_T)b_i) - 1].
        pos_y[0];
    }

    for (b_k = 0; b_k < 5; b_k++) {
      eval_fun = traj->sections[(int32_T)(1.0F + (real32_T)b_i) - 1].pos_y[b_k +
        1];
      for (curr_pos_tmp = 0; curr_pos_tmp < 12; curr_pos_tmp++) {
        py[curr_pos_tmp] = x[curr_pos_tmp] * py[curr_pos_tmp] + eval_fun;
      }
    }

    for (curr_pos_tmp = 0; curr_pos_tmp < 12; curr_pos_tmp++) {
      pz[curr_pos_tmp] = traj->sections[(int32_T)(1.0F + (real32_T)b_i) - 1].
        pos_z[0];
    }

    for (b_k = 0; b_k < 5; b_k++) {
      eval_fun = traj->sections[(int32_T)(1.0F + (real32_T)b_i) - 1].pos_z[b_k +
        1];
      for (curr_pos_tmp = 0; curr_pos_tmp < 12; curr_pos_tmp++) {
        pz[curr_pos_tmp] = x[curr_pos_tmp] * pz[curr_pos_tmp] + eval_fun;
      }
    }

    for (b_k = 0; b_k < 12; b_k++) {
      curr_pos[3 * b_k] = px[b_k];
      curr_pos_tmp = 1 + 3 * b_k;
      curr_pos[curr_pos_tmp] = py[b_k];
      curr_pos_tmp_0 = 2 + 3 * b_k;
      curr_pos[curr_pos_tmp_0] = pz[b_k];
      curr_pos[3 * b_k] -= position[0];
      curr_pos[curr_pos_tmp] = curr_pos[3 * b_k + 1] - position[1];
      curr_pos[curr_pos_tmp_0] = curr_pos[3 * b_k + 2] - position[2];
    }

    for (b_k = 0; b_k < 12; b_k++) {
      curr_pos_tmp = b_k * 3 + 1;
      b_px = 0.0F;
      b_py = 1.29246971E-26F;
      for (curr_pos_tmp_0 = curr_pos_tmp; curr_pos_tmp_0 <= curr_pos_tmp + 2;
           curr_pos_tmp_0++) {
        b_pz = std::abs(curr_pos[curr_pos_tmp_0 - 1]);
        if (b_pz > b_py) {
          eval_fun = b_py / b_pz;
          b_px = b_px * eval_fun * eval_fun + 1.0F;
          b_py = b_pz;
        } else {
          eval_fun = b_pz / b_py;
          b_px += eval_fun * eval_fun;
        }
      }

      px[b_k] = b_py * std::sqrt(b_px);
    }

    b_px = px[0];
    b_k = -1;
    for (curr_pos_tmp = 0; curr_pos_tmp < 11; curr_pos_tmp++) {
      eval_fun = px[curr_pos_tmp + 1];
      if (b_px > eval_fun) {
        b_px = eval_fun;
        b_k = curr_pos_tmp;
      }
    }

    if (b_px < *error) {
      *error = b_px;
      *section_idx = 1.0F + (real32_T)b_i;
      *t = b[b_k + 1];
    }
  }

  b_section_idx = *section_idx;
  if ((*section_idx > traj->num_sections_set) || (*section_idx < 1.0F)) {
    b_section_idx = 1.0;
  }

  for (curr_pos_tmp = 0; curr_pos_tmp < 6; curr_pos_tmp++) {
    trajSection_pos_x[curr_pos_tmp] = traj->sections[(int32_T)b_section_idx - 1]
      .pos_x[curr_pos_tmp];
    trajSection_pos_y[curr_pos_tmp] = traj->sections[(int32_T)b_section_idx - 1]
      .pos_y[curr_pos_tmp];
    trajSection_pos_z[curr_pos_tmp] = traj->sections[(int32_T)b_section_idx - 1]
      .pos_z[curr_pos_tmp];
  }

  b_i = 0;
  exitg1 = false;
  while ((!exitg1) && (b_i < 100)) {
    trajSectionGetPos_i(trajSection_pos_x, trajSection_pos_y, trajSection_pos_z,
                        *t, f);
    polyder_p(trajSection_pos_x, dx_data, dx_size);
    polyder_p(trajSection_pos_y, dy_data, dy_size);
    polyder_p(trajSection_pos_z, dz_data, dz_size);
    b_px = polyVal(dx_data, dx_size, (real_T)*t);
    b_py = polyVal(dy_data, dy_size, (real_T)*t);
    b_pz = polyVal(dz_data, dz_size, (real_T)*t);
    polyder_mj(dx_data, dx_size, tmp_data, tmp_size);
    polyder_mj(dy_data, dy_size, tmp_data_0, dx_size);
    polyder_mj(dz_data, dz_size, tmp_data_1, dy_size);
    eval_fun_tmp = f[0] - position[0];
    eval_fun_tmp_0 = f[1] - position[1];
    eval_fun_tmp_1 = f[2] - position[2];
    eval_fun = ((eval_fun_tmp * b_px + eval_fun_tmp_0 * b_py) + eval_fun_tmp_1 *
                b_pz) * 2.0F;
    b_px = (((((eval_fun_tmp * polyVal(tmp_data, tmp_size, (real_T)*t) +
                eval_fun_tmp_0 * polyVal(tmp_data_0, dx_size, (real_T)*t)) +
               eval_fun_tmp_1 * polyVal(tmp_data_1, dy_size, (real_T)*t)) + b_px
              * b_px) + b_py * b_py) + b_pz * b_pz) * 2.0F;
    if (std::abs(b_px) < 1.0E-16) {
      b_px = 1.0E-8F;
    }

    if (std::abs(b_px) < 2.22044605E-16F) {
      b_px = 2.22044605E-16F;
    }

    *t -= eval_fun / b_px;
    if (std::abs(eval_fun) < 1.0E-12) {
      exitg1 = true;
    } else {
      if ((*t < 0.0F) || (*t > 1.0F)) {
        if (*t < 0.0F) {
          if (*t + 1.0F < 0.95) {
            *t = 0.95F;
          } else {
            (*t)++;
          }

          (*section_idx)--;
          if (*section_idx < 1.0F) {
            if (traj->is_repeated_course) {
              *section_idx = traj->num_sections_set;
            } else {
              *section_idx = 1.0F;
              *t = 0.0F;
            }
          }
        } else {
          if (*t - 1.0F > 0.05) {
            *t = 0.05F;
          } else {
            (*t)--;
          }

          (*section_idx)++;
          if (*section_idx > traj->num_sections_set) {
            if (traj->is_repeated_course) {
              *section_idx = 1.0F;
            } else {
              *section_idx = traj->num_sections_set;
              *t = 1.0F;
            }
          }
        }

        b_section_idx = *section_idx;
        if ((*section_idx > traj->num_sections_set) || (*section_idx < 1.0F)) {
          b_section_idx = 1.0;
        }

        for (curr_pos_tmp = 0; curr_pos_tmp < 6; curr_pos_tmp++) {
          trajSection_pos_x[curr_pos_tmp] = traj->sections[(int32_T)
            b_section_idx - 1].pos_x[curr_pos_tmp];
          trajSection_pos_y[curr_pos_tmp] = traj->sections[(int32_T)
            b_section_idx - 1].pos_y[curr_pos_tmp];
          trajSection_pos_z[curr_pos_tmp] = traj->sections[(int32_T)
            b_section_idx - 1].pos_z[curr_pos_tmp];
        }
      }

      b_i++;
    }
  }

  if (*t < 0.0F) {
    b_px = 0.0F;
  } else {
    b_px = *t;
  }

  if (b_px > 1.0F) {
    *t = 1.0F;
  } else {
    *t = b_px;
  }
}

// Function for MATLAB Function: '<S78>/flight path matching'
void MatlabControllerClass::trajSectionGetFrenetSerretWithG(const real32_T
  traj_section_pos_x[6], const real32_T traj_section_pos_y[6], const real32_T
  traj_section_pos_z[6], real32_T vel, real32_T g, real32_T varargin_1, real32_T
  T[3], real32_T B_6[3], real32_T N[3], real32_T *kappa, real32_T *tau)
{
  real32_T ddot_r_g[3];
  real32_T dot_r[3];
  real32_T dx_data[5];
  real32_T dy_data[5];
  real32_T dz_data[5];
  real32_T ddx_data[4];
  real32_T ddy_data[4];
  real32_T ddz_data[4];
  real32_T absxk;
  int32_T exponent;
  boolean_T isodd;
  real32_T A[9];
  int8_T ipiv[3];
  int32_T c_c;
  int32_T ix;
  real32_T smax;
  real32_T s;
  int32_T c_k;
  int32_T iy;
  int32_T c_ix;
  int32_T d;
  int32_T ijA;
  real32_T tmp_data[4];
  int32_T dx_size[2];
  int32_T dy_size[2];
  int32_T dz_size[2];
  int32_T ddx_size[2];
  real32_T ddot_r_idx_0;
  real32_T ddot_r_idx_1;
  real32_T ddot_r_idx_2;
  real32_T tmp;
  real32_T tmp_0;
  real32_T ddot_r_g_tmp;
  real32_T smax_tmp;
  polyder_p(traj_section_pos_x, dx_data, dx_size);
  polyder_p(traj_section_pos_y, dy_data, dy_size);
  polyder_p(traj_section_pos_z, dz_data, dz_size);
  s = polyVal(dx_data, dx_size, (real_T)varargin_1);
  tmp = polyVal(dy_data, dy_size, (real_T)varargin_1);
  tmp_0 = polyVal(dz_data, dz_size, (real_T)varargin_1);
  dot_r[0] = s;
  dot_r[1] = tmp;
  dot_r[2] = tmp_0;
  polyder_mj(dx_data, dx_size, ddx_data, ddx_size);
  polyder_mj(dy_data, dy_size, ddy_data, dx_size);
  polyder_mj(dz_data, dz_size, ddz_data, dy_size);
  ddot_r_idx_0 = polyVal(ddx_data, ddx_size, (real_T)varargin_1);
  ddot_r_idx_1 = polyVal(ddy_data, dx_size, (real_T)varargin_1);
  ddot_r_idx_2 = polyVal(ddz_data, dy_size, (real_T)varargin_1);
  absxk = std::abs(vel);
  if (absxk <= 1.17549435E-38F) {
    absxk = 1.4013E-45F;
  } else {
    std::frexp(absxk, &exponent);
    absxk = std::ldexp(1.0F, exponent - 24);
  }

  smax = vel * vel;
  if (absxk <= smax) {
    absxk = smax;
  }

  smax_tmp = norm_e(dot_r);
  smax = smax_tmp * smax_tmp;
  ddot_r_g_tmp = 0.0F / absxk * smax;
  ddot_r_g[0] = ddot_r_g_tmp + ddot_r_idx_0;
  ddot_r_g[1] = ddot_r_g_tmp + ddot_r_idx_1;
  ddot_r_g[2] = -g / absxk * smax + ddot_r_idx_2;
  if (norm_e(ddot_r_g) > 2.22044605E-16F) {
    ddot_r_idx_0 = ddot_r_g[0];
    ddot_r_idx_1 = ddot_r_g[1];
    ddot_r_idx_2 = ddot_r_g[2];
  }

  ddot_r_g[0] = tmp * ddot_r_idx_2 - tmp_0 * ddot_r_idx_1;
  ddot_r_g[1] = tmp_0 * ddot_r_idx_0 - s * ddot_r_idx_2;
  ddot_r_g[2] = s * ddot_r_idx_1 - tmp * ddot_r_idx_0;
  if (norm_e(ddot_r_g) < 2.22044605E-16F) {
    ddot_r_g[0] = 0.0F;
    ddot_r_g[1] = 2.22044605E-16F;
    ddot_r_g[2] = 0.0F;
  }

  absxk = smax_tmp;
  if (smax_tmp < 2.22044605E-16F) {
    absxk = 2.22044605E-16F;
  }

  T[0] = s / absxk;
  T[1] = tmp / absxk;
  T[2] = tmp_0 / absxk;
  ddot_r_g_tmp = norm_e(ddot_r_g);
  absxk = ddot_r_g_tmp;
  if (ddot_r_g_tmp < 2.22044605E-16F) {
    absxk = 2.22044605E-16F;
  }

  B_6[0] = ddot_r_g[0] / absxk;
  A[0] = s;
  A[3] = ddot_r_idx_0;
  ipiv[0] = 1;
  B_6[1] = ddot_r_g[1] / absxk;
  A[1] = tmp;
  A[4] = ddot_r_idx_1;
  ipiv[1] = 2;
  B_6[2] = ddot_r_g[2] / absxk;
  A[2] = tmp_0;
  A[5] = ddot_r_idx_2;
  absxk = std::pow(smax_tmp, 3.0F);
  polyder_mj(ddx_data, ddx_size, tmp_data, dz_size);
  polyder_mj(ddy_data, dx_size, ddx_data, ddx_size);
  polyder_mj(ddz_data, dy_size, ddy_data, dx_size);
  A[6] = polyVal(tmp_data, dz_size, (real_T)varargin_1);
  A[7] = polyVal(ddx_data, ddx_size, (real_T)varargin_1);
  A[8] = polyVal(ddy_data, dx_size, (real_T)varargin_1);
  for (exponent = 0; exponent < 2; exponent++) {
    c_c = exponent << 2;
    iy = 0;
    ix = c_c;
    smax = std::abs(A[c_c]);
    for (c_k = 2; c_k <= 3 - exponent; c_k++) {
      ix++;
      s = std::abs(A[ix]);
      if (s > smax) {
        iy = c_k - 1;
        smax = s;
      }
    }

    if (A[c_c + iy] != 0.0F) {
      if (iy != 0) {
        iy += exponent;
        ipiv[exponent] = (int8_T)(iy + 1);
        smax = A[exponent];
        A[exponent] = A[iy];
        A[iy] = smax;
        ix = exponent + 3;
        iy += 3;
        smax = A[ix];
        A[ix] = A[iy];
        A[iy] = smax;
        ix += 3;
        iy += 3;
        smax = A[ix];
        A[ix] = A[iy];
        A[iy] = smax;
      }

      iy = (c_c - exponent) + 3;
      for (ix = c_c + 1; ix < iy; ix++) {
        A[ix] /= A[c_c];
      }
    }

    iy = c_c + 4;
    ix = c_c + 3;
    for (c_k = 0; c_k <= 1 - exponent; c_k++) {
      smax = A[ix];
      if (A[ix] != 0.0F) {
        c_ix = c_c + 1;
        d = (iy - exponent) + 2;
        for (ijA = iy; ijA < d; ijA++) {
          A[ijA] += A[c_ix] * -smax;
          c_ix++;
        }
      }

      ix += 3;
      iy += 3;
    }
  }

  isodd = false;
  if (ipiv[0] > 1) {
    isodd = true;
  }

  smax = A[0] * A[4] * A[8];
  if (ipiv[1] > 2) {
    isodd = !isodd;
  }

  if (isodd) {
    smax = -smax;
  }

  s = ddot_r_g_tmp * ddot_r_g_tmp;
  N[0] = B_6[1] * T[2] - B_6[2] * T[1];
  N[1] = B_6[2] * T[0] - B_6[0] * T[2];
  N[2] = B_6[0] * T[1] - B_6[1] * T[0];
  if (absxk < 2.22044605E-16F) {
    absxk = 2.22044605E-16F;
  }

  *kappa = ddot_r_g_tmp / absxk;
  if (s < 2.22044605E-16F) {
    s = 2.22044605E-16F;
  }

  *tau = smax / s;
}

// Function for MATLAB Function: '<S77>/flight path look ahead'
real32_T MatlabControllerClass::integralSimpson(const b_cell_wrap_1
  func_tunableEnvironment[3], real_T B_5)
{
  real32_T Q;
  real_T t;
  real_T step;
  real_T step_1_2;
  int32_T i;
  real32_T a;
  real32_T b_a;
  real32_T c_a;
  real32_T d_a;
  real32_T e_a;
  real32_T f_a;
  real_T varargin_1;
  t = 0.0;
  step = B_5 / 15.0;
  step_1_2 = 0.5 * step;
  a = polyVal(func_tunableEnvironment[0].f1.data, func_tunableEnvironment[0].
              f1.size, 0.0);
  b_a = polyVal(func_tunableEnvironment[1].f1.data, func_tunableEnvironment[1].
                f1.size, 0.0);
  c_a = polyVal(func_tunableEnvironment[2].f1.data, func_tunableEnvironment[2].
                f1.size, 0.0);
  d_a = polyVal(func_tunableEnvironment[0].f1.data, func_tunableEnvironment[0].
                f1.size, B_5);
  e_a = polyVal(func_tunableEnvironment[1].f1.data, func_tunableEnvironment[1].
                f1.size, B_5);
  f_a = polyVal(func_tunableEnvironment[2].f1.data, func_tunableEnvironment[2].
                f1.size, B_5);
  Q = std::sqrt((a * a + b_a * b_a) + c_a * c_a) * 0.5F - std::sqrt((d_a * d_a +
    e_a * e_a) + f_a * f_a) * 0.5F;
  for (i = 0; i < 15; i++) {
    t += step;
    varargin_1 = t - step_1_2;
    a = polyVal(func_tunableEnvironment[0].f1.data, func_tunableEnvironment[0].
                f1.size, varargin_1);
    b_a = polyVal(func_tunableEnvironment[1].f1.data, func_tunableEnvironment[1]
                  .f1.size, varargin_1);
    c_a = polyVal(func_tunableEnvironment[2].f1.data, func_tunableEnvironment[2]
                  .f1.size, varargin_1);
    d_a = polyVal(func_tunableEnvironment[0].f1.data, func_tunableEnvironment[0]
                  .f1.size, t);
    e_a = polyVal(func_tunableEnvironment[1].f1.data, func_tunableEnvironment[1]
                  .f1.size, t);
    f_a = polyVal(func_tunableEnvironment[2].f1.data, func_tunableEnvironment[2]
                  .f1.size, t);
    Q = (std::sqrt((a * a + b_a * b_a) + c_a * c_a) * 2.0F + Q) + std::sqrt((d_a
      * d_a + e_a * e_a) + f_a * f_a);
  }

  Q *= (real32_T)(0.33333333333333331 * step);
  return Q;
}

// Function for MATLAB Function: '<S77>/flight path look ahead'
void MatlabControllerClass::trajSectionGetArcLength(const real32_T
  traj_section_pos_x[6], const real32_T traj_section_pos_y[6], const real32_T
  traj_section_pos_z[6], real32_T varargin_1, real32_T *arc_length, real32_T
  *arc_length_dt)
{
  real_T t;
  real32_T dx_data[5];
  real32_T dy_data[5];
  real32_T dz_data[5];
  b_cell_wrap_1 tunableEnvironment[3];
  real32_T a;
  real32_T b_a;
  real32_T c_a;
  int32_T loop_ub;
  int32_T i;
  int32_T dx_size[2];
  int32_T dy_size[2];
  int32_T dz_size[2];
  if (1.1 < varargin_1) {
    t = 1.1;
  } else {
    t = varargin_1;
  }

  if (t <= -0.1) {
    t = -0.1;
  }

  polyder_p(traj_section_pos_x, dx_data, dx_size);
  polyder_p(traj_section_pos_y, dy_data, dy_size);
  polyder_p(traj_section_pos_z, dz_data, dz_size);
  tunableEnvironment[0].f1.size[0] = 1;
  tunableEnvironment[0].f1.size[1] = dx_size[1];
  loop_ub = dx_size[0] * dx_size[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    tunableEnvironment[0].f1.data[i] = dx_data[i];
  }

  tunableEnvironment[1].f1.size[0] = 1;
  tunableEnvironment[1].f1.size[1] = dy_size[1];
  loop_ub = dy_size[0] * dy_size[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    tunableEnvironment[1].f1.data[i] = dy_data[i];
  }

  tunableEnvironment[2].f1.size[0] = 1;
  tunableEnvironment[2].f1.size[1] = dz_size[1];
  loop_ub = dz_size[0] * dz_size[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    tunableEnvironment[2].f1.data[i] = dz_data[i];
  }

  *arc_length = integralSimpson(tunableEnvironment, t);
  a = polyVal(dx_data, dx_size, t);
  b_a = polyVal(dy_data, dy_size, t);
  c_a = polyVal(dz_data, dz_size, t);
  *arc_length_dt = std::sqrt((a * a + b_a * b_a) + c_a * c_a);
}

// Function for MATLAB Function: '<S77>/flight path look ahead'
void MatlabControllerClass::trajSectionGetArcLength_j(const real32_T
  traj_section_pos_x[6], const real32_T traj_section_pos_y[6], const real32_T
  traj_section_pos_z[6], real_T varargin_1, real32_T *arc_length, real32_T
  *arc_length_dt)
{
  real_T t;
  real32_T dx_data[5];
  real32_T dy_data[5];
  real32_T dz_data[5];
  b_cell_wrap_1 tunableEnvironment[3];
  real32_T a;
  real32_T b_a;
  real32_T c_a;
  int32_T loop_ub;
  int32_T i;
  int32_T dx_size[2];
  int32_T dy_size[2];
  int32_T dz_size[2];
  if (1.1 < varargin_1) {
    t = 1.1;
  } else {
    t = varargin_1;
  }

  if (t <= -0.1) {
    t = -0.1;
  }

  polyder_p(traj_section_pos_x, dx_data, dx_size);
  polyder_p(traj_section_pos_y, dy_data, dy_size);
  polyder_p(traj_section_pos_z, dz_data, dz_size);
  tunableEnvironment[0].f1.size[0] = 1;
  tunableEnvironment[0].f1.size[1] = dx_size[1];
  loop_ub = dx_size[0] * dx_size[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    tunableEnvironment[0].f1.data[i] = dx_data[i];
  }

  tunableEnvironment[1].f1.size[0] = 1;
  tunableEnvironment[1].f1.size[1] = dy_size[1];
  loop_ub = dy_size[0] * dy_size[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    tunableEnvironment[1].f1.data[i] = dy_data[i];
  }

  tunableEnvironment[2].f1.size[0] = 1;
  tunableEnvironment[2].f1.size[1] = dz_size[1];
  loop_ub = dz_size[0] * dz_size[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    tunableEnvironment[2].f1.data[i] = dz_data[i];
  }

  *arc_length = integralSimpson(tunableEnvironment, t);
  a = polyVal(dx_data, dx_size, t);
  b_a = polyVal(dy_data, dy_size, t);
  c_a = polyVal(dz_data, dz_size, t);
  *arc_length_dt = std::sqrt((a * a + b_a * b_a) + c_a * c_a);
}

// Function for MATLAB Function: '<S80>/position controller reference from flight path'
void MatlabControllerClass::trajSectionGetFrenetSerretWit_i(const real32_T
  traj_section_pos_x[6], const real32_T traj_section_pos_y[6], const real32_T
  traj_section_pos_z[6], real32_T vel, real_T varargin_1, real32_T T[3],
  real32_T B_7[3], real32_T N[3], real32_T *kappa, real32_T *tau)
{
  real32_T ddot_r_g[3];
  real32_T dot_r[3];
  real32_T dx_data[5];
  real32_T dy_data[5];
  real32_T dz_data[5];
  real32_T ddx_data[4];
  real32_T ddy_data[4];
  real32_T ddz_data[4];
  real32_T absxk;
  int32_T exponent;
  boolean_T isodd;
  real32_T A[9];
  int8_T ipiv[3];
  int32_T c_c;
  int32_T ix;
  real32_T smax;
  real32_T s;
  int32_T c_k;
  int32_T iy;
  int32_T c_ix;
  int32_T d;
  int32_T ijA;
  real32_T tmp_data[4];
  int32_T dx_size[2];
  int32_T dy_size[2];
  int32_T dz_size[2];
  int32_T ddx_size[2];
  real32_T ddot_r_idx_0;
  real32_T ddot_r_idx_1;
  real32_T ddot_r_idx_2;
  real32_T tmp;
  real32_T tmp_0;
  real32_T ddot_r_g_tmp;
  real32_T smax_tmp;
  polyder_p(traj_section_pos_x, dx_data, dx_size);
  polyder_p(traj_section_pos_y, dy_data, dy_size);
  polyder_p(traj_section_pos_z, dz_data, dz_size);
  s = polyVal(dx_data, dx_size, varargin_1);
  tmp = polyVal(dy_data, dy_size, varargin_1);
  tmp_0 = polyVal(dz_data, dz_size, varargin_1);
  dot_r[0] = s;
  dot_r[1] = tmp;
  dot_r[2] = tmp_0;
  polyder_mj(dx_data, dx_size, ddx_data, ddx_size);
  polyder_mj(dy_data, dy_size, ddy_data, dx_size);
  polyder_mj(dz_data, dz_size, ddz_data, dy_size);
  ddot_r_idx_0 = polyVal(ddx_data, ddx_size, varargin_1);
  ddot_r_idx_1 = polyVal(ddy_data, dx_size, varargin_1);
  ddot_r_idx_2 = polyVal(ddz_data, dy_size, varargin_1);
  absxk = std::abs(vel);
  if (absxk <= 1.17549435E-38F) {
    absxk = 1.4013E-45F;
  } else {
    std::frexp(absxk, &exponent);
    absxk = std::ldexp(1.0F, exponent - 24);
  }

  smax = vel * vel;
  if (absxk <= smax) {
    absxk = smax;
  }

  smax_tmp = norm_e(dot_r);
  smax = smax_tmp * smax_tmp;
  ddot_r_g_tmp = 0.0F / absxk * smax;
  ddot_r_g[0] = ddot_r_g_tmp + ddot_r_idx_0;
  ddot_r_g[1] = ddot_r_g_tmp + ddot_r_idx_1;
  ddot_r_g[2] = -0.0F / absxk * smax + ddot_r_idx_2;
  if (norm_e(ddot_r_g) > 2.22044605E-16F) {
    ddot_r_idx_0 = ddot_r_g[0];
    ddot_r_idx_1 = ddot_r_g[1];
    ddot_r_idx_2 = ddot_r_g[2];
  }

  ddot_r_g[0] = tmp * ddot_r_idx_2 - tmp_0 * ddot_r_idx_1;
  ddot_r_g[1] = tmp_0 * ddot_r_idx_0 - s * ddot_r_idx_2;
  ddot_r_g[2] = s * ddot_r_idx_1 - tmp * ddot_r_idx_0;
  if (norm_e(ddot_r_g) < 2.22044605E-16F) {
    ddot_r_g[0] = 0.0F;
    ddot_r_g[1] = 2.22044605E-16F;
    ddot_r_g[2] = 0.0F;
  }

  absxk = smax_tmp;
  if (smax_tmp < 2.22044605E-16F) {
    absxk = 2.22044605E-16F;
  }

  T[0] = s / absxk;
  T[1] = tmp / absxk;
  T[2] = tmp_0 / absxk;
  ddot_r_g_tmp = norm_e(ddot_r_g);
  absxk = ddot_r_g_tmp;
  if (ddot_r_g_tmp < 2.22044605E-16F) {
    absxk = 2.22044605E-16F;
  }

  B_7[0] = ddot_r_g[0] / absxk;
  A[0] = s;
  A[3] = ddot_r_idx_0;
  ipiv[0] = 1;
  B_7[1] = ddot_r_g[1] / absxk;
  A[1] = tmp;
  A[4] = ddot_r_idx_1;
  ipiv[1] = 2;
  B_7[2] = ddot_r_g[2] / absxk;
  A[2] = tmp_0;
  A[5] = ddot_r_idx_2;
  absxk = std::pow(smax_tmp, 3.0F);
  polyder_mj(ddx_data, ddx_size, tmp_data, dz_size);
  polyder_mj(ddy_data, dx_size, ddx_data, ddx_size);
  polyder_mj(ddz_data, dy_size, ddy_data, dx_size);
  A[6] = polyVal(tmp_data, dz_size, varargin_1);
  A[7] = polyVal(ddx_data, ddx_size, varargin_1);
  A[8] = polyVal(ddy_data, dx_size, varargin_1);
  for (exponent = 0; exponent < 2; exponent++) {
    c_c = exponent << 2;
    iy = 0;
    ix = c_c;
    smax = std::abs(A[c_c]);
    for (c_k = 2; c_k <= 3 - exponent; c_k++) {
      ix++;
      s = std::abs(A[ix]);
      if (s > smax) {
        iy = c_k - 1;
        smax = s;
      }
    }

    if (A[c_c + iy] != 0.0F) {
      if (iy != 0) {
        iy += exponent;
        ipiv[exponent] = (int8_T)(iy + 1);
        smax = A[exponent];
        A[exponent] = A[iy];
        A[iy] = smax;
        ix = exponent + 3;
        iy += 3;
        smax = A[ix];
        A[ix] = A[iy];
        A[iy] = smax;
        ix += 3;
        iy += 3;
        smax = A[ix];
        A[ix] = A[iy];
        A[iy] = smax;
      }

      iy = (c_c - exponent) + 3;
      for (ix = c_c + 1; ix < iy; ix++) {
        A[ix] /= A[c_c];
      }
    }

    iy = c_c + 4;
    ix = c_c + 3;
    for (c_k = 0; c_k <= 1 - exponent; c_k++) {
      smax = A[ix];
      if (A[ix] != 0.0F) {
        c_ix = c_c + 1;
        d = (iy - exponent) + 2;
        for (ijA = iy; ijA < d; ijA++) {
          A[ijA] += A[c_ix] * -smax;
          c_ix++;
        }
      }

      ix += 3;
      iy += 3;
    }
  }

  isodd = false;
  if (ipiv[0] > 1) {
    isodd = true;
  }

  smax = A[0] * A[4] * A[8];
  if (ipiv[1] > 2) {
    isodd = !isodd;
  }

  if (isodd) {
    smax = -smax;
  }

  s = ddot_r_g_tmp * ddot_r_g_tmp;
  N[0] = B_7[1] * T[2] - B_7[2] * T[1];
  N[1] = B_7[2] * T[0] - B_7[0] * T[2];
  N[2] = B_7[0] * T[1] - B_7[1] * T[0];
  if (absxk < 2.22044605E-16F) {
    absxk = 2.22044605E-16F;
  }

  *kappa = ddot_r_g_tmp / absxk;
  if (s < 2.22044605E-16F) {
    s = 2.22044605E-16F;
  }

  *tau = smax / s;
}

real_T rt_urand_Upu32_Yd_f_pw(uint32_T *u)
{
  uint32_T lo;
  uint32_T hi;

  // Uniform random number generator (random number between 0 and 1)

  // #define IA      16807                      magic multiplier = 7^5
  // #define IM      2147483647                 modulus = 2^31-1
  // #define IQ      127773                     IM div IA
  // #define IR      2836                       IM modulo IA
  // #define S       4.656612875245797e-10      reciprocal of 2^31-1
  // test = IA * (seed % IQ) - IR * (seed/IQ)
  // seed = test < 0 ? (test + IM) : test
  // return (seed*S)

  lo = *u % 127773U * 16807U;
  hi = *u / 127773U * 2836U;
  if (lo < hi) {
    *u = 2147483647U - (hi - lo);
  } else {
    *u = lo - hi;
  }

  return (real_T)*u * 4.6566128752457969E-10;
}

real_T rt_nrand_Upu32_Yd_f_pw(uint32_T *u)
{
  real_T y;
  real_T sr;
  real_T si;

  // Normal (Gaussian) random number generator
  do {
    sr = 2.0 * rt_urand_Upu32_Yd_f_pw(u) - 1.0;
    si = 2.0 * rt_urand_Upu32_Yd_f_pw(u) - 1.0;
    si = sr * sr + si * si;
  } while (si > 1.0);

  y = std::sqrt(-2.0 * std::log(si) / si) * sr;
  return y;
}

// Function for MATLAB Function: '<S121>/n ref norm'
void MatlabControllerClass::leanVectorNormDeriv2_c(const real32_T nn[3], const
  real32_T nn_dt[3], const real32_T nn_dt2[3], real_T n_dt2[3])
{
  real32_T xyz;
  real32_T a_tmp;
  real32_T n_dt2_tmp;
  real32_T xyz_tmp;
  real32_T xyz_tmp_0;
  real32_T xyz_tmp_1;
  real32_T a_tmp_tmp;
  real32_T a_tmp_tmp_0;
  real32_T a_tmp_tmp_1;
  real32_T n_dt2_tmp_0;
  real32_T n_dt2_tmp_1;
  real32_T n_dt2_tmp_2;
  real32_T n_dt2_tmp_3;
  real32_T n_dt2_tmp_4;
  real32_T n_dt2_tmp_5;
  xyz_tmp = nn[1] * nn[1];
  xyz_tmp_0 = nn[0] * nn[0];
  xyz_tmp_1 = nn[2] * nn[2];
  xyz = (xyz_tmp_0 + xyz_tmp) + xyz_tmp_1;
  a_tmp_tmp = 2.0F * nn[1] * nn_dt[1];
  a_tmp_tmp_0 = 2.0F * nn[0] * nn_dt[0];
  a_tmp_tmp_1 = 2.0F * nn[2] * nn_dt[2];
  a_tmp = (a_tmp_tmp_0 + a_tmp_tmp) + a_tmp_tmp_1;
  n_dt2_tmp = std::pow(xyz, 1.5F);
  n_dt2_tmp_0 = 2.0F * nn[1] * nn_dt2[1];
  n_dt2_tmp_1 = nn_dt[1] * nn_dt[1] * 2.0F;
  n_dt2_tmp_2 = 2.0F * nn[0] * nn_dt2[0];
  n_dt2_tmp_3 = nn_dt[0] * nn_dt[0] * 2.0F;
  n_dt2_tmp_4 = 2.0F * nn[2] * nn_dt2[2];
  n_dt2_tmp_5 = nn_dt[2] * nn_dt[2] * 2.0F;
  n_dt2[0] = (a_tmp * a_tmp * 3.0F / (4.0F * std::pow(xyz, 2.5F)) -
              (((((n_dt2_tmp_2 + n_dt2_tmp_3) + n_dt2_tmp_0) + n_dt2_tmp_1) +
                n_dt2_tmp_4) + n_dt2_tmp_5) / (2.0F * n_dt2_tmp)) * nn[0] +
    (nn_dt2[0] / std::sqrt(xyz) - a_tmp * nn_dt[0] / n_dt2_tmp);
  xyz = (xyz_tmp + xyz_tmp_0) + xyz_tmp_1;
  a_tmp = (a_tmp_tmp + a_tmp_tmp_0) + a_tmp_tmp_1;
  n_dt2_tmp = std::pow(xyz, 1.5F);
  n_dt2[1] = (a_tmp * a_tmp * 3.0F / (4.0F * std::pow(xyz, 2.5F)) -
              (((((n_dt2_tmp_0 + n_dt2_tmp_1) + n_dt2_tmp_2) + n_dt2_tmp_3) +
                n_dt2_tmp_4) + n_dt2_tmp_5) / (2.0F * n_dt2_tmp)) * nn[1] +
    (nn_dt2[1] / std::sqrt(xyz) - a_tmp * nn_dt[1] / n_dt2_tmp);
  xyz = (xyz_tmp_1 + xyz_tmp_0) + xyz_tmp;
  a_tmp = (a_tmp_tmp_1 + a_tmp_tmp_0) + a_tmp_tmp;
  n_dt2_tmp = std::pow(xyz, 1.5F);
  n_dt2[2] = (a_tmp * a_tmp * 3.0F / (4.0F * std::pow(xyz, 2.5F)) -
              (((((n_dt2_tmp_4 + n_dt2_tmp_5) + n_dt2_tmp_2) + n_dt2_tmp_3) +
                n_dt2_tmp_0) + n_dt2_tmp_1) / (2.0F * n_dt2_tmp)) * nn[2] +
    (nn_dt2[2] / std::sqrt(xyz) - a_tmp * nn_dt[2] / n_dt2_tmp);
}

// Function for MATLAB Function: '<S69>/caIndiWls'
void MatlabControllerClass::LSQFromQR(const real32_T A_data[], const int32_T
  A_size[2], const real32_T tau_data[], const int32_T jpvt_data[], real32_T B_3
  [8], int32_T rankA, real32_T Y_data[], int32_T *Y_size)
{
  int32_T b_i;
  real32_T wj;
  int32_T b_j;
  int32_T loop_ub;
  int8_T b_idx_0;
  b_idx_0 = (int8_T)A_size[1];
  *Y_size = b_idx_0;
  if (0 <= b_idx_0 - 1) {
    memset(&Y_data[0], 0, b_idx_0 * sizeof(real32_T));
  }

  for (b_j = 0; b_j < A_size[1]; b_j++) {
    if (tau_data[b_j] != 0.0F) {
      wj = B_3[b_j];
      for (loop_ub = b_j + 1; loop_ub + 1 < 9; loop_ub++) {
        wj += A_data[(b_j << 3) + loop_ub] * B_3[loop_ub];
      }

      wj *= tau_data[b_j];
      if (wj != 0.0F) {
        B_3[b_j] -= wj;
        for (loop_ub = b_j + 1; loop_ub + 1 < 9; loop_ub++) {
          B_3[loop_ub] -= A_data[(b_j << 3) + loop_ub] * wj;
        }
      }
    }
  }

  for (loop_ub = 0; loop_ub < rankA; loop_ub++) {
    Y_data[jpvt_data[loop_ub] - 1] = B_3[loop_ub];
  }

  for (loop_ub = rankA - 1; loop_ub + 1 > 0; loop_ub--) {
    b_j = loop_ub << 3;
    Y_data[jpvt_data[loop_ub] - 1] /= A_data[b_j + loop_ub];
    for (b_i = 0; b_i < loop_ub; b_i++) {
      Y_data[jpvt_data[b_i] - 1] -= A_data[b_j + b_i] * Y_data[jpvt_data[loop_ub]
        - 1];
    }
  }
}

// Function for MATLAB Function: '<S69>/caIndiWls'
real32_T MatlabControllerClass::xnrm2(int32_T n, const real32_T x_data[],
  int32_T ix0)
{
  real32_T y;
  real32_T scale;
  int32_T kend;
  real32_T absxk;
  real32_T t;
  int32_T k;
  y = 0.0F;
  scale = 1.29246971E-26F;
  kend = (ix0 + n) - 1;
  for (k = ix0; k <= kend; k++) {
    absxk = std::abs(x_data[k - 1]);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * std::sqrt(y);
}

real32_T rt_hypotf(real32_T u0, real32_T u1)
{
  real32_T y;
  real32_T a;
  real32_T b;
  a = std::abs(u0);
  b = std::abs(u1);
  if (a < b) {
    a /= b;
    y = std::sqrt(a * a + 1.0F) * b;
  } else if (a > b) {
    b /= a;
    y = std::sqrt(b * b + 1.0F) * a;
  } else {
    y = a * 1.41421354F;
  }

  return y;
}

// Function for MATLAB Function: '<S69>/caIndiWls'
void MatlabControllerClass::xzlarf(int32_T m, int32_T n, int32_T iv0, real32_T
  tau, real32_T C_data[], int32_T ic0, real32_T work_data[])
{
  int32_T lastv;
  int32_T lastc;
  int32_T coltop;
  int32_T ix;
  real32_T c;
  int32_T iac;
  int32_T d;
  int32_T b_ia;
  int32_T jy;
  int32_T exitg1;
  boolean_T exitg2;
  if (tau != 0.0F) {
    lastv = m;
    lastc = iv0 + m;
    while ((lastv > 0) && (C_data[lastc - 2] == 0.0F)) {
      lastv--;
      lastc--;
    }

    lastc = n - 1;
    exitg2 = false;
    while ((!exitg2) && (lastc + 1 > 0)) {
      coltop = (lastc << 3) + ic0;
      jy = coltop;
      do {
        exitg1 = 0;
        if (jy <= (coltop + lastv) - 1) {
          if (C_data[jy - 1] != 0.0F) {
            exitg1 = 1;
          } else {
            jy++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    lastc = -1;
  }

  if (lastv > 0) {
    if (lastc + 1 != 0) {
      for (coltop = 0; coltop <= lastc; coltop++) {
        work_data[coltop] = 0.0F;
      }

      coltop = 0;
      jy = (lastc << 3) + ic0;
      for (iac = ic0; iac <= jy; iac += 8) {
        ix = iv0;
        c = 0.0F;
        d = (iac + lastv) - 1;
        for (b_ia = iac; b_ia <= d; b_ia++) {
          c += C_data[b_ia - 1] * C_data[ix - 1];
          ix++;
        }

        work_data[coltop] += c;
        coltop++;
      }
    }

    if (-tau != 0.0F) {
      coltop = ic0 - 1;
      jy = 0;
      for (iac = 0; iac <= lastc; iac++) {
        if (work_data[jy] != 0.0F) {
          c = work_data[jy] * -tau;
          ix = iv0;
          d = lastv + coltop;
          for (b_ia = coltop; b_ia < d; b_ia++) {
            C_data[b_ia] += C_data[ix - 1] * c;
            ix++;
          }
        }

        jy++;
        coltop += 8;
      }
    }
  }
}

// Function for MATLAB Function: '<S69>/caIndiWls'
void MatlabControllerClass::qrsolve(const real32_T A_data[], const int32_T
  A_size[2], const real32_T B_1[8], real32_T Y_data[], int32_T *Y_size)
{
  real32_T b_A_data[32];
  real32_T tau_data[4];
  int32_T jpvt_data[4];
  int32_T n;
  real32_T work_data[4];
  real32_T vn1_data[4];
  real32_T vn2_data[4];
  int32_T nmi;
  int32_T b_n;
  int32_T yk;
  int32_T idxmax;
  int32_T ix;
  real32_T smax;
  real32_T s;
  int32_T b_ix;
  int32_T iy;
  real32_T absxk;
  real32_T t;
  real32_T B_2[8];
  int32_T b_A_size[2];
  int8_T c_idx_0;
  b_A_size[0] = 8;
  b_A_size[1] = A_size[1];
  b_n = A_size[0] * A_size[1] - 1;
  if (0 <= b_n) {
    memcpy(&b_A_data[0], &A_data[0], (b_n + 1) * sizeof(real32_T));
  }

  n = A_size[1];
  if (A_size[1] < 1) {
    b_n = 0;
  } else {
    b_n = A_size[1];
  }

  if (b_n > 0) {
    jpvt_data[0] = 1;
    yk = 1;
    for (nmi = 2; nmi <= b_n; nmi++) {
      yk++;
      jpvt_data[nmi - 1] = yk;
    }
  }

  if (A_size[1] != 0) {
    c_idx_0 = (int8_T)A_size[1];
    if (0 <= c_idx_0 - 1) {
      memset(&work_data[0], 0, c_idx_0 * sizeof(real32_T));
    }

    b_n = 1;
    for (yk = 0; yk < n; yk++) {
      smax = 0.0F;
      s = 1.29246971E-26F;
      for (nmi = b_n; nmi <= b_n + 7; nmi++) {
        absxk = std::abs(A_data[nmi - 1]);
        if (absxk > s) {
          t = s / absxk;
          smax = smax * t * t + 1.0F;
          s = absxk;
        } else {
          t = absxk / s;
          smax += t * t;
        }
      }

      vn1_data[yk] = s * std::sqrt(smax);
      vn2_data[yk] = vn1_data[yk];
      b_n += 8;
    }

    for (b_n = 0; b_n < n; b_n++) {
      iy = b_n << 3;
      yk = iy + b_n;
      nmi = n - b_n;
      if (nmi < 1) {
        idxmax = 0;
      } else {
        idxmax = 1;
        if (nmi > 1) {
          ix = b_n;
          smax = std::abs(vn1_data[b_n]);
          for (b_ix = 2; b_ix <= nmi; b_ix++) {
            ix++;
            s = std::abs(vn1_data[ix]);
            if (s > smax) {
              idxmax = b_ix;
              smax = s;
            }
          }
        }
      }

      ix = (b_n + idxmax) - 1;
      if (ix + 1 != b_n + 1) {
        b_ix = ix << 3;
        for (idxmax = 0; idxmax < 8; idxmax++) {
          smax = b_A_data[b_ix];
          b_A_data[b_ix] = b_A_data[iy];
          b_A_data[iy] = smax;
          b_ix++;
          iy++;
        }

        b_ix = jpvt_data[ix];
        jpvt_data[ix] = jpvt_data[b_n];
        jpvt_data[b_n] = b_ix;
        vn1_data[ix] = vn1_data[b_n];
        vn2_data[ix] = vn2_data[b_n];
      }

      smax = b_A_data[yk];
      tau_data[b_n] = 0.0F;
      s = xnrm2(7 - b_n, b_A_data, yk + 2);
      if (s != 0.0F) {
        s = rt_hypotf(b_A_data[yk], s);
        if (b_A_data[yk] >= 0.0F) {
          s = -s;
        }

        if (std::abs(s) < 9.86076132E-32F) {
          ix = -1;
          b_ix = (yk - b_n) + 8;
          do {
            ix++;
            for (iy = yk + 1; iy < b_ix; iy++) {
              b_A_data[iy] *= 1.01412048E+31F;
            }

            s *= 1.01412048E+31F;
            smax *= 1.01412048E+31F;
          } while (std::abs(s) < 9.86076132E-32F);

          s = rt_hypotf(smax, xnrm2(7 - b_n, b_A_data, yk + 2));
          if (smax >= 0.0F) {
            s = -s;
          }

          tau_data[b_n] = (s - smax) / s;
          smax = 1.0F / (smax - s);
          b_ix = (yk - b_n) + 8;
          for (iy = yk + 1; iy < b_ix; iy++) {
            b_A_data[iy] *= smax;
          }

          for (idxmax = 0; idxmax <= ix; idxmax++) {
            s *= 9.86076132E-32F;
          }

          smax = s;
        } else {
          tau_data[b_n] = (s - b_A_data[yk]) / s;
          smax = 1.0F / (b_A_data[yk] - s);
          ix = (yk - b_n) + 8;
          for (b_ix = yk + 1; b_ix < ix; b_ix++) {
            b_A_data[b_ix] *= smax;
          }

          smax = s;
        }
      }

      b_A_data[yk] = smax;
      if (b_n + 1 < n) {
        smax = b_A_data[yk];
        b_A_data[yk] = 1.0F;
        xzlarf(8 - b_n, nmi - 1, yk + 1, tau_data[b_n], b_A_data, (b_n + ((b_n +
                  1) << 3)) + 1, work_data);
        b_A_data[yk] = smax;
      }

      for (yk = b_n + 1; yk < n; yk++) {
        if (vn1_data[yk] != 0.0F) {
          nmi = (yk << 3) + b_n;
          smax = std::abs(b_A_data[nmi]) / vn1_data[yk];
          smax = 1.0F - smax * smax;
          if (smax < 0.0F) {
            smax = 0.0F;
          }

          s = vn1_data[yk] / vn2_data[yk];
          s = s * s * smax;
          if (s <= 0.000345266977F) {
            vn1_data[yk] = xnrm2(7 - b_n, b_A_data, nmi + 2);
            vn2_data[yk] = vn1_data[yk];
          } else {
            vn1_data[yk] *= std::sqrt(smax);
          }
        }
      }
    }
  }

  n = 0;
  if (b_A_size[1] > 0) {
    while ((n < b_A_size[1]) && (std::abs(b_A_data[(n << 3) + n]) >
            9.53674316E-6F * std::abs(b_A_data[0]))) {
      n++;
    }
  }

  for (b_ix = 0; b_ix < 8; b_ix++) {
    B_2[b_ix] = B_1[b_ix];
  }

  LSQFromQR(b_A_data, b_A_size, tau_data, jpvt_data, B_2, n, Y_data, Y_size);
}

// Function for MATLAB Function: '<S69>/caIndiWls'
void MatlabControllerClass::mldivide(const real32_T A_data[], const int32_T
  A_size[2], const real32_T B_0[8], real32_T Y_data[], int32_T *Y_size)
{
  if (A_size[1] == 0) {
    *Y_size = 0;
  } else {
    qrsolve(A_data, A_size, B_0, Y_data, Y_size);
  }
}

// Function for MATLAB Function: '<S69>/caIndiWls'
boolean_T MatlabControllerClass::any(const boolean_T x_data[], const int32_T
  *x_size)
{
  boolean_T y;
  int32_T ix;
  boolean_T exitg1;
  y = false;
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= *x_size)) {
    if (!x_data[ix - 1]) {
      ix++;
    } else {
      y = true;
      exitg1 = true;
    }
  }

  return y;
}

// Function for MATLAB Function: '<S69>/caIndiWls'
real32_T MatlabControllerClass::wls_alloc(const real32_T B_4[16], const real32_T
  v[4], const real32_T umin[4], const real32_T umax[4], const real32_T Wv[16],
  const real32_T Wu[16], const real32_T ud[4], real32_T gam, real32_T u[4],
  real32_T W[4], real32_T imax)
{
  real32_T iter;
  real32_T gam_sq;
  real32_T A[32];
  real32_T d[8];
  boolean_T i_free[4];
  real32_T A_free_data[32];
  real32_T p_free_data[4];
  real_T p[4];
  real32_T u_opt[4];
  real_T b_data[4];
  int8_T e_data[4];
  int8_T f_data[4];
  int8_T g_data[4];
  int8_T h_data[4];
  int32_T aoffset;
  int32_T b_k;
  int32_T b_aoffset;
  boolean_T x[4];
  real32_T A_tmp[16];
  boolean_T u_opt_data[4];
  real32_T A_tmp_0[16];
  real32_T A_tmp_1[8];
  boolean_T u_opt_0[4];
  int32_T A_free_size[2];
  real_T p_0;
  boolean_T x_0;
  real_T dist_idx_0;
  boolean_T c_idx_0;
  real_T dist_idx_1;
  boolean_T c_idx_1;
  real_T dist_idx_2;
  boolean_T c_idx_2;
  real_T dist_idx_3;
  boolean_T c_idx_3;
  real32_T Wu_0;
  int32_T A_tmp_tmp;
  int32_T A_tmp_tmp_0;
  int32_T A_tmp_tmp_1;
  boolean_T exitg1;
  boolean_T exitg2;
  gam_sq = std::sqrt(gam);
  for (b_k = 0; b_k < 16; b_k++) {
    A_tmp[b_k] = gam_sq * Wv[b_k];
  }

  for (b_k = 0; b_k < 4; b_k++) {
    for (aoffset = 0; aoffset < 4; aoffset++) {
      A_tmp_tmp = b_k << 2;
      A_tmp_tmp_0 = aoffset + A_tmp_tmp;
      A_tmp_0[A_tmp_tmp_0] = 0.0F;
      A_tmp_tmp_1 = A_tmp_tmp + aoffset;
      A_tmp_0[A_tmp_tmp_0] = A_tmp_0[A_tmp_tmp_1] + B_4[A_tmp_tmp] *
        A_tmp[aoffset];
      A_tmp_0[A_tmp_tmp_0] = B_4[A_tmp_tmp + 1] * A_tmp[aoffset + 4] +
        A_tmp_0[A_tmp_tmp_1];
      A_tmp_0[A_tmp_tmp_0] = B_4[A_tmp_tmp + 2] * A_tmp[aoffset + 8] +
        A_tmp_0[A_tmp_tmp_1];
      A_tmp_0[A_tmp_tmp_0] = B_4[A_tmp_tmp + 3] * A_tmp[aoffset + 12] +
        A_tmp_0[A_tmp_tmp_1];
    }
  }

  for (b_k = 0; b_k < 4; b_k++) {
    A_tmp_tmp = b_k << 2;
    A_tmp_tmp_0 = b_k << 3;
    A[A_tmp_tmp_0] = A_tmp_0[A_tmp_tmp];
    A[4 + A_tmp_tmp_0] = Wu[A_tmp_tmp];
    A_tmp_tmp_1 = A_tmp_tmp + 1;
    A[1 + A_tmp_tmp_0] = A_tmp_0[A_tmp_tmp_1];
    A[5 + A_tmp_tmp_0] = Wu[A_tmp_tmp_1];
    A_tmp_tmp_1 = A_tmp_tmp + 2;
    A[2 + A_tmp_tmp_0] = A_tmp_0[A_tmp_tmp_1];
    A[6 + A_tmp_tmp_0] = Wu[A_tmp_tmp_1];
    A_tmp_tmp += 3;
    A[3 + A_tmp_tmp_0] = A_tmp_0[A_tmp_tmp];
    A[7 + A_tmp_tmp_0] = Wu[A_tmp_tmp];
    gam_sq = A_tmp[b_k + 12] * v[3] + (A_tmp[b_k + 8] * v[2] + (A_tmp[b_k + 4] *
      v[1] + A_tmp[b_k] * v[0]));
    Wu_0 = Wu[b_k + 12] * ud[3] + (Wu[b_k + 8] * ud[2] + (Wu[b_k + 4] * ud[1] +
      Wu[b_k] * ud[0]));
    A_tmp_1[b_k] = gam_sq;
    A_tmp_1[b_k + 4] = Wu_0;
  }

  for (b_k = 0; b_k < 8; b_k++) {
    gam_sq = A[b_k + 24] * u[3] + (A[b_k + 16] * u[2] + (A[b_k + 8] * u[1] +
      A[b_k] * u[0]));
    d[b_k] = A_tmp_1[b_k] - gam_sq;
  }

  i_free[0] = (W[0] == 0.0F);
  i_free[1] = (W[1] == 0.0F);
  i_free[2] = (W[2] == 0.0F);
  i_free[3] = (W[3] == 0.0F);
  iter = 1.0F;
  A_tmp_tmp = 0;
  exitg1 = false;
  while ((!exitg1) && (A_tmp_tmp <= (int32_T)imax - 1)) {
    iter = 1.0F + (real32_T)A_tmp_tmp;
    A_tmp_tmp_1 = 0;
    if (i_free[0]) {
      A_tmp_tmp_1 = 1;
    }

    if (i_free[1]) {
      A_tmp_tmp_1++;
    }

    if (i_free[2]) {
      A_tmp_tmp_1++;
    }

    if (i_free[3]) {
      A_tmp_tmp_1++;
    }

    A_tmp_tmp_0 = A_tmp_tmp_1;
    A_tmp_tmp_1 = 0;
    if (i_free[0]) {
      e_data[0] = 1;
      A_tmp_tmp_1 = 1;
    }

    if (i_free[1]) {
      e_data[A_tmp_tmp_1] = 2;
      A_tmp_tmp_1++;
    }

    if (i_free[2]) {
      e_data[A_tmp_tmp_1] = 3;
      A_tmp_tmp_1++;
    }

    if (i_free[3]) {
      e_data[A_tmp_tmp_1] = 4;
    }

    A_free_size[0] = 8;
    A_free_size[1] = A_tmp_tmp_0;
    for (b_k = 0; b_k < A_tmp_tmp_0; b_k++) {
      for (aoffset = 0; aoffset < 8; aoffset++) {
        A_free_data[aoffset + (b_k << 3)] = A[((e_data[b_k] - 1) << 3) + aoffset];
      }
    }

    mldivide(A_free_data, A_free_size, d, p_free_data, &b_aoffset);
    A_tmp_tmp_1 = 0;
    p_0 = 0.0;
    if (i_free[0]) {
      p_0 = p_free_data[0];
      A_tmp_tmp_1 = 1;
    }

    u_opt[0] = u[0] + (real32_T)p_0;
    p[0] = p_0;
    p_0 = 0.0;
    if (i_free[1]) {
      p_0 = p_free_data[A_tmp_tmp_1];
      A_tmp_tmp_1++;
    }

    u_opt[1] = u[1] + (real32_T)p_0;
    p[1] = p_0;
    p_0 = 0.0;
    if (i_free[2]) {
      p_0 = p_free_data[A_tmp_tmp_1];
      A_tmp_tmp_1++;
    }

    u_opt[2] = u[2] + (real32_T)p_0;
    p[2] = p_0;
    p_0 = 0.0;
    if (i_free[3]) {
      p_0 = p_free_data[A_tmp_tmp_1];
    }

    u_opt[3] = u[3] + (real32_T)p_0;
    p[3] = p_0;
    A_tmp_tmp_1 = 0;
    if (i_free[0]) {
      A_tmp_tmp_1 = 1;
    }

    if (i_free[1]) {
      A_tmp_tmp_1++;
    }

    if (i_free[2]) {
      A_tmp_tmp_1++;
    }

    if (i_free[3]) {
      A_tmp_tmp_1++;
    }

    aoffset = A_tmp_tmp_1;
    A_tmp_tmp_1 = 0;
    if (i_free[0]) {
      f_data[0] = 1;
      A_tmp_tmp_1 = 1;
    }

    u_opt_0[0] = ((u_opt[0] < umin[0]) || (u_opt[0] > umax[0]));
    if (i_free[1]) {
      f_data[A_tmp_tmp_1] = 2;
      A_tmp_tmp_1++;
    }

    u_opt_0[1] = ((u_opt[1] < umin[1]) || (u_opt[1] > umax[1]));
    if (i_free[2]) {
      f_data[A_tmp_tmp_1] = 3;
      A_tmp_tmp_1++;
    }

    u_opt_0[2] = ((u_opt[2] < umin[2]) || (u_opt[2] > umax[2]));
    if (i_free[3]) {
      f_data[A_tmp_tmp_1] = 4;
    }

    u_opt_0[3] = ((u_opt[3] < umin[3]) || (u_opt[3] > umax[3]));
    for (b_k = 0; b_k < aoffset; b_k++) {
      u_opt_data[b_k] = u_opt_0[f_data[b_k] - 1];
    }

    if (!any(u_opt_data, &aoffset)) {
      u[0] = u_opt[0];
      u[1] = u_opt[1];
      u[2] = u_opt[2];
      u[3] = u_opt[3];
      if (A_tmp_tmp_0 == 1) {
        for (b_k = 0; b_k < 8; b_k++) {
          A_tmp_1[b_k] = 0.0F;
          for (aoffset = 0; aoffset < A_tmp_tmp_0; aoffset++) {
            A_tmp_1[b_k] += A_free_data[(aoffset << 3) + b_k] *
              p_free_data[aoffset];
          }
        }
      } else if (b_aoffset == 1) {
        for (b_k = 0; b_k < 8; b_k++) {
          A_tmp_1[b_k] = 0.0F;
          for (aoffset = 0; aoffset < A_tmp_tmp_0; aoffset++) {
            A_tmp_1[b_k] += A_free_data[(aoffset << 3) + b_k] *
              p_free_data[aoffset];
          }
        }
      } else {
        for (A_tmp_tmp_1 = 0; A_tmp_tmp_1 < 8; A_tmp_tmp_1++) {
          A_tmp_1[A_tmp_tmp_1] = 0.0F;
        }

        for (A_tmp_tmp_1 = 0; A_tmp_tmp_1 < A_tmp_tmp_0; A_tmp_tmp_1++) {
          b_aoffset = A_tmp_tmp_1 << 3;
          for (b_k = 0; b_k < 8; b_k++) {
            aoffset = b_aoffset + b_k;
            A_tmp_1[b_k] += A[((e_data[aoffset / 8] - 1) << 3) + aoffset % 8] *
              p_free_data[A_tmp_tmp_1];
          }
        }
      }

      for (b_k = 0; b_k < 8; b_k++) {
        d[b_k] -= A_tmp_1[b_k];
      }

      for (A_tmp_tmp_0 = 0; A_tmp_tmp_0 < 4; A_tmp_tmp_0++) {
        p_free_data[A_tmp_tmp_0] = 0.0F;
        for (b_k = 0; b_k < 8; b_k++) {
          p_free_data[A_tmp_tmp_0] += A[(A_tmp_tmp_0 << 3) + b_k] * d[b_k];
        }

        gam_sq = W[A_tmp_tmp_0] * p_free_data[A_tmp_tmp_0];
        x[A_tmp_tmp_0] = (gam_sq >= -2.22044605E-16F);
        u_opt[A_tmp_tmp_0] = gam_sq;
      }

      x_0 = true;
      A_tmp_tmp_1 = 0;
      exitg2 = false;
      while ((!exitg2) && (A_tmp_tmp_1 < 4)) {
        if (!x[A_tmp_tmp_1]) {
          x_0 = false;
          exitg2 = true;
        } else {
          A_tmp_tmp_1++;
        }
      }

      if (x_0) {
        exitg1 = true;
      } else {
        gam_sq = u_opt[0];
        A_tmp_tmp_1 = 0;
        if (u_opt[0] > u_opt[1]) {
          gam_sq = u_opt[1];
          A_tmp_tmp_1 = 1;
        }

        if (gam_sq > u_opt[2]) {
          gam_sq = u_opt[2];
          A_tmp_tmp_1 = 2;
        }

        if (gam_sq > u_opt[3]) {
          A_tmp_tmp_1 = 3;
        }

        W[A_tmp_tmp_1] = 0.0F;
        i_free[A_tmp_tmp_1] = true;
        A_tmp_tmp++;
      }
    } else {
      A_tmp_tmp_1 = 0;
      dist_idx_0 = 1.0;
      x_0 = (p[0] < 0.0);
      c_idx_0 = (p[0] > 0.0);
      if (i_free[0] && x_0) {
        A_tmp_tmp_1 = 1;
      }

      x[0] = x_0;
      dist_idx_1 = 1.0;
      x_0 = (p[1] < 0.0);
      c_idx_1 = (p[1] > 0.0);
      if (i_free[1] && x_0) {
        A_tmp_tmp_1++;
      }

      x[1] = x_0;
      dist_idx_2 = 1.0;
      x_0 = (p[2] < 0.0);
      c_idx_2 = (p[2] > 0.0);
      if (i_free[2] && x_0) {
        A_tmp_tmp_1++;
      }

      x[2] = x_0;
      dist_idx_3 = 1.0;
      x_0 = (p_0 < 0.0);
      c_idx_3 = (p_0 > 0.0);
      if (i_free[3] && x_0) {
        A_tmp_tmp_1++;
      }

      aoffset = A_tmp_tmp_1;
      A_tmp_tmp_1 = 0;
      if (i_free[0] && x[0]) {
        g_data[0] = 1;
        A_tmp_tmp_1 = 1;
      }

      if (i_free[1] && x[1]) {
        g_data[A_tmp_tmp_1] = 2;
        A_tmp_tmp_1++;
      }

      if (i_free[2] && x[2]) {
        g_data[A_tmp_tmp_1] = 3;
        A_tmp_tmp_1++;
      }

      if (i_free[3] && x_0) {
        g_data[A_tmp_tmp_1] = 4;
      }

      for (b_k = 0; b_k < aoffset; b_k++) {
        A_tmp_tmp_1 = g_data[b_k] - 1;
        b_data[b_k] = (umin[A_tmp_tmp_1] - u[A_tmp_tmp_1]) / (real32_T)
          p[A_tmp_tmp_1];
      }

      A_tmp_tmp_1 = 0;
      if (i_free[0] && x[0]) {
        dist_idx_0 = b_data[0];
        A_tmp_tmp_1 = 1;
      }

      if (i_free[1] && x[1]) {
        dist_idx_1 = b_data[A_tmp_tmp_1];
        A_tmp_tmp_1++;
      }

      if (i_free[2] && x[2]) {
        dist_idx_2 = b_data[A_tmp_tmp_1];
        A_tmp_tmp_1++;
      }

      if (i_free[3] && x_0) {
        dist_idx_3 = b_data[A_tmp_tmp_1];
      }

      A_tmp_tmp_1 = 0;
      if (i_free[0] && c_idx_0) {
        A_tmp_tmp_1 = 1;
      }

      if (i_free[1] && c_idx_1) {
        A_tmp_tmp_1++;
      }

      if (i_free[2] && c_idx_2) {
        A_tmp_tmp_1++;
      }

      if (i_free[3] && c_idx_3) {
        A_tmp_tmp_1++;
      }

      aoffset = A_tmp_tmp_1;
      A_tmp_tmp_1 = 0;
      if (i_free[0] && c_idx_0) {
        h_data[0] = 1;
        A_tmp_tmp_1 = 1;
      }

      if (i_free[1] && c_idx_1) {
        h_data[A_tmp_tmp_1] = 2;
        A_tmp_tmp_1++;
      }

      if (i_free[2] && c_idx_2) {
        h_data[A_tmp_tmp_1] = 3;
        A_tmp_tmp_1++;
      }

      if (i_free[3] && c_idx_3) {
        h_data[A_tmp_tmp_1] = 4;
      }

      for (b_k = 0; b_k < aoffset; b_k++) {
        A_tmp_tmp_1 = h_data[b_k] - 1;
        b_data[b_k] = (umax[A_tmp_tmp_1] - u[A_tmp_tmp_1]) / (real32_T)
          p[A_tmp_tmp_1];
      }

      A_tmp_tmp_1 = 0;
      if (i_free[0] && c_idx_0) {
        dist_idx_0 = b_data[0];
        A_tmp_tmp_1 = 1;
      }

      if (i_free[1] && c_idx_1) {
        dist_idx_1 = b_data[A_tmp_tmp_1];
        A_tmp_tmp_1++;
      }

      if (i_free[2] && c_idx_2) {
        dist_idx_2 = b_data[A_tmp_tmp_1];
        A_tmp_tmp_1++;
      }

      if (i_free[3] && c_idx_3) {
        dist_idx_3 = b_data[A_tmp_tmp_1];
      }

      A_tmp_tmp_1 = 0;
      if (dist_idx_0 > dist_idx_1) {
        dist_idx_0 = dist_idx_1;
        A_tmp_tmp_1 = 1;
      }

      if (dist_idx_0 > dist_idx_2) {
        dist_idx_0 = dist_idx_2;
        A_tmp_tmp_1 = 2;
      }

      if (dist_idx_0 > dist_idx_3) {
        dist_idx_0 = dist_idx_3;
        A_tmp_tmp_1 = 3;
      }

      u[0] += (real32_T)(dist_idx_0 * p[0]);
      u[1] += (real32_T)(dist_idx_0 * p[1]);
      u[2] += (real32_T)(dist_idx_0 * p[2]);
      u[3] += (real32_T)(dist_idx_0 * p_0);
      aoffset = (A_tmp_tmp_0 << 3) - 1;
      for (b_k = 0; b_k <= aoffset; b_k++) {
        A_free_data[b_k] *= (real32_T)dist_idx_0;
      }

      if (A_tmp_tmp_0 == 1) {
        for (b_k = 0; b_k < 8; b_k++) {
          A_tmp_1[b_k] = 0.0F;
          for (aoffset = 0; aoffset < 1; aoffset++) {
            A_tmp_1[b_k] += A_free_data[b_k] * p_free_data[0];
          }
        }
      } else if (b_aoffset == 1) {
        for (b_k = 0; b_k < 8; b_k++) {
          A_tmp_1[b_k] = 0.0F;
          for (aoffset = 0; aoffset < A_tmp_tmp_0; aoffset++) {
            A_tmp_1[b_k] += A_free_data[(aoffset << 3) + b_k] *
              p_free_data[aoffset];
          }
        }
      } else {
        for (b_aoffset = 0; b_aoffset < 8; b_aoffset++) {
          A_tmp_1[b_aoffset] = 0.0F;
        }

        for (b_k = 0; b_k < A_tmp_tmp_0; b_k++) {
          aoffset = b_k << 3;
          for (b_aoffset = 0; b_aoffset < 8; b_aoffset++) {
            A_tmp_1[b_aoffset] += A_free_data[aoffset + b_aoffset] *
              p_free_data[b_k];
          }
        }
      }

      for (b_k = 0; b_k < 8; b_k++) {
        d[b_k] -= A_tmp_1[b_k];
      }

      if (p[A_tmp_tmp_1] < 0.0) {
        W[A_tmp_tmp_1] = -1.0F;
      } else if (p[A_tmp_tmp_1] > 0.0) {
        W[A_tmp_tmp_1] = 1.0F;
      } else {
        W[A_tmp_tmp_1] = (real32_T)p[A_tmp_tmp_1];
      }

      i_free[A_tmp_tmp_1] = false;
      A_tmp_tmp++;
    }
  }

  return iter;
}

// Function for MATLAB Function: '<S45>/G1 learn rate'
real32_T MatlabControllerClass::mean(const real32_T x[4])
{
  return (((x[0] + x[1]) + x[2]) + x[3]) / 4.0F;
}

// Model step function
void MatlabControllerClass::step()
{
  real32_T scale;
  real32_T absxk;
  real32_T t;
  real32_T q1_q1;
  real32_T q0_q1;
  real32_T q0_q3;
  real32_T q1_q2;
  real32_T q1_q3;
  real32_T q2_q3;
  real32_T G_omega[16];
  real32_T umax[4];
  real_T d[16];
  dtoSgl_trajectoryStructBus traj;
  uint16_T num_wp;
  real32_T b_data[60];
  real32_T x_data[60];
  real32_T u_data[60];
  real32_T v_data[60];
  real32_T b_w_data[24];
  real32_T b_u_data[24];
  real32_T b_v_data[24];
  int32_T db;
  real_T b_section_idx;
  int32_T b_k;
  real_T learn_rate[4];
  real32_T rtb_Add_a[4];
  real32_T rtb_G2[16];
  real32_T rtb_G1[16];
  real32_T rtb_y_bn[16];
  real32_T rtb_n_b[3];
  real32_T rtb_Sum2_e[3];
  real32_T rtb_q_red_b[4];
  real32_T rtb_n_b_dt2[3];
  real32_T rtb_n_b_dt_p[3];
  real32_T rtb_n_b_c[3];
  real32_T rtb_n_dt2_i[3];
  real32_T rtb_n_dt_o[3];
  real32_T rtb_n[3];
  real32_T rtb_n_b_dt[3];
  real32_T rtb_n_dt2[3];
  real32_T rtb_n_dt[3];
  real32_T rtb_y_dt_cs[3];
  real32_T rtb_omega2_f[3];
  boolean_T rtb_is_finished;
  boolean_T rtb_is_slowing;
  boolean_T rtb_state_isVertPscEnabled;
  boolean_T rtb_state_isAttiCmdEnabled;
  real32_T rtb_M_bg[9];
  real32_T rtb_s_g_dt2_ref[3];
  real32_T rtb_Add1[3];
  real32_T rtb_Sum2_p[9];
  real32_T rtb_y_p[3];
  real32_T rtb_y_dt_g[3];
  real32_T rtb_uvwxb[120];
  real32_T rtb_Sum2_a[4];
  real32_T rtb_y_lm;
  real32_T rtb_Gain6;
  boolean_T rtb_is_intercept_arc;
  boolean_T rtb_is_descent;
  boolean_T rtb_UnitDelay4;
  boolean_T rtb_Compare;
  int32_T i;
  real32_T G_omega_0[16];
  real32_T rtb_n_dt_0[4];
  real_T tmp[3];
  real32_T tmp_data[5];
  real32_T tmp_data_0[5];
  real32_T tmp_data_1[5];
  real32_T tmp_data_2[60];
  real32_T tmp_data_3[10];
  real32_T x_data_0[24];
  real32_T t_0[9];
  real32_T tmp_0[9];
  real32_T tmp_1[9];
  real32_T tmp_2[9];
  real32_T tmp_3[3];
  real32_T tmp_4[16];
  real32_T rtb_y_l[4];
  int32_T loop_ub;
  int32_T b_v_size;
  int32_T tmp_size[2];
  int32_T tmp_size_0[2];
  int32_T tmp_size_1[2];
  real32_T state_vec[6];
  real_T rtb_Output_idx_3;
  real_T rtb_Output_idx_2;
  real_T rtb_Output_idx_1;
  real_T Delta_diag_W_v_idx_3;
  real_T Delta_diag_W_v_idx_1;
  real_T Delta_diag_W_v_idx_0;
  uint16_T varargin_1_idx_1;
  int32_T rtb_G1_tmp;
  const dtoSgl_trajectoryStructBus *Switch;
  boolean_T exitg1;
  boolean_T exitg2;

  // Outputs for Iterator SubSystem: '<S32>/Trajectory from Waypoints' incorporates:
  //   ForIterator: '<S75>/For Iterator'

  // Memory: '<S75>/uvwxbMemory'
  memcpy(&rtb_uvwxb[0], &rtDW.uvwxbMemory_PreviousInput[0], 120U * sizeof
         (real32_T));

  // Memory: '<S75>/AnormAlfaRhoPhiMemory'
  rtb_Add_a[0] = rtDW.AnormAlfaRhoPhiMemory_PreviousI[0];
  rtb_Add_a[1] = rtDW.AnormAlfaRhoPhiMemory_PreviousI[1];
  rtb_Add_a[2] = rtDW.AnormAlfaRhoPhiMemory_PreviousI[2];
  rtb_Add_a[3] = rtDW.AnormAlfaRhoPhiMemory_PreviousI[3];

  // RelationalOperator: '<S83>/Compare' incorporates:
  //   Constant: '<S83>/Constant'
  //   Inport: '<Root>/cmd'
  //   SignalConversion: '<Root>/BusConversion_InsertedFor_cmd_at_outport_0'

  rtb_Compare = (rtU.cmd.mission_change > 0);

  // Memory: '<S75>/StateVecMemory '
  for (i = 0; i < 6; i++) {
    state_vec[i] = rtDW.StateVecMemory_PreviousInput[i];
  }

  // End of Memory: '<S75>/StateVecMemory '

  // MATLAB Function: '<S75>/trajFromWaypoints' incorporates:
  //   Constant: '<S32>/Constant'
  //   Constant: '<S75>/Constant1'
  //   Inport: '<Root>/cmd'
  //   Memory: '<S75>/AnormAlfaRhoPhiMemory'
  //   Memory: '<S75>/TrajMemory'
  //   Memory: '<S75>/uvwxbMemory'
  //   RelationalOperator: '<S81>/FixPt Relational Operator'
  //   SignalConversion: '<Root>/BusConversion_InsertedFor_cmd_at_outport_0'
  //   UnitDelay: '<S81>/Delay Input1'
  //
  //  Block description for '<S81>/Delay Input1':
  //
  //   Store in Global RAM

  traj = rtDW.TrajMemory_PreviousInput;
  scale = state_vec[0];
  rtb_Gain6 = state_vec[1];
  rtb_y_lm = state_vec[2];
  t = state_vec[3];
  q1_q1 = state_vec[4];
  q0_q1 = state_vec[5];
  absxk = rt_roundf(rtDW.TrajMemory_PreviousInput.num_sections_max);
  if (absxk < 65536.0F) {
    if (absxk >= 0.0F) {
      varargin_1_idx_1 = (uint16_T)absxk;
    } else {
      varargin_1_idx_1 = 0U;
    }
  } else {
    varargin_1_idx_1 = MAX_uint16_T;
  }

  num_wp = rtU.cmd.num_waypoints;
  if (rtU.cmd.num_waypoints > varargin_1_idx_1) {
    num_wp = varargin_1_idx_1;
  }

  if (num_wp > 10) {
    num_wp = 10U;
  }

  if (((int32_T)rtb_Compare > (int32_T)rtDW.DelayInput1_DSTATE) && (state_vec[0]
       == 0.0F) && (state_vec[2] == 0.0F)) {
    q1_q1 = 0.0F;
    traj = rtConstP.pooled3;
    if (trajValidateWaypoints(num_wp)) {
      rtb_y_lm = 1.0F;
      scale = 0.0F;
    }
  } else if ((state_vec[2] >= 1.0F) && (state_vec[2] <= 3.0F)) {
    if (state_vec[0] == 0.0F) {
      if (1 > num_wp) {
        loop_ub = 0;
      } else {
        loop_ub = num_wp;
      }

      i = (int32_T)state_vec[2];
      tmp_size_1[0] = 1;
      tmp_size_1[1] = loop_ub;
      for (db = 0; db < loop_ub; db++) {
        tmp_data_3[db] = rtU.cmd.waypoints[((db << 2) + i) - 1];
      }

      polyInterpolationb(tmp_data_3, tmp_size_1, true, b_data, &db, &t);
      ladacLsqrInit(t, true, b_data, &db, x_data, &i, tmp_data_2, &b_k, u_data,
                    &b_v_size, v_data, &loop_ub, &rtb_Add_a[1], &rtb_Add_a[2],
                    &rtb_Add_a[3]);
      scale = 1.0F;
      rtb_Gain6 = (real32_T)db;
      if (0 <= db - 1) {
        memcpy(&rtb_uvwxb[0], &b_data[0], db * sizeof(real32_T));
      }

      if (0 <= b_v_size - 1) {
        memcpy(&rtb_uvwxb[24], &u_data[0], b_v_size * sizeof(real32_T));
      }

      if (0 <= loop_ub - 1) {
        memcpy(&rtb_uvwxb[48], &v_data[0], loop_ub * sizeof(real32_T));
      }

      if (0 <= b_k - 1) {
        memcpy(&rtb_uvwxb[72], &tmp_data_2[0], b_k * sizeof(real32_T));
      }

      if (0 <= i - 1) {
        memcpy(&rtb_uvwxb[96], &x_data[0], i * sizeof(real32_T));
      }

      rtb_Add_a[0] = 0.0F;
      polyInterpolationAx_f(t, true, x_data, &i, tmp_data_2, &db);
      for (i = 0; i < db; i++) {
        x_data[i] = tmp_data_2[i] - b_data[i];
      }

      q0_q1 = norm(x_data, &db);
    } else if (state_vec[0] < 1000.0F) {
      if (1.0F > state_vec[1]) {
        loop_ub = 0;
        b_k = 0;
      } else {
        loop_ub = (int32_T)state_vec[1];
        b_k = (int32_T)state_vec[1];
      }

      if (0 <= loop_ub - 1) {
        memcpy(&b_data[0], &rtDW.uvwxbMemory_PreviousInput[0], loop_ub * sizeof
               (real32_T));
      }

      i = b_k;
      if (0 <= b_k - 1) {
        memcpy(&x_data_0[0], &rtDW.uvwxbMemory_PreviousInput[96], b_k * sizeof
               (real32_T));
      }

      if (1.0F > state_vec[1]) {
        b_k = 0;
      } else {
        b_k = (int32_T)state_vec[1];
      }

      db = b_k;
      if (0 <= b_k - 1) {
        memcpy(&b_w_data[0], &rtDW.uvwxbMemory_PreviousInput[72], b_k * sizeof
               (real32_T));
      }

      if (1.0F > state_vec[1]) {
        b_k = 0;
      } else {
        b_k = (int32_T)state_vec[1];
      }

      if (0 <= b_k - 1) {
        memcpy(&b_u_data[0], &rtDW.uvwxbMemory_PreviousInput[24], b_k * sizeof
               (real32_T));
      }

      if (1.0F > state_vec[1]) {
        b_k = 0;
      } else {
        b_k = (int32_T)state_vec[1];
      }

      b_v_size = b_k;
      if (0 <= b_k - 1) {
        memcpy(&b_v_data[0], &rtDW.uvwxbMemory_PreviousInput[48], b_k * sizeof
               (real32_T));
      }

      rtb_Add_a[0] = rtDW.AnormAlfaRhoPhiMemory_PreviousI[0];
      rtb_Add_a[1] = rtDW.AnormAlfaRhoPhiMemory_PreviousI[1];
      rtb_Add_a[2] = rtDW.AnormAlfaRhoPhiMemory_PreviousI[2];
      rtb_Add_a[3] = rtDW.AnormAlfaRhoPhiMemory_PreviousI[3];
      ladacLsqrIterate(state_vec[3], true, x_data_0, &i, b_w_data, &db, b_u_data,
                       &b_k, b_v_data, &b_v_size, &rtb_Add_a[0], &rtb_Add_a[1],
                       &rtb_Add_a[2], &rtb_Add_a[3]);
      if (0 <= loop_ub - 1) {
        memcpy(&rtb_uvwxb[0], &b_data[0], loop_ub * sizeof(real32_T));
      }

      if (0 <= b_k - 1) {
        memcpy(&rtb_uvwxb[24], &b_u_data[0], b_k * sizeof(real32_T));
      }

      if (0 <= b_v_size - 1) {
        memcpy(&rtb_uvwxb[48], &b_v_data[0], b_v_size * sizeof(real32_T));
      }

      if (0 <= db - 1) {
        memcpy(&rtb_uvwxb[72], &b_w_data[0], db * sizeof(real32_T));
      }

      if (0 <= i - 1) {
        memcpy(&rtb_uvwxb[96], &x_data_0[0], i * sizeof(real32_T));
      }

      scale = state_vec[0] + 1.0F;
      polyInterpolationAx_f(state_vec[3], true, x_data_0, &i, tmp_data_2, &db);
      for (i = 0; i < db; i++) {
        x_data_0[i] = tmp_data_2[i] - b_data[i];
      }

      q0_q1 = norm(x_data_0, &db);
      if ((q0_q1 < 0.001) || (state_vec[5] < q0_q1)) {
        scale = 1000.0F;
      }
    } else {
      if (1.0F > state_vec[1]) {
        i = 0;
      } else {
        i = (int32_T)state_vec[1];
      }

      loop_ub = i - 1;
      if (0 <= loop_ub) {
        memcpy(&x_data[0], &rtDW.uvwxbMemory_PreviousInput[96], (loop_ub + 1) *
               sizeof(real32_T));
      }

      traj.num_sections_set = state_vec[3];
      traj.is_repeated_course = true;
      for (b_k = 0; b_k < (int32_T)t; b_k++) {
        scale = ((1.0F + (real32_T)b_k) - 1.0F) * 6.0F + 1.0F;
        if (rtb_y_lm == 1.0F) {
          if (scale > scale + 5.0F) {
            db = 1;
            i = 0;
          } else {
            db = (int32_T)scale;
            i = (int32_T)(scale + 5.0F);
          }

          loop_ub = i - db;
          for (i = 0; i <= loop_ub; i++) {
            x_data_0[i] = x_data[(db + i) - 1];
          }

          for (i = 0; i < 6; i++) {
            traj.sections[(int32_T)(1.0F + (real32_T)b_k) - 1].pos_x[i] =
              x_data_0[i];
          }
        } else if (rtb_y_lm == 2.0F) {
          if (scale > scale + 5.0F) {
            db = 1;
            i = 0;
          } else {
            db = (int32_T)scale;
            i = (int32_T)(scale + 5.0F);
          }

          loop_ub = i - db;
          for (i = 0; i <= loop_ub; i++) {
            x_data_0[i] = x_data[(db + i) - 1];
          }

          for (i = 0; i < 6; i++) {
            traj.sections[(int32_T)(1.0F + (real32_T)b_k) - 1].pos_y[i] =
              x_data_0[i];
          }
        } else {
          if (rtb_y_lm == 3.0F) {
            if (scale > scale + 5.0F) {
              db = 1;
              i = 0;
            } else {
              db = (int32_T)scale;
              i = (int32_T)(scale + 5.0F);
            }

            loop_ub = i - db;
            for (i = 0; i <= loop_ub; i++) {
              x_data_0[i] = x_data[(db + i) - 1];
            }

            for (i = 0; i < 6; i++) {
              traj.sections[(int32_T)(1.0F + (real32_T)b_k) - 1].pos_z[i] =
                x_data_0[i];
            }
          }
        }
      }

      scale = 0.0F;
      rtb_y_lm = state_vec[2] + 1.0F;
    }
  } else {
    if (state_vec[2] == 4.0F) {
      trajSetArcLength(&traj);
      q1_q1 = 1.0F;
      rtb_y_lm = 0.0F;
      scale = 0.0F;
    }
  }

  state_vec[0] = scale;
  state_vec[1] = rtb_Gain6;
  state_vec[2] = rtb_y_lm;
  state_vec[3] = t;
  state_vec[4] = q1_q1;
  state_vec[5] = q0_q1;

  // Switch: '<S75>/Switch' incorporates:
  //   Constant: '<S75>/Constant1'
  //   MATLAB Function: '<S75>/trajFromWaypoints'

  if (q1_q1 >= 1.0F) {
    Switch = &traj;
  } else {
    Switch = (&rtConstP.pooled3);
  }

  // End of Switch: '<S75>/Switch'

  // Update for Memory: '<S75>/uvwxbMemory'
  memcpy(&rtDW.uvwxbMemory_PreviousInput[0], &rtb_uvwxb[0], 120U * sizeof
         (real32_T));

  // Update for Memory: '<S75>/AnormAlfaRhoPhiMemory'
  rtDW.AnormAlfaRhoPhiMemory_PreviousI[0] = rtb_Add_a[0];
  rtDW.AnormAlfaRhoPhiMemory_PreviousI[1] = rtb_Add_a[1];
  rtDW.AnormAlfaRhoPhiMemory_PreviousI[2] = rtb_Add_a[2];
  rtDW.AnormAlfaRhoPhiMemory_PreviousI[3] = rtb_Add_a[3];

  // Update for UnitDelay: '<S81>/Delay Input1'
  //
  //  Block description for '<S81>/Delay Input1':
  //
  //   Store in Global RAM

  rtDW.DelayInput1_DSTATE = rtb_Compare;

  // Update for Memory: '<S75>/StateVecMemory '
  for (i = 0; i < 6; i++) {
    rtDW.StateVecMemory_PreviousInput[i] = state_vec[i];
  }

  // End of Update for Memory: '<S75>/StateVecMemory '

  // Update for Memory: '<S75>/TrajMemory' incorporates:
  //   MATLAB Function: '<S75>/trajFromWaypoints'

  rtDW.TrajMemory_PreviousInput = traj;

  // End of Outputs for SubSystem: '<S32>/Trajectory from Waypoints'

  // MATLAB Function: '<Root>/MATLAB Function1' incorporates:
  //   Inport: '<Root>/measure'

  scale = 1.29246971E-26F;
  absxk = std::abs(rtU.measure.V_Kg[0]);
  if (absxk > 1.29246971E-26F) {
    rtb_y_lm = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    rtb_y_lm = t * t;
  }

  absxk = std::abs(rtU.measure.V_Kg[1]);
  if (absxk > scale) {
    t = scale / absxk;
    rtb_y_lm = rtb_y_lm * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    rtb_y_lm += t * t;
  }

  absxk = std::abs(rtU.measure.V_Kg[2]);
  if (absxk > scale) {
    t = scale / absxk;
    rtb_y_lm = rtb_y_lm * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    rtb_y_lm += t * t;
  }

  rtb_y_lm = scale * std::sqrt(rtb_y_lm);

  // End of MATLAB Function: '<Root>/MATLAB Function1'

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix' incorporates:
  //   Inport: '<Root>/measure'

  scale = 1.29246971E-26F;
  absxk = std::abs(rtU.measure.q_bg[0]);
  if (absxk > 1.29246971E-26F) {
    q1_q1 = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    q1_q1 = t * t;
  }

  absxk = std::abs(rtU.measure.q_bg[1]);
  if (absxk > scale) {
    t = scale / absxk;
    q1_q1 = q1_q1 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q1_q1 += t * t;
  }

  absxk = std::abs(rtU.measure.q_bg[2]);
  if (absxk > scale) {
    t = scale / absxk;
    q1_q1 = q1_q1 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q1_q1 += t * t;
  }

  absxk = std::abs(rtU.measure.q_bg[3]);
  if (absxk > scale) {
    t = scale / absxk;
    q1_q1 = q1_q1 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q1_q1 += t * t;
  }

  q1_q1 = scale * std::sqrt(q1_q1);
  if (2.22044605E-16F < q1_q1) {
    t = q1_q1;
  } else {
    t = 2.22044605E-16F;
  }

  rtb_Add_a[0] = rtU.measure.q_bg[0] / t;
  rtb_Add_a[1] = rtU.measure.q_bg[1] / t;
  rtb_Add_a[2] = rtU.measure.q_bg[2] / t;
  rtb_Add_a[3] = rtU.measure.q_bg[3] / t;
  t = rtb_Add_a[0] * rtb_Add_a[0];
  q1_q1 = rtb_Add_a[1] * rtb_Add_a[1];
  scale = rtb_Add_a[2] * rtb_Add_a[2];
  absxk = rtb_Add_a[3] * rtb_Add_a[3];
  q0_q1 = rtb_Add_a[0] * rtb_Add_a[1];
  rtb_Gain6 = rtb_Add_a[0] * rtb_Add_a[2];
  q0_q3 = rtb_Add_a[0] * rtb_Add_a[3];
  q1_q2 = rtb_Add_a[1] * rtb_Add_a[2];
  q1_q3 = rtb_Add_a[1] * rtb_Add_a[3];
  q2_q3 = rtb_Add_a[2] * rtb_Add_a[3];
  rtb_M_bg[0] = ((t + q1_q1) - scale) - absxk;
  rtb_M_bg[3] = (q1_q2 + q0_q3) * 2.0F;
  rtb_M_bg[6] = (q1_q3 - rtb_Gain6) * 2.0F;
  rtb_M_bg[1] = (q1_q2 - q0_q3) * 2.0F;
  t -= q1_q1;
  rtb_M_bg[4] = (t + scale) - absxk;
  rtb_M_bg[7] = (q2_q3 + q0_q1) * 2.0F;
  rtb_M_bg[2] = (q1_q3 + rtb_Gain6) * 2.0F;
  rtb_M_bg[5] = (q2_q3 - q0_q1) * 2.0F;
  rtb_M_bg[8] = (t - scale) + absxk;

  // End of MATLAB Function: '<Root>/Quaternions to Rotation Matrix'

  // RelationalOperator: '<S2>/Compare' incorporates:
  //   Constant: '<S2>/Constant'
  //   Inport: '<Root>/cmd'
  //   SignalConversion: '<Root>/BusConversion_InsertedFor_cmd_at_outport_0'

  rtb_Compare = (rtU.cmd.RC_pwm[7] > 1500.0F);

  // Outputs for Enabled SubSystem: '<S3>/fast descent sequencer' incorporates:
  //   EnablePort: '<S11>/Enable'

  if (rtb_Compare) {
    if (!rtDW.fastdescentsequencer_MODE) {
      // InitializeConditions for UnitDelay: '<S11>/Unit Delay'
      rtDW.UnitDelay_DSTATE_l = false;

      // InitializeConditions for UnitDelay: '<S11>/Unit Delay4'
      rtDW.UnitDelay4_DSTATE = false;

      // InitializeConditions for UnitDelay: '<S11>/Unit Delay2'
      rtDW.UnitDelay2_DSTATE = false;

      // InitializeConditions for UnitDelay: '<S11>/Unit Delay3'
      rtDW.UnitDelay3_DSTATE = 0.0F;
      rtDW.fastdescentsequencer_MODE = true;
    }

    // Outputs for Enabled SubSystem: '<S11>/Subsystem2' incorporates:
    //   EnablePort: '<S19>/Enable'

    // UnitDelay: '<S11>/Unit Delay'
    if (rtDW.UnitDelay_DSTATE_l) {
      if (!rtDW.Subsystem2_MODE) {
        rtDW.Subsystem2_MODE = true;
      }

      // MATLAB Function: '<S19>/MATLAB Function' incorporates:
      //   Constant: '<S19>/Constant'
      //   Constant: '<S19>/Constant1'
      //   Constant: '<S19>/a_max'

      t = rtb_y_lm * rtb_y_lm / 19.6199989F;
      rtDW.t_abfang = 3.14159274F * t / (2.0F * rtb_y_lm);
      rtDW.h_abfang = 40.0F + t;
    } else {
      if (rtDW.Subsystem2_MODE) {
        rtDW.Subsystem2_MODE = false;
      }
    }

    // End of UnitDelay: '<S11>/Unit Delay'
    // End of Outputs for SubSystem: '<S11>/Subsystem2'

    // UnitDelay: '<S11>/Unit Delay4'
    rtb_UnitDelay4 = rtDW.UnitDelay4_DSTATE;

    // Outputs for Enabled SubSystem: '<S11>/Duration in Intercept Arc' incorporates:
    //   EnablePort: '<S17>/Enable'

    // UnitDelay: '<S11>/Unit Delay2'
    if (rtDW.UnitDelay2_DSTATE) {
      if (!rtDW.DurationinInterceptArc_MODE) {
        // InitializeConditions for DiscreteIntegrator: '<S17>/Discrete-Time Integrator' 
        rtDW.DiscreteTimeIntegrator_DSTAT_ld = 0.0F;
        rtDW.DiscreteTimeIntegrator_PrevRe_m = 2;
        rtDW.DurationinInterceptArc_MODE = true;
      }

      // DiscreteIntegrator: '<S17>/Discrete-Time Integrator' incorporates:
      //   UnitDelay: '<S11>/Unit Delay4'

      if ((!rtDW.UnitDelay4_DSTATE) && (rtDW.DiscreteTimeIntegrator_PrevRe_m ==
           1)) {
        rtDW.DiscreteTimeIntegrator_DSTAT_ld = 0.0F;
      }

      rtDW.DiscreteTimeIntegrator = rtDW.DiscreteTimeIntegrator_DSTAT_ld;

      // End of DiscreteIntegrator: '<S17>/Discrete-Time Integrator'

      // Update for DiscreteIntegrator: '<S17>/Discrete-Time Integrator' incorporates:
      //   UnitDelay: '<S11>/Unit Delay4'

      rtDW.DiscreteTimeIntegrator_DSTAT_ld += 0.0025F;
      rtDW.DiscreteTimeIntegrator_PrevRe_m = (int8_T)rtDW.UnitDelay4_DSTATE;
    } else {
      if (rtDW.DurationinInterceptArc_MODE) {
        rtDW.DurationinInterceptArc_MODE = false;
      }
    }

    // End of UnitDelay: '<S11>/Unit Delay2'
    // End of Outputs for SubSystem: '<S11>/Duration in Intercept Arc'

    // MATLAB Function: '<S11>/Mission Status' incorporates:
    //   Constant: '<S11>/Constant'
    //   Constant: '<S11>/t_slowing_max'
    //   Constant: '<S11>/v_loiter'
    //   Gain: '<Root>/Gain6'
    //   Inport: '<Root>/measure'
    //   UnitDelay: '<S11>/Unit Delay3'

    t = rtDW.UnitDelay3_DSTATE;
    b_k = 0;
    rtb_is_descent = false;
    rtb_is_intercept_arc = false;
    rtb_is_slowing = false;
    rtb_is_finished = false;
    if (rtDW.UnitDelay3_DSTATE == 0.0F) {
      rtb_is_descent = true;
      b_k = 2;
      if (-rtU.measure.s_Kg[2] < rtDW.h_abfang) {
        t = rtDW.UnitDelay3_DSTATE + 1.0F;
      }

      // Outputs for Enabled SubSystem: '<S11>/Desired Lean Descent' incorporates:
      //   EnablePort: '<S13>/Enable'

      // SignalConversion: '<S13>/OutportBuffer_InsertedFor_lean_des_at_inport_0' incorporates:
      //   Constant: '<S13>/Constant'
      //   Gain: '<Root>/Gain6'
      //   Inport: '<Root>/measure'

      rtDW.Merge_m = -1.57079637F;

      // End of Outputs for SubSystem: '<S11>/Desired Lean Descent'
    } else if (rtDW.UnitDelay3_DSTATE == 1.0F) {
      rtb_is_intercept_arc = true;
      b_k = 2;
      if (-rtU.measure.s_Kg[2] < 40.0F) {
        t = rtDW.UnitDelay3_DSTATE + 1.0F;
      } else {
        if (rtDW.DiscreteTimeIntegrator > rtDW.t_abfang) {
          t = rtDW.UnitDelay3_DSTATE + 1.0F;
        }
      }
    } else if (rtDW.UnitDelay3_DSTATE == 2.0F) {
      rtb_is_slowing = true;
      b_k = 3;
      if (rtb_y_lm < 10.0F) {
        t = rtDW.UnitDelay3_DSTATE + 1.0F;
      } else {
        if (rtDW.DiscreteTimeIntegrator > rtDW.t_abfang + 5.0F) {
          t = rtDW.UnitDelay3_DSTATE + 1.0F;
        }
      }
    } else {
      if (rtDW.UnitDelay3_DSTATE == 3.0F) {
        rtb_is_finished = true;
      }
    }

    // Outputs for Enabled SubSystem: '<S11>/Desired Lean Intercept Arc' incorporates:
    //   EnablePort: '<S14>/Enable'

    if (rtb_is_intercept_arc) {
      // MATLAB Function: '<S14>/MATLAB Function6'
      q1_q1 = rtDW.t_abfang * rtDW.t_abfang * 2.0F;
      if (std::abs(q1_q1) < 2.22044605E-16F) {
        q1_q1 = 2.22044605E-16F;
      }

      rtb_y_lm = rtDW.t_abfang;
      if (std::abs(rtDW.t_abfang) < 2.22044605E-16F) {
        rtb_y_lm = 2.22044605E-16F;
      }

      // SignalConversion: '<S14>/OutportBufferForlean_des' incorporates:
      //   MATLAB Function: '<S14>/MATLAB Function6'

      rtDW.Merge_m = (-(rtDW.DiscreteTimeIntegrator *
                        rtDW.DiscreteTimeIntegrator * 3.14159274F / q1_q1) +
                      3.14159274F * rtDW.DiscreteTimeIntegrator / rtb_y_lm) -
        1.57079637F;
    }

    // End of Outputs for SubSystem: '<S11>/Desired Lean Intercept Arc'

    // Outputs for Enabled SubSystem: '<S11>/Desired Lean slowing' incorporates:
    //   EnablePort: '<S16>/Enable'

    if (rtb_is_slowing) {
      // SignalConversion: '<S16>/OutportBuffer_InsertedFor_lean_des_at_inport_0' incorporates:
      //   Constant: '<S16>/Constant'

      rtDW.Merge_m = 0.3F;
    }

    // End of Outputs for SubSystem: '<S11>/Desired Lean slowing'

    // Outputs for Enabled SubSystem: '<S11>/Desired Lean finished' incorporates:
    //   EnablePort: '<S15>/Enable'

    if (rtb_is_finished) {
      // SignalConversion: '<S15>/OutportBuffer_InsertedFor_lean_des_at_inport_0' incorporates:
      //   Constant: '<S15>/Constant'

      rtDW.Merge_m = 0.0F;
    }

    // End of Outputs for SubSystem: '<S11>/Desired Lean finished'

    // Outputs for Enabled SubSystem: '<S11>/Throttle Descent' incorporates:
    //   EnablePort: '<S20>/Enable'

    if (rtb_is_descent) {
      // SignalConversion: '<S20>/OutportBuffer_InsertedFor_throttle_at_inport_0' incorporates:
      //   Constant: '<S20>/throttle decent'

      rtDW.Merge1_k = 0.7F;
    }

    // End of Outputs for SubSystem: '<S11>/Throttle Descent'

    // Outputs for Enabled SubSystem: '<S11>/Throttle Intercept Arc (Load factor controller)' incorporates:
    //   EnablePort: '<S21>/Enable'

    if (rtb_is_intercept_arc) {
      if (!rtDW.ThrottleInterceptArcLoadfactorc) {
        // InitializeConditions for DiscreteIntegrator: '<S27>/Discrete-Time Integrator' 
        rtDW.DiscreteTimeIntegrator_DSTATE_g = 0.7F;
        rtDW.ThrottleInterceptArcLoadfactorc = true;
      }

      // MATLAB Function: '<S21>/measured specific thrust' incorporates:
      //   Inport: '<Root>/measure'
      //   Sum: '<Root>/Add'

      for (i = 0; i < 3; i++) {
        rtb_y_lm = rtb_M_bg[i + 6];
        rtb_n_dt[i] = (rtU.measure.a_Kg[2] + 9.81F) * rtb_y_lm + (rtb_M_bg[i + 3]
          * rtU.measure.a_Kg[1] + rtb_M_bg[i] * rtU.measure.a_Kg[0]);
        rtb_n_b[i] = rtb_y_lm * 9.81F;
      }

      // Sum: '<S21>/Add1' incorporates:
      //   Constant: '<S21>/Constant'
      //   Constant: '<S21>/Constant1'
      //   DiscreteIntegrator: '<S27>/Discrete-Time Integrator'
      //   Gain: '<S21>/Gain4'
      //   MATLAB Function: '<S21>/MATLAB Function'
      //   MATLAB Function: '<S21>/measured specific thrust'
      //   Sum: '<S21>/Add'

      q1_q1 = ((9.81F * std::cos(rtDW.Merge_m) + 19.6199989F) - (-rtb_n_dt[2] +
                rtb_n_b[2])) * 0.05F + rtDW.DiscreteTimeIntegrator_DSTATE_g;

      // Saturate: '<S21>/Saturation'
      if (q1_q1 > 1.0F) {
        q1_q1 = 1.0F;
      } else {
        if (q1_q1 < 0.0F) {
          q1_q1 = 0.0F;
        }
      }

      // End of Saturate: '<S21>/Saturation'

      // SignalConversion: '<S21>/OutportBufferForthrottle'
      rtDW.Merge1_k = q1_q1;

      // Update for DiscreteIntegrator: '<S27>/Discrete-Time Integrator' incorporates:
      //   Gain: '<S27>/1//T'
      //   Sum: '<S27>/Sum2'

      rtDW.DiscreteTimeIntegrator_DSTATE_g += (q1_q1 -
        rtDW.DiscreteTimeIntegrator_DSTATE_g) * 19.0416317F * 0.0025F;
    } else {
      if (rtDW.ThrottleInterceptArcLoadfactorc) {
        rtDW.ThrottleInterceptArcLoadfactorc = false;
      }
    }

    // End of Outputs for SubSystem: '<S11>/Throttle Intercept Arc (Load factor controller)' 

    // Outputs for Enabled SubSystem: '<S11>/Throttle slowing'
    Throttlefinished(rtb_is_slowing, &rtDW.Merge1_k);

    // End of Outputs for SubSystem: '<S11>/Throttle slowing'

    // Outputs for Enabled SubSystem: '<S11>/Throttle finished'
    Throttlefinished(rtb_is_finished, &rtDW.Merge1_k);

    // End of Outputs for SubSystem: '<S11>/Throttle finished'

    // SignalConversion: '<S11>/OutportBufferForrpyt' incorporates:
    //   Constant: '<S12>/Constant3'
    //   Constant: '<S12>/Constant4'
    //   Gain: '<S12>/Gain2'

    rtDW.Merge[0] = 0.0F;
    rtDW.Merge[1] = 0.318309873F * rtDW.Merge_m;
    rtDW.Merge[2] = 0.0F;
    rtDW.Merge[3] = rtDW.Merge1_k;

    // SignalConversion: '<S11>/OutportBufferForflight_mode' incorporates:
    //   MATLAB Function: '<S11>/Mission Status'

    rtDW.Merge1 = (real32_T)b_k;

    // Update for UnitDelay: '<S11>/Unit Delay'
    rtDW.UnitDelay_DSTATE_l = rtb_is_descent;

    // Update for UnitDelay: '<S11>/Unit Delay4'
    rtDW.UnitDelay4_DSTATE = rtb_is_intercept_arc;

    // Update for UnitDelay: '<S11>/Unit Delay2'
    rtDW.UnitDelay2_DSTATE = rtb_UnitDelay4;

    // Update for UnitDelay: '<S11>/Unit Delay3' incorporates:
    //   MATLAB Function: '<S11>/Mission Status'

    rtDW.UnitDelay3_DSTATE = t;
  } else {
    if (rtDW.fastdescentsequencer_MODE) {
      // Disable for Enabled SubSystem: '<S11>/Subsystem2'
      if (rtDW.Subsystem2_MODE) {
        rtDW.Subsystem2_MODE = false;
      }

      // End of Disable for SubSystem: '<S11>/Subsystem2'

      // Disable for Enabled SubSystem: '<S11>/Duration in Intercept Arc'
      if (rtDW.DurationinInterceptArc_MODE) {
        rtDW.DurationinInterceptArc_MODE = false;
      }

      // End of Disable for SubSystem: '<S11>/Duration in Intercept Arc'

      // Disable for Enabled SubSystem: '<S11>/Throttle Intercept Arc (Load factor controller)' 
      if (rtDW.ThrottleInterceptArcLoadfactorc) {
        rtDW.ThrottleInterceptArcLoadfactorc = false;
      }

      // End of Disable for SubSystem: '<S11>/Throttle Intercept Arc (Load factor controller)' 
      rtDW.fastdescentsequencer_MODE = false;
    }
  }

  // End of Outputs for SubSystem: '<S3>/fast descent sequencer'

  // Logic: '<S3>/Logical Operator'
  rtb_is_intercept_arc = !rtb_Compare;

  // MATLAB Function: '<Root>/MATLAB Function' incorporates:
  //   Inport: '<Root>/cmd'
  //   SignalConversion: '<Root>/BusConversion_InsertedFor_cmd_at_outport_0'

  b_k = 0;
  if (rtU.cmd.RC_pwm[6] >= 1300.0F) {
    if ((rtU.cmd.RC_pwm[6] >= 1300.0F) && (rtU.cmd.RC_pwm[6] < 1700.0F)) {
      b_k = 2;
    } else {
      if (rtU.cmd.RC_pwm[6] >= 1700.0F) {
        b_k = 4;
      }
    }
  }

  // Outputs for Enabled SubSystem: '<S3>/bypass flightmode' incorporates:
  //   EnablePort: '<S9>/Enable'

  if (rtb_is_intercept_arc) {
    // Inport: '<S9>/flightmode_in' incorporates:
    //   MATLAB Function: '<Root>/MATLAB Function'

    rtDW.Merge1 = (real32_T)b_k;
  }

  // End of Outputs for SubSystem: '<S3>/bypass flightmode'

  // MATLAB Function: '<S39>/LindiCopter State Logic' incorporates:
  //   UnitDelay: '<S39>/Unit Delay'

  rtb_is_slowing = true;
  rtb_is_finished = true;
  rtb_state_isVertPscEnabled = true;
  rtb_Compare = true;
  rtb_state_isAttiCmdEnabled = true;
  rtb_UnitDelay4 = true;
  rtb_is_descent = true;
  switch ((int32_T)rtDW.Merge1) {
   case 0:
    rtb_state_isVertPscEnabled = false;
    rtb_Compare = false;
    rtb_state_isAttiCmdEnabled = false;
    rtb_UnitDelay4 = false;
    rtb_is_descent = false;
    break;

   case 1:
    rtb_is_finished = (rtDW.UnitDelay_DSTATE == 0.0F);
    rtb_state_isVertPscEnabled = false;
    rtb_state_isAttiCmdEnabled = false;
    rtb_UnitDelay4 = false;
    rtb_is_descent = false;
    break;

   case 2:
    rtb_is_slowing = false;
    rtb_is_finished = false;
    rtb_state_isVertPscEnabled = false;
    rtb_Compare = false;
    rtb_is_descent = false;
    break;

   case 3:
    rtb_Compare = false;
    rtb_UnitDelay4 = false;
    rtb_is_descent = false;
    break;

   case 4:
    rtb_state_isVertPscEnabled = false;
    rtb_Compare = false;
    rtb_state_isAttiCmdEnabled = false;
    rtb_UnitDelay4 = false;
    break;
  }

  // End of MATLAB Function: '<S39>/LindiCopter State Logic'

  // DiscreteIntegrator: '<S97>/Discrete-Time Integrator y' incorporates:
  //   Inport: '<Root>/measure'

  if (rtDW.DiscreteTimeIntegratory_IC_LOAD != 0) {
    rtDW.DiscreteTimeIntegratory_DSTATE[0] = rtU.measure.s_Kg[0];
    rtDW.DiscreteTimeIntegratory_DSTATE[1] = rtU.measure.s_Kg[1];
    rtDW.DiscreteTimeIntegratory_DSTATE[2] = rtU.measure.s_Kg[2];
  }

  // DiscreteIntegrator: '<S96>/Discrete-Time Integrator y' incorporates:
  //   Inport: '<Root>/measure'

  if (rtDW.DiscreteTimeIntegratory_IC_LO_n != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_a[0] = rtU.measure.V_Kg[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_a[1] = rtU.measure.V_Kg[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_a[2] = rtU.measure.V_Kg[2];
  }

  // Outputs for Enabled SubSystem: '<S3>/bypass stick commands' incorporates:
  //   EnablePort: '<S10>/Enable'

  if (rtb_is_intercept_arc) {
    // Inport: '<S10>/rpyt_in' incorporates:
    //   Inport: '<Root>/cmd'
    //   SignalConversion: '<Root>/BusConversion_InsertedFor_cmd_at_outport_0'

    rtDW.Merge[0] = rtU.cmd.roll;
    rtDW.Merge[1] = rtU.cmd.pitch;
    rtDW.Merge[2] = rtU.cmd.yaw;
    rtDW.Merge[3] = rtU.cmd.thr;
  }

  // End of Outputs for SubSystem: '<S3>/bypass stick commands'

  // Outputs for Enabled SubSystem: '<S4>/NDI position controller for copters reference model' incorporates:
  //   EnablePort: '<S36>/Enable'

  if (rtb_is_finished) {
    if (!rtDW.NDIpositioncontrollerforcopte_k) {
      // InitializeConditions for DiscreteIntegrator: '<S131>/Discrete-Time Integrator' 
      rtDW.DiscreteTimeIntegrator_IC_LOADI = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S36>/Discrete-Time Integrator' 
      rtDW.DiscreteTimeIntegrator_IC_LOA_o = 1U;
      rtDW.NDIpositioncontrollerforcopte_k = true;
    }

    // DiscreteIntegrator: '<S131>/Discrete-Time Integrator' incorporates:
    //   DiscreteIntegrator: '<S96>/Discrete-Time Integrator y'

    if (rtDW.DiscreteTimeIntegrator_IC_LOADI != 0) {
      rtDW.DiscreteTimeIntegrator_DSTATE_l[0] =
        rtDW.DiscreteTimeIntegratory_DSTAT_a[0];
      rtDW.DiscreteTimeIntegrator_DSTATE_l[1] =
        rtDW.DiscreteTimeIntegratory_DSTAT_a[1];
      rtDW.DiscreteTimeIntegrator_DSTATE_l[2] =
        rtDW.DiscreteTimeIntegratory_DSTAT_a[2];
    }

    // Gain: '<S36>/Gain3' incorporates:
    //   Gain: '<Root>/Gain'
    //   MATLAB Function: '<S36>/MATLAB Function1'

    q0_q1 = std::sqrt(rtDW.Merge[0] * rtDW.Merge[0] + -rtDW.Merge[1] *
                      -rtDW.Merge[1]) * 16.7692528F;

    // Gain: '<S131>/1//T' incorporates:
    //   DiscreteIntegrator: '<S131>/Discrete-Time Integrator'
    //   Gain: '<Root>/Gain'
    //   Gain: '<Root>/Gain5'
    //   Lookup_n-D: '<S36>/1-D Lookup Table'
    //   Product: '<S36>/Product'
    //   Sum: '<S131>/Sum2'

    rtb_Sum2_e[0] = (-rtDW.Merge[1] * q0_q1 -
                     rtDW.DiscreteTimeIntegrator_DSTATE_l[0]) * 1.24316764F;
    rtb_Sum2_e[1] = (rtDW.Merge[0] * q0_q1 -
                     rtDW.DiscreteTimeIntegrator_DSTATE_l[1]) * 1.24316764F;
    rtb_Sum2_e[2] = (look1_iflf_binlx(-rtDW.Merge[3],
      rtConstP.uDLookupTable_bp01Data, rtConstP.uDLookupTable_tableData, 2U) -
                     rtDW.DiscreteTimeIntegrator_DSTATE_l[2]) * 1.24316764F;

    // DiscreteIntegrator: '<S131>/Discrete-Time Integrator'
    rtb_y_p[0] = rtDW.DiscreteTimeIntegrator_DSTATE_l[0];

    // Gain: '<S36>/Gain' incorporates:
    //   DiscreteIntegrator: '<S131>/Discrete-Time Integrator'

    rtDW.s_g_ref_dt[0] = rtDW.DiscreteTimeIntegrator_DSTATE_l[0];

    // Saturate: '<S131>/Saturation'
    if (rtb_Sum2_e[0] > 29.7814178F) {
      q1_q3 = 29.7814178F;
    } else if (rtb_Sum2_e[0] < -29.7814178F) {
      q1_q3 = -29.7814178F;
    } else {
      q1_q3 = rtb_Sum2_e[0];
    }

    // Gain: '<S36>/Gain1'
    rtDW.s_g_ref_dt2[0] = q1_q3;

    // Saturate: '<S131>/Saturation'
    rtb_y_dt_g[0] = q1_q3;

    // DiscreteIntegrator: '<S131>/Discrete-Time Integrator'
    rtb_y_p[1] = rtDW.DiscreteTimeIntegrator_DSTATE_l[1];

    // Gain: '<S36>/Gain' incorporates:
    //   DiscreteIntegrator: '<S131>/Discrete-Time Integrator'

    rtDW.s_g_ref_dt[1] = rtDW.DiscreteTimeIntegrator_DSTATE_l[1];

    // Saturate: '<S131>/Saturation'
    if (rtb_Sum2_e[1] > 29.7814178F) {
      q1_q3 = 29.7814178F;
    } else if (rtb_Sum2_e[1] < -29.7814178F) {
      q1_q3 = -29.7814178F;
    } else {
      q1_q3 = rtb_Sum2_e[1];
    }

    // Gain: '<S36>/Gain1'
    rtDW.s_g_ref_dt2[1] = q1_q3;

    // Saturate: '<S131>/Saturation'
    rtb_y_dt_g[1] = q1_q3;

    // DiscreteIntegrator: '<S131>/Discrete-Time Integrator'
    rtb_y_p[2] = rtDW.DiscreteTimeIntegrator_DSTATE_l[2];

    // Gain: '<S36>/Gain' incorporates:
    //   DiscreteIntegrator: '<S131>/Discrete-Time Integrator'

    rtDW.s_g_ref_dt[2] = rtDW.DiscreteTimeIntegrator_DSTATE_l[2];

    // Saturate: '<S131>/Saturation'
    if (rtb_Sum2_e[2] > 8.12739182F) {
      q1_q3 = 8.12739182F;
    } else if (rtb_Sum2_e[2] < -22.410017F) {
      q1_q3 = -22.410017F;
    } else {
      q1_q3 = rtb_Sum2_e[2];
    }

    // Gain: '<S36>/Gain1'
    rtDW.s_g_ref_dt2[2] = q1_q3;

    // DiscreteIntegrator: '<S36>/Discrete-Time Integrator' incorporates:
    //   DiscreteIntegrator: '<S97>/Discrete-Time Integrator y'

    if (rtDW.DiscreteTimeIntegrator_IC_LOA_o != 0) {
      rtDW.DiscreteTimeIntegrator_DSTAT_em[0] =
        rtDW.DiscreteTimeIntegratory_DSTATE[0];
      rtDW.DiscreteTimeIntegrator_DSTAT_em[1] =
        rtDW.DiscreteTimeIntegratory_DSTATE[1];
      rtDW.DiscreteTimeIntegrator_DSTAT_em[2] =
        rtDW.DiscreteTimeIntegratory_DSTATE[2];
    }

    // Update for DiscreteIntegrator: '<S131>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator_IC_LOADI = 0U;

    // Update for DiscreteIntegrator: '<S36>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator_IC_LOA_o = 0U;

    // DiscreteIntegrator: '<S36>/Discrete-Time Integrator'
    rtDW.s_g_ref[0] = rtDW.DiscreteTimeIntegrator_DSTAT_em[0];

    // Update for DiscreteIntegrator: '<S131>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator_DSTATE_l[0] += 0.0025F * rtb_y_dt_g[0];

    // Update for DiscreteIntegrator: '<S36>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator_DSTAT_em[0] += 0.0025F * rtb_y_p[0];

    // DiscreteIntegrator: '<S36>/Discrete-Time Integrator'
    rtDW.s_g_ref[1] = rtDW.DiscreteTimeIntegrator_DSTAT_em[1];

    // Update for DiscreteIntegrator: '<S131>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator_DSTATE_l[1] += 0.0025F * rtb_y_dt_g[1];

    // Update for DiscreteIntegrator: '<S36>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator_DSTAT_em[1] += 0.0025F * rtb_y_p[1];

    // DiscreteIntegrator: '<S36>/Discrete-Time Integrator'
    rtDW.s_g_ref[2] = rtDW.DiscreteTimeIntegrator_DSTAT_em[2];

    // Update for DiscreteIntegrator: '<S131>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator_DSTATE_l[2] += 0.0025F * q1_q3;

    // Update for DiscreteIntegrator: '<S36>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator_DSTAT_em[2] += 0.0025F * rtb_y_p[2];
  } else {
    if (rtDW.NDIpositioncontrollerforcopte_k) {
      rtDW.NDIpositioncontrollerforcopte_k = false;
    }
  }

  // End of Outputs for SubSystem: '<S4>/NDI position controller for copters reference model' 

  // Sqrt: '<S84>/Sqrt' incorporates:
  //   DiscreteIntegrator: '<S96>/Discrete-Time Integrator y'
  //   DotProduct: '<S84>/Dot Product'

  q1_q1 = std::sqrt((rtDW.DiscreteTimeIntegratory_DSTAT_a[0] *
                     rtDW.DiscreteTimeIntegratory_DSTAT_a[0] +
                     rtDW.DiscreteTimeIntegratory_DSTAT_a[1] *
                     rtDW.DiscreteTimeIntegratory_DSTAT_a[1]) +
                    rtDW.DiscreteTimeIntegratory_DSTAT_a[2] *
                    rtDW.DiscreteTimeIntegratory_DSTAT_a[2]);

  // Outputs for Enabled SubSystem: '<S32>/reference' incorporates:
  //   EnablePort: '<S80>/Enable'

  // Outputs for Enabled SubSystem: '<S32>/look ahead' incorporates:
  //   EnablePort: '<S77>/Enable'

  // Outputs for Enabled SubSystem: '<S32>/matching' incorporates:
  //   EnablePort: '<S78>/Enable'

  // Logic: '<S32>/Logical Operator' incorporates:
  //   MATLAB Function: '<S80>/position controller reference from flight path'
  //   UnitDelay: '<S32>/Unit Delay'

  if (rtb_Compare && (rtDW.UnitDelay_DSTATE_c != 0.0F)) {
    // MATLAB Function: '<S78>/flight path matching' incorporates:
    //   Constant: '<S4>/Constant2'
    //   DiscreteIntegrator: '<S97>/Discrete-Time Integrator y'

    trajGetMatchEnhanced(Switch, rtDW.DiscreteTimeIntegratory_DSTATE, &scale,
                         &absxk, &t);
    rtb_y_lm = scale;
    b_section_idx = scale;
    if ((scale > Switch->num_sections_set) || (scale < 1.0F)) {
      b_section_idx = 1.0;
    }

    trajSectionGetFrenetSerretWithG(Switch->sections[(int32_T)b_section_idx - 1]
      .pos_x, Switch->sections[(int32_T)b_section_idx - 1].pos_y,
      Switch->sections[(int32_T)b_section_idx - 1].pos_z, q1_q1, 9.81F, t,
      rtb_y_p, rtb_y_dt_g, rtb_omega2_f, &scale, &absxk);

    // Gain: '<S77>/look ahead distance'
    absxk = 0.268164873F * q1_q1;

    // MATLAB Function: '<S77>/flight path look ahead' incorporates:
    //   MATLAB Function: '<S78>/flight path matching'

    scale = t;
    b_section_idx = -1.0;
    rtb_Output_idx_1 = -1.0;
    q1_q1 = Switch->arc_length;
    if (std::abs(Switch->arc_length) < 2.22044605E-16F) {
      q1_q1 = 2.22044605E-16F;
    }

    t = absxk / q1_q1;
    if (t < 0.0F) {
      t = std::ceil(t);
    } else {
      t = std::floor(t);
    }

    t = std::abs(absxk - t * Switch->arc_length);
    if (absxk < 0.0F) {
      absxk = -1.0F;
    } else {
      if (absxk > 0.0F) {
        absxk = 1.0F;
      }
    }

    if (absxk >= 0.0F) {
      b_k = 0;
      exitg2 = false;
      while ((!exitg2) && (b_k <= (int32_T)(Switch->num_sections_set + 1.0F) - 1))
      {
        q1_q1 = rtb_y_lm + (real32_T)b_k;
        if (q1_q1 > Switch->num_sections_set) {
          q1_q1 -= Switch->num_sections_set;
        }

        trajSectionGetArcLength(Switch->sections[(int32_T)q1_q1 - 1].pos_x,
          Switch->sections[(int32_T)q1_q1 - 1].pos_y, Switch->sections[(int32_T)
          q1_q1 - 1].pos_z, scale, &absxk, &q0_q1);
        rtb_Gain6 = Switch->sections[(int32_T)q1_q1 - 1].arc_length - absxk;
        if (t <= rtb_Gain6) {
          rtb_Output_idx_1 = q1_q1;
          if (std::abs(q0_q1) < 2.22044605E-16F) {
            q0_q1 = 2.22044605E-16F;
          }

          b_section_idx = scale - ((absxk - t) - absxk) / q0_q1;
          trajSectionGetArcLength_j(Switch->sections[(int32_T)q1_q1 - 1].pos_x,
            Switch->sections[(int32_T)q1_q1 - 1].pos_y, Switch->sections
            [(int32_T)q1_q1 - 1].pos_z, b_section_idx, &scale, &q0_q1);
          if (std::abs(q0_q1) < 2.22044605E-16F) {
            q0_q1 = 2.22044605E-16F;
          }

          b_section_idx = (real32_T)b_section_idx - ((scale - t) - absxk) /
            q0_q1;
          trajSectionGetArcLength_j(Switch->sections[(int32_T)q1_q1 - 1].pos_x,
            Switch->sections[(int32_T)q1_q1 - 1].pos_y, Switch->sections
            [(int32_T)q1_q1 - 1].pos_z, b_section_idx, &scale, &q0_q1);
          if (std::abs(q0_q1) < 2.22044605E-16F) {
            q0_q1 = 2.22044605E-16F;
          }

          b_section_idx = (real32_T)b_section_idx - ((scale - t) - absxk) /
            q0_q1;
          exitg2 = true;
        } else {
          t -= rtb_Gain6;
          scale = 0.0F;
          b_k++;
        }
      }
    } else {
      b_k = 0;
      exitg1 = false;
      while ((!exitg1) && (b_k <= (int32_T)(Switch->num_sections_set + 1.0F) - 1))
      {
        q1_q1 = rtb_y_lm - (real32_T)b_k;
        if (q1_q1 < 1.0F) {
          q1_q1 += Switch->num_sections_set;
        }

        trajSectionGetArcLength(Switch->sections[(int32_T)q1_q1 - 1].pos_x,
          Switch->sections[(int32_T)q1_q1 - 1].pos_y, Switch->sections[(int32_T)
          q1_q1 - 1].pos_z, scale, &absxk, &q0_q1);
        if (t <= absxk) {
          rtb_Output_idx_1 = q1_q1;
          if (std::abs(q0_q1) < 2.22044605E-16F) {
            q0_q1 = 2.22044605E-16F;
          }

          b_section_idx = scale - ((t + absxk) - absxk) / q0_q1;
          trajSectionGetArcLength_j(Switch->sections[(int32_T)q1_q1 - 1].pos_x,
            Switch->sections[(int32_T)q1_q1 - 1].pos_y, Switch->sections
            [(int32_T)q1_q1 - 1].pos_z, b_section_idx, &scale, &q0_q1);
          if (std::abs(q0_q1) < 2.22044605E-16F) {
            q0_q1 = 2.22044605E-16F;
          }

          b_section_idx = (real32_T)b_section_idx - ((t + scale) - absxk) /
            q0_q1;
          trajSectionGetArcLength_j(Switch->sections[(int32_T)q1_q1 - 1].pos_x,
            Switch->sections[(int32_T)q1_q1 - 1].pos_y, Switch->sections
            [(int32_T)q1_q1 - 1].pos_z, b_section_idx, &scale, &q0_q1);
          if (std::abs(q0_q1) < 2.22044605E-16F) {
            q0_q1 = 2.22044605E-16F;
          }

          b_section_idx = (real32_T)b_section_idx - ((t + scale) - absxk) /
            q0_q1;
          exitg1 = true;
        } else {
          t -= absxk;
          scale = 1.0F;
          b_k++;
        }
      }
    }

    rtb_y_lm = (real32_T)b_section_idx;

    // MATLAB Function: '<S80>/position controller reference from flight path' incorporates:
    //   MATLAB Function: '<S77>/flight path look ahead'

    b_section_idx = (real32_T)rtb_Output_idx_1;
    if (((real32_T)rtb_Output_idx_1 > Switch->num_sections_set) || ((real32_T)
         rtb_Output_idx_1 < 1.0F)) {
      b_section_idx = 1.0;
    }

    scale = Switch->sections[(int32_T)b_section_idx - 1].pos_x[0];
    absxk = Switch->sections[(int32_T)b_section_idx - 1].pos_y[0];
    q0_q1 = Switch->sections[(int32_T)b_section_idx - 1].pos_z[0];
    for (b_k = 0; b_k < 5; b_k++) {
      scale = rtb_y_lm * scale + Switch->sections[(int32_T)b_section_idx - 1].
        pos_x[b_k + 1];
      absxk = rtb_y_lm * absxk + Switch->sections[(int32_T)b_section_idx - 1].
        pos_y[b_k + 1];
      q0_q1 = rtb_y_lm * q0_q1 + Switch->sections[(int32_T)b_section_idx - 1].
        pos_z[b_k + 1];
    }

    polyder_p(Switch->sections[(int32_T)b_section_idx - 1].pos_x, tmp_data,
              tmp_size_1);
    polyder_p(Switch->sections[(int32_T)b_section_idx - 1].pos_y, tmp_data_0,
              tmp_size);
    polyder_p(Switch->sections[(int32_T)b_section_idx - 1].pos_z, tmp_data_1,
              tmp_size_0);
    q1_q3 = polyVal(tmp_data, tmp_size_1, (real_T)rtb_y_lm);

    // MATLAB Function: '<S80>/position controller reference from flight path'
    rtb_y_p[0] = q1_q3;
    q2_q3 = polyVal(tmp_data_0, tmp_size, (real_T)rtb_y_lm);

    // MATLAB Function: '<S80>/position controller reference from flight path'
    rtb_y_p[1] = q2_q3;
    q0_q3 = polyVal(tmp_data_1, tmp_size_0, (real_T)rtb_y_lm);

    // MATLAB Function: '<S80>/position controller reference from flight path' incorporates:
    //   Inport: '<Root>/cmd'
    //   Selector: '<S32>/Selector'
    //   SignalConversion: '<Root>/BusConversion_InsertedFor_cmd_at_outport_0'

    rtb_y_p[2] = q0_q3;
    t = norm_e(rtb_y_p);
    if (2.22044605E-16F >= t) {
      t = 2.22044605E-16F;
    }

    trajSectionGetFrenetSerretWit_i(Switch->sections[(int32_T)b_section_idx - 1]
      .pos_x, Switch->sections[(int32_T)b_section_idx - 1].pos_y,
      Switch->sections[(int32_T)b_section_idx - 1].pos_z, rtU.cmd.waypoints[3],
      (real_T)rtb_y_lm, rtb_y_dt_g, rtb_omega2_f, rtb_s_g_dt2_ref, &rtb_Gain6,
      &q1_q1);
    q1_q1 = rtU.cmd.waypoints[3] * rtU.cmd.waypoints[3];

    // SignalConversion: '<S80>/BusConversion_InsertedFor_reference_at_inport_0' incorporates:
    //   Inport: '<Root>/cmd'
    //   MATLAB Function: '<S80>/position controller reference from flight path'
    //   Selector: '<S32>/Selector'
    //   SignalConversion: '<Root>/BusConversion_InsertedFor_cmd_at_outport_0'

    rtDW.s_g_ref_dt[0] = q1_q3 / t * rtU.cmd.waypoints[3];

    // SignalConversion: '<S80>/BusConversion_InsertedFor_reference_at_inport_0' incorporates:
    //   MATLAB Function: '<S80>/position controller reference from flight path'

    rtDW.s_g_ref_dt2[0] = rtb_Gain6 * rtb_s_g_dt2_ref[0] * q1_q1;

    // SignalConversion: '<S80>/BusConversion_InsertedFor_reference_at_inport_0' incorporates:
    //   Inport: '<Root>/cmd'
    //   MATLAB Function: '<S80>/position controller reference from flight path'
    //   Selector: '<S32>/Selector'
    //   SignalConversion: '<Root>/BusConversion_InsertedFor_cmd_at_outport_0'

    rtDW.s_g_ref_dt[1] = q2_q3 / t * rtU.cmd.waypoints[3];

    // SignalConversion: '<S80>/BusConversion_InsertedFor_reference_at_inport_0' incorporates:
    //   MATLAB Function: '<S80>/position controller reference from flight path'

    rtDW.s_g_ref_dt2[1] = rtb_Gain6 * rtb_s_g_dt2_ref[1] * q1_q1;

    // SignalConversion: '<S80>/BusConversion_InsertedFor_reference_at_inport_0' incorporates:
    //   Inport: '<Root>/cmd'
    //   MATLAB Function: '<S80>/position controller reference from flight path'
    //   Selector: '<S32>/Selector'
    //   SignalConversion: '<Root>/BusConversion_InsertedFor_cmd_at_outport_0'

    rtDW.s_g_ref_dt[2] = q0_q3 / t * rtU.cmd.waypoints[3];

    // SignalConversion: '<S80>/BusConversion_InsertedFor_reference_at_inport_0' incorporates:
    //   MATLAB Function: '<S80>/position controller reference from flight path'

    rtDW.s_g_ref_dt2[2] = rtb_Gain6 * rtb_s_g_dt2_ref[2] * q1_q1;

    // SignalConversion: '<S80>/BusConversion_InsertedFor_reference_at_inport_0' incorporates:
    //   MATLAB Function: '<S80>/position controller reference from flight path'

    rtDW.s_g_ref[0] = scale;
    rtDW.s_g_ref[1] = absxk;
    rtDW.s_g_ref[2] = q0_q1;
  }

  // End of Logic: '<S32>/Logical Operator'
  // End of Outputs for SubSystem: '<S32>/matching'
  // End of Outputs for SubSystem: '<S32>/look ahead'
  // End of Outputs for SubSystem: '<S32>/reference'

  // Outputs for Enabled SubSystem: '<S4>/NDI position controller for copters with reference input' incorporates:
  //   EnablePort: '<S37>/Enable'

  if (rtb_is_slowing) {
    if (!rtDW.NDIpositioncontrollerforcopters) {
      // InitializeConditions for DiscreteIntegrator: '<S134>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_L_b2 = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S134>/Discrete-Time Integrator y_dt' 
      for (i = 0; i < 9; i++) {
        rtDW.DiscreteTimeIntegratory_dt_DS_e[i] = 0.0F;
      }

      // End of InitializeConditions for DiscreteIntegrator: '<S134>/Discrete-Time Integrator y_dt' 
      rtDW.NDIpositioncontrollerforcopters = true;
    }

    // DiscreteIntegrator: '<S134>/Discrete-Time Integrator y'
    if (rtDW.DiscreteTimeIntegratory_IC_L_b2 != 0) {
      rtDW.DiscreteTimeIntegratory_DSTAT_l[0] = rtDW.s_g_ref[0];
      rtDW.DiscreteTimeIntegratory_DSTAT_l[3] = rtDW.s_g_ref_dt[0];
      rtDW.DiscreteTimeIntegratory_DSTAT_l[6] = rtDW.s_g_ref_dt2[0];
      rtDW.DiscreteTimeIntegratory_DSTAT_l[1] = rtDW.s_g_ref[1];
      rtDW.DiscreteTimeIntegratory_DSTAT_l[4] = rtDW.s_g_ref_dt[1];
      rtDW.DiscreteTimeIntegratory_DSTAT_l[7] = rtDW.s_g_ref_dt2[1];
      rtDW.DiscreteTimeIntegratory_DSTAT_l[2] = rtDW.s_g_ref[2];
      rtDW.DiscreteTimeIntegratory_DSTAT_l[5] = rtDW.s_g_ref_dt[2];
      rtDW.DiscreteTimeIntegratory_DSTAT_l[8] = rtDW.s_g_ref_dt2[2];
    }

    // Sum: '<S134>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S134>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S134>/Discrete-Time Integrator y_dt'
    //   Gain: '<S134>/2*d//omega'
    //   Sum: '<S134>/Sum3'

    rtb_Sum2_p[0] = rtDW.s_g_ref[0] - (0.268164873F *
      rtDW.DiscreteTimeIntegratory_dt_DS_e[0] +
      rtDW.DiscreteTimeIntegratory_DSTAT_l[0]);
    rtb_Sum2_p[3] = rtDW.s_g_ref_dt[0] - (0.268164873F *
      rtDW.DiscreteTimeIntegratory_dt_DS_e[3] +
      rtDW.DiscreteTimeIntegratory_DSTAT_l[3]);
    rtb_Sum2_p[6] = rtDW.s_g_ref_dt2[0] - (0.268164873F *
      rtDW.DiscreteTimeIntegratory_dt_DS_e[6] +
      rtDW.DiscreteTimeIntegratory_DSTAT_l[6]);

    // Sum: '<S133>/Add' incorporates:
    //   DiscreteIntegrator: '<S134>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S94>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S96>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S97>/Discrete-Time Integrator y'

    rtDW.Add[0] = rtDW.DiscreteTimeIntegratory_DSTAT_l[0] -
      rtDW.DiscreteTimeIntegratory_DSTATE[0];
    rtDW.Add[3] = rtDW.DiscreteTimeIntegratory_DSTAT_l[3] -
      rtDW.DiscreteTimeIntegratory_DSTAT_a[0];
    rtDW.Add[6] = rtDW.DiscreteTimeIntegratory_DSTAT_l[6] -
      rtDW.DiscreteTimeIntegratory_DSTA_al[0];

    // Sum: '<S134>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S134>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S134>/Discrete-Time Integrator y_dt'
    //   Gain: '<S134>/2*d//omega'
    //   Sum: '<S134>/Sum3'

    rtb_Sum2_p[1] = rtDW.s_g_ref[1] - (0.268164873F *
      rtDW.DiscreteTimeIntegratory_dt_DS_e[1] +
      rtDW.DiscreteTimeIntegratory_DSTAT_l[1]);
    rtb_Sum2_p[4] = rtDW.s_g_ref_dt[1] - (0.268164873F *
      rtDW.DiscreteTimeIntegratory_dt_DS_e[4] +
      rtDW.DiscreteTimeIntegratory_DSTAT_l[4]);
    rtb_Sum2_p[7] = rtDW.s_g_ref_dt2[1] - (0.268164873F *
      rtDW.DiscreteTimeIntegratory_dt_DS_e[7] +
      rtDW.DiscreteTimeIntegratory_DSTAT_l[7]);

    // Sum: '<S133>/Add' incorporates:
    //   DiscreteIntegrator: '<S134>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S94>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S96>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S97>/Discrete-Time Integrator y'

    rtDW.Add[1] = rtDW.DiscreteTimeIntegratory_DSTAT_l[1] -
      rtDW.DiscreteTimeIntegratory_DSTATE[1];
    rtDW.Add[4] = rtDW.DiscreteTimeIntegratory_DSTAT_l[4] -
      rtDW.DiscreteTimeIntegratory_DSTAT_a[1];
    rtDW.Add[7] = rtDW.DiscreteTimeIntegratory_DSTAT_l[7] -
      rtDW.DiscreteTimeIntegratory_DSTA_al[1];

    // Sum: '<S134>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S134>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S134>/Discrete-Time Integrator y_dt'
    //   Gain: '<S134>/2*d//omega'
    //   Sum: '<S134>/Sum3'

    rtb_Sum2_p[2] = rtDW.s_g_ref[2] - (0.268164873F *
      rtDW.DiscreteTimeIntegratory_dt_DS_e[2] +
      rtDW.DiscreteTimeIntegratory_DSTAT_l[2]);
    rtb_Sum2_p[5] = rtDW.s_g_ref_dt[2] - (0.268164873F *
      rtDW.DiscreteTimeIntegratory_dt_DS_e[5] +
      rtDW.DiscreteTimeIntegratory_DSTAT_l[5]);
    rtb_Sum2_p[8] = rtDW.s_g_ref_dt2[2] - (0.268164873F *
      rtDW.DiscreteTimeIntegratory_dt_DS_e[8] +
      rtDW.DiscreteTimeIntegratory_DSTAT_l[8]);

    // Sum: '<S133>/Add' incorporates:
    //   DiscreteIntegrator: '<S134>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S94>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S96>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S97>/Discrete-Time Integrator y'

    rtDW.Add[2] = rtDW.DiscreteTimeIntegratory_DSTAT_l[2] -
      rtDW.DiscreteTimeIntegratory_DSTATE[2];
    rtDW.Add[5] = rtDW.DiscreteTimeIntegratory_DSTAT_l[5] -
      rtDW.DiscreteTimeIntegratory_DSTAT_a[2];
    rtDW.Add[8] = rtDW.DiscreteTimeIntegratory_DSTAT_l[8] -
      rtDW.DiscreteTimeIntegratory_DSTA_al[2];

    // Switch: '<S135>/Switch2' incorporates:
    //   RelationalOperator: '<S135>/LowerRelop1'
    //   RelationalOperator: '<S135>/UpperRelop'
    //   Switch: '<S135>/Switch'

    if (rtDW.Add[0] > 13.2792645F) {
      absxk = 13.2792645F;
    } else if (rtDW.Add[0] < -13.2792645F) {
      // Switch: '<S135>/Switch'
      absxk = -13.2792645F;
    } else {
      absxk = rtDW.Add[0];
    }

    // Sum: '<S133>/Add1' incorporates:
    //   Gain: '<S133>/Gain'
    //   Gain: '<S133>/Gain3'
    //   Gain: '<S133>/Gain4'

    rtb_n_b_dt2[0] = (3.55246162F * absxk + 4.01875639F * rtDW.Add[3]) +
      0.74909091F * rtDW.Add[6];

    // Switch: '<S135>/Switch2' incorporates:
    //   RelationalOperator: '<S135>/LowerRelop1'
    //   RelationalOperator: '<S135>/UpperRelop'
    //   Switch: '<S135>/Switch'

    if (rtDW.Add[1] > 13.2792645F) {
      absxk = 13.2792645F;
    } else if (rtDW.Add[1] < -13.2792645F) {
      // Switch: '<S135>/Switch'
      absxk = -13.2792645F;
    } else {
      absxk = rtDW.Add[1];
    }

    // Sum: '<S133>/Add1' incorporates:
    //   Gain: '<S133>/Gain'
    //   Gain: '<S133>/Gain3'
    //   Gain: '<S133>/Gain4'

    rtb_n_b_dt2[1] = (3.55246162F * absxk + 4.01875639F * rtDW.Add[4]) +
      0.74909091F * rtDW.Add[7];

    // Switch: '<S135>/Switch2' incorporates:
    //   RelationalOperator: '<S135>/LowerRelop1'
    //   RelationalOperator: '<S135>/UpperRelop'
    //   Switch: '<S135>/Switch'

    if (rtDW.Add[2] > 13.2792645F) {
      absxk = 13.2792645F;
    } else if (rtDW.Add[2] < -13.2792645F) {
      // Switch: '<S135>/Switch'
      absxk = -13.2792645F;
    } else {
      absxk = rtDW.Add[2];
    }

    // Sum: '<S133>/Add1' incorporates:
    //   Gain: '<S133>/Gain'
    //   Gain: '<S133>/Gain3'
    //   Gain: '<S133>/Gain4'

    rtb_n_b_dt2[2] = (3.55246162F * absxk + 4.01875639F * rtDW.Add[5]) +
      0.74909091F * rtDW.Add[8];

    // RelationalOperator: '<S136>/LowerRelop1'
    t = rtb_n_b_dt2[0];

    // Switch: '<S136>/Switch' incorporates:
    //   RelationalOperator: '<S136>/LowerRelop1'
    //   RelationalOperator: '<S136>/UpperRelop'

    if (rtb_n_b_dt2[0] < -22.3360634F) {
      t = -22.3360634F;
    }

    // Switch: '<S136>/Switch2' incorporates:
    //   RelationalOperator: '<S136>/LowerRelop1'

    if (rtb_n_b_dt2[0] > 22.3360634F) {
      t = 22.3360634F;
    }

    // Sum: '<S37>/Add1'
    rtDW.nu[0] = rtDW.s_g_ref_dt2[0] + t;

    // SignalConversion: '<S37>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' 
    rtDW.s_g_ref_f[0] = rtDW.s_g_ref[0];

    // SignalConversion: '<S37>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S97>/Discrete-Time Integrator y'

    rtDW.s_g[0] = rtDW.DiscreteTimeIntegratory_DSTATE[0];

    // SignalConversion: '<S37>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S96>/Discrete-Time Integrator y'

    rtDW.s_g_dt[0] = rtDW.DiscreteTimeIntegratory_DSTAT_a[0];

    // SignalConversion: '<S37>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S94>/Discrete-Time Integrator y'

    rtDW.s_g_dt2[0] = rtDW.DiscreteTimeIntegratory_DSTA_al[0];

    // Switch: '<S136>/Switch' incorporates:
    //   RelationalOperator: '<S136>/UpperRelop'

    rtb_n_b_dt2[0] = t;

    // RelationalOperator: '<S136>/LowerRelop1'
    t = rtb_n_b_dt2[1];

    // Switch: '<S136>/Switch' incorporates:
    //   RelationalOperator: '<S136>/LowerRelop1'
    //   RelationalOperator: '<S136>/UpperRelop'

    if (rtb_n_b_dt2[1] < -22.3360634F) {
      t = -22.3360634F;
    }

    // Switch: '<S136>/Switch2' incorporates:
    //   RelationalOperator: '<S136>/LowerRelop1'

    if (rtb_n_b_dt2[1] > 22.3360634F) {
      t = 22.3360634F;
    }

    // Sum: '<S37>/Add1'
    rtDW.nu[1] = rtDW.s_g_ref_dt2[1] + t;

    // SignalConversion: '<S37>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' 
    rtDW.s_g_ref_f[1] = rtDW.s_g_ref[1];

    // SignalConversion: '<S37>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S97>/Discrete-Time Integrator y'

    rtDW.s_g[1] = rtDW.DiscreteTimeIntegratory_DSTATE[1];

    // SignalConversion: '<S37>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S96>/Discrete-Time Integrator y'

    rtDW.s_g_dt[1] = rtDW.DiscreteTimeIntegratory_DSTAT_a[1];

    // SignalConversion: '<S37>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S94>/Discrete-Time Integrator y'

    rtDW.s_g_dt2[1] = rtDW.DiscreteTimeIntegratory_DSTA_al[1];

    // Switch: '<S136>/Switch' incorporates:
    //   RelationalOperator: '<S136>/UpperRelop'

    rtb_n_b_dt2[1] = t;

    // RelationalOperator: '<S136>/LowerRelop1'
    t = rtb_n_b_dt2[2];

    // Switch: '<S136>/Switch' incorporates:
    //   RelationalOperator: '<S136>/LowerRelop1'
    //   RelationalOperator: '<S136>/UpperRelop'

    if (rtb_n_b_dt2[2] < -22.3360634F) {
      t = -22.3360634F;
    }

    // Switch: '<S136>/Switch2' incorporates:
    //   RelationalOperator: '<S136>/LowerRelop1'

    if (rtb_n_b_dt2[2] > 22.3360634F) {
      t = 22.3360634F;
    }

    // Sum: '<S37>/Add1'
    rtDW.nu[2] = rtDW.s_g_ref_dt2[2] + t;

    // SignalConversion: '<S37>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' 
    rtDW.s_g_ref_f[2] = rtDW.s_g_ref[2];

    // SignalConversion: '<S37>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S97>/Discrete-Time Integrator y'

    rtDW.s_g[2] = rtDW.DiscreteTimeIntegratory_DSTATE[2];

    // SignalConversion: '<S37>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S96>/Discrete-Time Integrator y'

    rtDW.s_g_dt[2] = rtDW.DiscreteTimeIntegratory_DSTAT_a[2];

    // SignalConversion: '<S37>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S94>/Discrete-Time Integrator y'

    rtDW.s_g_dt2[2] = rtDW.DiscreteTimeIntegratory_DSTA_al[2];

    // Switch: '<S136>/Switch' incorporates:
    //   RelationalOperator: '<S136>/UpperRelop'

    rtb_n_b_dt2[2] = t;

    // Update for DiscreteIntegrator: '<S134>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S134>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_IC_L_b2 = 0U;
    for (i = 0; i < 9; i++) {
      rtDW.DiscreteTimeIntegratory_DSTAT_l[i] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_DS_e[i];

      // Update for DiscreteIntegrator: '<S134>/Discrete-Time Integrator y_dt' incorporates:
      //   Gain: '<S134>/omega^2'

      rtDW.DiscreteTimeIntegratory_dt_DS_e[i] += 55.6232338F * rtb_Sum2_p[i] *
        0.0025F;
    }

    // End of Update for DiscreteIntegrator: '<S134>/Discrete-Time Integrator y' 
  } else {
    if (rtDW.NDIpositioncontrollerforcopters) {
      rtDW.NDIpositioncontrollerforcopters = false;
    }
  }

  // End of Outputs for SubSystem: '<S4>/NDI position controller for copters with reference input' 

  // DiscreteIntegrator: '<S98>/Discrete-Time Integrator y_dt'
  rtb_y_p[0] = rtDW.DiscreteTimeIntegratory_dt_DSTA[0];
  rtb_y_p[1] = rtDW.DiscreteTimeIntegratory_dt_DSTA[1];
  rtb_y_p[2] = rtDW.DiscreteTimeIntegratory_dt_DSTA[2];

  // DiscreteIntegrator: '<S95>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_LO_b != 0) {
    for (i = 0; i < 9; i++) {
      rtDW.DiscreteTimeIntegratory_DSTAT_p[i] = rtb_M_bg[i];
    }
  }

  for (i = 0; i < 9; i++) {
    rtb_Sum2_p[i] = rtDW.DiscreteTimeIntegratory_DSTAT_p[i];
  }

  // Outputs for Enabled SubSystem: '<S4>/Pilot Reduced Attitude Commands' incorporates:
  //   EnablePort: '<S38>/Enable'

  if (rtb_state_isAttiCmdEnabled) {
    // MATLAB Function: '<S38>/MATLAB Function' incorporates:
    //   Gain: '<Root>/Gain'

    rtb_y_lm = std::atan2(rtDW.Merge[0], -rtDW.Merge[1]);
    t = std::sqrt(rtDW.Merge[0] * rtDW.Merge[0] + rtDW.Merge[1] * rtDW.Merge[1]);

    // MATLAB Function: '<S38>/MATLAB Function1' incorporates:
    //   Gain: '<S38>/Gain1'

    scale = 3.14159274F * t;
    q1_q1 = std::sin(scale);

    // SignalConversion: '<S38>/BusConversion_InsertedFor_red_atti_des_at_inport_0' incorporates:
    //   MATLAB Function: '<S38>/MATLAB Function1'

    rtDW.n_g_des[0] = q1_q1 * std::cos(rtb_y_lm);
    rtDW.n_g_des[1] = q1_q1 * std::sin(rtb_y_lm);
    rtDW.n_g_des[2] = -std::cos(scale);

    // SignalConversion: '<S38>/BusConversion_InsertedFor_red_atti_des_at_inport_0' 
    rtDW.lean_dir_angle_des = rtb_y_lm;

    // SignalConversion: '<S38>/BusConversion_InsertedFor_red_atti_des_at_inport_0' 
    rtDW.cmd_lean_angle_01 = t;
  }

  // End of Outputs for SubSystem: '<S4>/Pilot Reduced Attitude Commands'

  // Outputs for Enabled SubSystem: '<S44>/Copter Random Excitation' incorporates:
  //   EnablePort: '<S46>/Enable'

  if (rtb_is_descent) {
    if (!rtDW.CopterRandomExcitation_MODE) {
      // InitializeConditions for RandomNumber: '<S56>/White Noise'
      rtDW.RandSeed[0] = 1529675776U;
      rtDW.NextOutput[0] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[0]);

      // InitializeConditions for DiscreteIntegrator: '<S57>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_DSTAT_d[0] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S57>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_h[0] = 0.0F;

      // InitializeConditions for RandomNumber: '<S56>/White Noise'
      rtDW.RandSeed[1] = 1529741312U;
      rtDW.NextOutput[1] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[1]);

      // InitializeConditions for DiscreteIntegrator: '<S57>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_DSTAT_d[1] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S57>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_h[1] = 0.0F;

      // InitializeConditions for RandomNumber: '<S56>/White Noise'
      rtDW.RandSeed[2] = 1529806848U;
      rtDW.NextOutput[2] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[2]);

      // InitializeConditions for DiscreteIntegrator: '<S57>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_DSTAT_d[2] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S57>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_h[2] = 0.0F;

      // InitializeConditions for RandomNumber: '<S56>/White Noise'
      rtDW.RandSeed[3] = 1529872384U;
      rtDW.NextOutput[3] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[3]);

      // InitializeConditions for DiscreteIntegrator: '<S57>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_DSTAT_d[3] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S57>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_h[3] = 0.0F;
      rtDW.CopterRandomExcitation_MODE = true;
    }

    // Sum: '<S57>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S57>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S57>/Discrete-Time Integrator y_dt'
    //   Gain: '<S46>/Gain4'
    //   Gain: '<S56>/Output'
    //   Product: '<S46>/Product'
    //   Product: '<S57>/Product2'
    //   RandomNumber: '<S56>/White Noise'
    //   Sum: '<S57>/Sum3'

    rtb_Add_a[0] = (real32_T)(3.1622776955095868 * rtDW.NextOutput[0] * 3.0) *
      0.162743419F - (rtDW.DiscreteTimeIntegratory_dt_DS_h[0] * 0.0529708415F +
                      rtDW.DiscreteTimeIntegratory_DSTAT_d[0]);
    rtb_Add_a[1] = (real32_T)(3.1622776955095868 * rtDW.NextOutput[1] * 3.0) *
      0.162743419F - (rtDW.DiscreteTimeIntegratory_dt_DS_h[1] * 0.0529708415F +
                      rtDW.DiscreteTimeIntegratory_DSTAT_d[1]);
    rtb_Add_a[2] = (real32_T)(3.1622776955095868 * rtDW.NextOutput[2] * 3.0) *
      0.162743419F - (rtDW.DiscreteTimeIntegratory_dt_DS_h[2] * 0.0529708415F +
                      rtDW.DiscreteTimeIntegratory_DSTAT_d[2]);
    rtb_Add_a[3] = (real32_T)(3.1622776955095868 * rtDW.NextOutput[3] * 3.0) *
      0.162743419F - (rtDW.DiscreteTimeIntegratory_dt_DS_h[3] * 0.0529708415F +
                      rtDW.DiscreteTimeIntegratory_DSTAT_d[3]);

    // Gain: '<S46>/Gain' incorporates:
    //   DiscreteIntegrator: '<S57>/Discrete-Time Integrator y'

    rtDW.Gain[0] = 3.5F * rtDW.DiscreteTimeIntegratory_DSTAT_d[0];
    rtDW.Gain[1] = 3.5F * rtDW.DiscreteTimeIntegratory_DSTAT_d[1];
    rtDW.Gain[2] = 3.5F * rtDW.DiscreteTimeIntegratory_DSTAT_d[2];

    // Gain: '<S46>/Gain1' incorporates:
    //   DiscreteIntegrator: '<S57>/Discrete-Time Integrator y_dt'
    //   Gain: '<S46>/Gain3'

    rtDW.Gain1 = 0.15F * rtDW.DiscreteTimeIntegratory_dt_DS_h[3] * 0.427200407F;

    // Update for RandomNumber: '<S56>/White Noise'
    rtDW.NextOutput[0] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[0]);

    // Update for DiscreteIntegrator: '<S57>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S57>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_d[0] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_h[0];

    // Update for DiscreteIntegrator: '<S57>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S57>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_DS_h[0] += rtb_Add_a[0] * 1425.5625F *
      0.0025F;

    // Update for RandomNumber: '<S56>/White Noise'
    rtDW.NextOutput[1] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[1]);

    // Update for DiscreteIntegrator: '<S57>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S57>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_d[1] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_h[1];

    // Update for DiscreteIntegrator: '<S57>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S57>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_DS_h[1] += rtb_Add_a[1] * 1425.5625F *
      0.0025F;

    // Update for RandomNumber: '<S56>/White Noise'
    rtDW.NextOutput[2] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[2]);

    // Update for DiscreteIntegrator: '<S57>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S57>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_d[2] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_h[2];

    // Update for DiscreteIntegrator: '<S57>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S57>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_DS_h[2] += rtb_Add_a[2] * 1425.5625F *
      0.0025F;

    // Update for RandomNumber: '<S56>/White Noise'
    rtDW.NextOutput[3] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[3]);

    // Update for DiscreteIntegrator: '<S57>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S57>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_d[3] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_h[3];

    // Update for DiscreteIntegrator: '<S57>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S57>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_DS_h[3] += rtb_Add_a[3] * 1425.5625F *
      0.0025F;
  } else {
    if (rtDW.CopterRandomExcitation_MODE) {
      // Disable for Outport: '<S46>/yaw_rate_excite'
      rtDW.Gain1 = 0.0F;

      // Disable for Outport: '<S46>/s_g_dt2_excite'
      rtDW.Gain[0] = 0.0F;
      rtDW.Gain[1] = 0.0F;
      rtDW.Gain[2] = 0.0F;
      rtDW.CopterRandomExcitation_MODE = false;
    }
  }

  // End of Outputs for SubSystem: '<S44>/Copter Random Excitation'

  // Sum: '<S4>/Add1'
  rtb_Add1[2] = rtDW.Gain[2] + rtDW.nu[2];

  // Outputs for Enabled SubSystem: '<S4>/Accelerations to Reduced Attitude and Thrust' incorporates:
  //   EnablePort: '<S29>/Enable'

  // Logic: '<S4>/Logical Operator' incorporates:
  //   Logic: '<S4>/Logical Operator1'
  //   Logic: '<S4>/Logical Operator2'
  //   MATLAB Function: '<S29>/INDI Copter Acc 2 Lean Vector'

  if ((!rtb_state_isAttiCmdEnabled) && (!rtb_state_isVertPscEnabled)) {
    // MATLAB Function: '<S29>/INDI Copter Acc 2 Lean Vector' incorporates:
    //   Constant: '<S4>/Constant2'
    //   DiscreteIntegrator: '<S94>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S95>/Discrete-Time Integrator y'
    //   Sum: '<S4>/Add1'

    rtb_omega2_f[0] = rtDW.DiscreteTimeIntegratory_DSTA_al[0];
    rtb_omega2_f[1] = rtDW.DiscreteTimeIntegratory_DSTA_al[1];
    rtb_omega2_f[2] = rtDW.DiscreteTimeIntegratory_DSTA_al[2] - 9.81F;
    q1_q1 = 0.0F;
    for (b_k = 0; b_k < 3; b_k++) {
      q1_q3 = -rtDW.DiscreteTimeIntegratory_DSTAT_p[3 * b_k + 2];
      q1_q1 += q1_q3 * rtb_omega2_f[b_k];
      rtb_y_dt_g[b_k] = q1_q3;
    }

    scale = 1.29246971E-26F;
    q1_q3 = ((rtDW.Gain[0] + rtDW.nu[0]) - rtDW.DiscreteTimeIntegratory_DSTA_al
             [0]) + rtb_y_dt_g[0] * q1_q1;
    absxk = std::abs(q1_q3);
    if (absxk > 1.29246971E-26F) {
      rtb_y_lm = 1.0F;
      scale = absxk;
    } else {
      t = absxk / 1.29246971E-26F;
      rtb_y_lm = t * t;
    }

    rtb_y_dt_g[0] = q1_q3;

    // MATLAB Function: '<S29>/INDI Copter Acc 2 Lean Vector' incorporates:
    //   DiscreteIntegrator: '<S94>/Discrete-Time Integrator y'
    //   Sum: '<S4>/Add1'

    q1_q3 = ((rtDW.Gain[1] + rtDW.nu[1]) - rtDW.DiscreteTimeIntegratory_DSTA_al
             [1]) + rtb_y_dt_g[1] * q1_q1;
    absxk = std::abs(q1_q3);
    if (absxk > scale) {
      t = scale / absxk;
      rtb_y_lm = rtb_y_lm * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      rtb_y_lm += t * t;
    }

    rtb_y_dt_g[1] = q1_q3;

    // MATLAB Function: '<S29>/INDI Copter Acc 2 Lean Vector' incorporates:
    //   DiscreteIntegrator: '<S94>/Discrete-Time Integrator y'

    q1_q3 = rtb_y_dt_g[2] * q1_q1 + (rtb_Add1[2] -
      rtDW.DiscreteTimeIntegratory_DSTA_al[2]);
    absxk = std::abs(q1_q3);
    if (absxk > scale) {
      t = scale / absxk;
      rtb_y_lm = rtb_y_lm * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      rtb_y_lm += t * t;
    }

    rtb_y_lm = scale * std::sqrt(rtb_y_lm);
    q1_q1 = rtb_y_lm;
    if (rtb_y_lm < 2.22044605E-16F) {
      q1_q1 = 2.22044605E-16F;
    }

    t = 1.0F / q1_q1;
    q1_q1 = t * rtb_y_dt_g[0];

    // SignalConversion: '<S29>/BusConversion_InsertedFor_red_atti_des_at_inport_0' 
    rtDW.n_g_des[0] = q1_q1;
    rtb_y_dt_g[0] = q1_q1;

    // MATLAB Function: '<S29>/INDI Copter Acc 2 Lean Vector'
    q1_q1 = t * rtb_y_dt_g[1];

    // SignalConversion: '<S29>/BusConversion_InsertedFor_red_atti_des_at_inport_0' 
    rtDW.n_g_des[1] = q1_q1;
    rtb_y_dt_g[1] = q1_q1;

    // MATLAB Function: '<S29>/INDI Copter Acc 2 Lean Vector'
    q1_q1 = t * q1_q3;

    // SignalConversion: '<S29>/BusConversion_InsertedFor_red_atti_des_at_inport_0' 
    rtDW.n_g_des[2] = q1_q1;

    // MATLAB Function: '<S29>/MATLAB Function4'
    if (1.0F > -q1_q1) {
      q1_q1 = -q1_q1;
    } else {
      q1_q1 = 1.0F;
    }

    // SignalConversion: '<S29>/BusConversion_InsertedFor_red_atti_des_at_inport_0' incorporates:
    //   MATLAB Function: '<S29>/MATLAB Function4'

    rtDW.lean_dir_angle_des = std::atan2(rtb_y_dt_g[1], rtb_y_dt_g[0]);

    // MATLAB Function: '<S29>/MATLAB Function4'
    if (-1.0F >= q1_q1) {
      q1_q1 = -1.0F;
    }

    q1_q1 = std::acos(q1_q1);

    // Saturate: '<S29>/Saturation1'
    if (q1_q1 > 3.14159274F) {
      q1_q1 = 3.14159274F;
    } else {
      if (q1_q1 < 0.0F) {
        q1_q1 = 0.0F;
      }
    }

    // End of Saturate: '<S29>/Saturation1'

    // SignalConversion: '<S29>/BusConversion_InsertedFor_red_atti_des_at_inport_0' incorporates:
    //   Gain: '<S29>/Gain1'

    rtDW.cmd_lean_angle_01 = 0.318309873F * q1_q1;

    // SignalConversion: '<S29>/OutportBufferForT_spec_des' incorporates:
    //   MATLAB Function: '<S29>/INDI Copter Acc 2 Lean Vector'

    rtDW.Merge1_e = rtb_y_lm;
  }

  // End of Logic: '<S4>/Logical Operator'
  // End of Outputs for SubSystem: '<S4>/Accelerations to Reduced Attitude and Thrust' 

  // Gain: '<S108>/lean_angle_max'
  scale = 3.14159274F * rtDW.cmd_lean_angle_01;

  // MATLAB Function: '<S108>/lean angles 2 lean vector' incorporates:
  //   Gain: '<S108>/lean_angle_max'

  q1_q1 = std::sin(scale);
  rtb_y_dt_g[0] = q1_q1 * std::cos(rtDW.lean_dir_angle_des);
  rtb_y_dt_g[1] = q1_q1 * std::sin(rtDW.lean_dir_angle_des);
  rtb_y_dt_g[2] = -std::cos(scale);

  // DiscreteIntegrator: '<S121>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_LO_i != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_m[0] = rtb_y_dt_g[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_m[1] = rtb_y_dt_g[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_m[2] = rtb_y_dt_g[2];
  }

  // MATLAB Function: '<S121>/n ref norm' incorporates:
  //   DiscreteIntegrator: '<S121>/Discrete-Time Integrator y'

  scale = 1.29246971E-26F;
  absxk = std::abs(rtDW.DiscreteTimeIntegratory_DSTAT_m[0]);
  if (absxk > 1.29246971E-26F) {
    rtb_y_lm = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    rtb_y_lm = t * t;
  }

  // Gain: '<S121>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S121>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S121>/Discrete-Time Integrator y_dt'
  //   Gain: '<S121>/2*d//omega'
  //   Sum: '<S121>/Sum2'
  //   Sum: '<S121>/Sum3'

  rtb_y_dt_g[0] = (rtb_y_dt_g[0] - (0.188708603F *
    rtDW.DiscreteTimeIntegratory_dt_DS_o[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_m[0])) * 112.325035F;

  // MATLAB Function: '<S121>/n ref norm' incorporates:
  //   DiscreteIntegrator: '<S121>/Discrete-Time Integrator y'

  absxk = std::abs(rtDW.DiscreteTimeIntegratory_DSTAT_m[1]);
  if (absxk > scale) {
    t = scale / absxk;
    rtb_y_lm = rtb_y_lm * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    rtb_y_lm += t * t;
  }

  // Gain: '<S121>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S121>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S121>/Discrete-Time Integrator y_dt'
  //   Gain: '<S121>/2*d//omega'
  //   Sum: '<S121>/Sum2'
  //   Sum: '<S121>/Sum3'

  rtb_y_dt_g[1] = (rtb_y_dt_g[1] - (0.188708603F *
    rtDW.DiscreteTimeIntegratory_dt_DS_o[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_m[1])) * 112.325035F;
  q1_q3 = (rtb_y_dt_g[2] - (0.188708603F * rtDW.DiscreteTimeIntegratory_dt_DS_o
            [2] + rtDW.DiscreteTimeIntegratory_DSTAT_m[2])) * 112.325035F;

  // MATLAB Function: '<S121>/n ref norm' incorporates:
  //   DiscreteIntegrator: '<S121>/Discrete-Time Integrator y'

  absxk = std::abs(rtDW.DiscreteTimeIntegratory_DSTAT_m[2]);
  if (absxk > scale) {
    t = scale / absxk;
    rtb_y_lm = rtb_y_lm * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    rtb_y_lm += t * t;
  }

  // Gain: '<S121>/omega^2' incorporates:
  //   Sum: '<S121>/Sum2'

  rtb_y_dt_g[2] = q1_q3;

  // MATLAB Function: '<S121>/n ref norm' incorporates:
  //   DiscreteIntegrator: '<S121>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S121>/Discrete-Time Integrator y_dt'

  rtb_y_lm = scale * std::sqrt(rtb_y_lm);
  if (rtb_y_lm < 2.22044605E-16F) {
    rtb_y_lm = 2.22044605E-16F;
  }

  rtb_n[0] = rtDW.DiscreteTimeIntegratory_DSTAT_m[0] / rtb_y_lm;
  rtb_n[1] = rtDW.DiscreteTimeIntegratory_DSTAT_m[1] / rtb_y_lm;
  rtb_n[2] = rtDW.DiscreteTimeIntegratory_DSTAT_m[2] / rtb_y_lm;
  q2_q3 = rtDW.DiscreteTimeIntegratory_DSTAT_m[1] *
    rtDW.DiscreteTimeIntegratory_DSTAT_m[1];
  scale = rtDW.DiscreteTimeIntegratory_DSTAT_m[2] *
    rtDW.DiscreteTimeIntegratory_DSTAT_m[2];
  q0_q1 = rtDW.DiscreteTimeIntegratory_DSTAT_m[0] *
    rtDW.DiscreteTimeIntegratory_DSTAT_m[0];
  rtb_Gain6 = q0_q1 + q2_q3;
  q1_q1 = std::pow(rtb_Gain6 + scale, 1.5F);
  rtb_y_lm = q1_q1;
  absxk = std::abs(q1_q1);
  if (absxk < 2.22044605E-16F) {
    rtb_y_lm = 2.22044605E-16F;
  }

  t = 1.0F / rtb_y_lm;
  t_0[0] = (q2_q3 + scale) * t;
  rtb_y_lm = -rtDW.DiscreteTimeIntegratory_DSTAT_m[0] *
    rtDW.DiscreteTimeIntegratory_DSTAT_m[1] * t;
  t_0[3] = rtb_y_lm;
  q2_q3 = -rtDW.DiscreteTimeIntegratory_DSTAT_m[0] *
    rtDW.DiscreteTimeIntegratory_DSTAT_m[2];
  q0_q3 = q2_q3 * t;
  t_0[6] = q0_q3;
  t_0[1] = rtb_y_lm;
  t_0[4] = (q0_q1 + scale) * t;
  rtb_y_lm = -rtDW.DiscreteTimeIntegratory_DSTAT_m[1] *
    rtDW.DiscreteTimeIntegratory_DSTAT_m[2] * t;
  t_0[7] = rtb_y_lm;
  t_0[2] = q0_q3;
  t_0[5] = rtb_y_lm;
  t_0[8] = rtb_Gain6 * t;
  for (i = 0; i < 3; i++) {
    rtb_n_dt_o[i] = t_0[i + 6] * rtDW.DiscreteTimeIntegratory_dt_DS_o[2] +
      (t_0[i + 3] * rtDW.DiscreteTimeIntegratory_dt_DS_o[1] + t_0[i] *
       rtDW.DiscreteTimeIntegratory_dt_DS_o[0]);
  }

  if (absxk < 2.22044605E-16F) {
    q1_q1 = 2.22044605E-16F;
  }

  rtb_n_dt_o[2] = ((q2_q3 * rtDW.DiscreteTimeIntegratory_dt_DS_o[0] + q0_q1 *
                    rtDW.DiscreteTimeIntegratory_dt_DS_o[2]) +
                   (rtDW.DiscreteTimeIntegratory_DSTAT_m[1] *
                    rtDW.DiscreteTimeIntegratory_dt_DS_o[2] -
                    rtDW.DiscreteTimeIntegratory_DSTAT_m[2] *
                    rtDW.DiscreteTimeIntegratory_dt_DS_o[1]) *
                   rtDW.DiscreteTimeIntegratory_DSTAT_m[1]) * (1.0F / q1_q1);
  leanVectorNormDeriv2_c(rtDW.DiscreteTimeIntegratory_DSTAT_m,
    rtDW.DiscreteTimeIntegratory_dt_DS_o, rtb_y_dt_g, tmp);
  rtb_n_dt2_i[0] = (real32_T)tmp[0];
  rtb_n_dt2_i[1] = (real32_T)tmp[1];
  rtb_n_dt2_i[2] = (real32_T)tmp[2];

  // DiscreteIntegrator: '<S122>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_LO_a != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_e[0] = rtb_n[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_e[1] = rtb_n[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_e[2] = rtb_n[2];
  }

  rtb_Sum2_e[0] = rtDW.DiscreteTimeIntegratory_DSTAT_e[0];

  // DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt'
  rtb_y_dt_cs[0] = rtDW.DiscreteTimeIntegratory_dt_DS_j[0];

  // Gain: '<S122>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt'
  //   Gain: '<S122>/2*d//omega'
  //   Sum: '<S122>/Sum2'
  //   Sum: '<S122>/Sum3'

  rtb_omega2_f[0] = (rtb_n[0] - (0.0794562623F *
    rtDW.DiscreteTimeIntegratory_dt_DS_j[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_e[0])) * 633.583374F;

  // DiscreteIntegrator: '<S122>/Discrete-Time Integrator y'
  rtb_Sum2_e[1] = rtDW.DiscreteTimeIntegratory_DSTAT_e[1];

  // DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt'
  rtb_y_dt_cs[1] = rtDW.DiscreteTimeIntegratory_dt_DS_j[1];

  // Gain: '<S122>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt'
  //   Gain: '<S122>/2*d//omega'
  //   Sum: '<S122>/Sum2'
  //   Sum: '<S122>/Sum3'

  rtb_omega2_f[1] = (rtb_n[1] - (0.0794562623F *
    rtDW.DiscreteTimeIntegratory_dt_DS_j[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_e[1])) * 633.583374F;

  // DiscreteIntegrator: '<S122>/Discrete-Time Integrator y'
  rtb_Sum2_e[2] = rtDW.DiscreteTimeIntegratory_DSTAT_e[2];

  // DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt'
  rtb_y_dt_cs[2] = rtDW.DiscreteTimeIntegratory_dt_DS_j[2];

  // Gain: '<S122>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt'
  //   Gain: '<S122>/2*d//omega'
  //   Sum: '<S122>/Sum2'
  //   Sum: '<S122>/Sum3'

  rtb_omega2_f[2] = (rtb_n[2] - (0.0794562623F *
    rtDW.DiscreteTimeIntegratory_dt_DS_j[2] +
    rtDW.DiscreteTimeIntegratory_DSTAT_e[2])) * 633.583374F;

  // MATLAB Function: '<S122>/n ref norm'
  nrefnorm(rtb_omega2_f, rtb_y_dt_cs, rtb_Sum2_e, rtb_n_dt2, rtb_n_dt,
           rtb_s_g_dt2_ref);

  // DiscreteIntegrator: '<S98>/Discrete-Time Integrator y' incorporates:
  //   Inport: '<Root>/measure'

  if (rtDW.DiscreteTimeIntegratory_IC_LO_d != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_k[0] = rtU.measure.omega_Kb[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_k[1] = rtU.measure.omega_Kb[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_k[2] = rtU.measure.omega_Kb[2];
  }

  rtb_y_dt_cs[0] = rtDW.DiscreteTimeIntegratory_DSTAT_k[0];
  rtb_y_dt_cs[1] = rtDW.DiscreteTimeIntegratory_DSTAT_k[1];
  rtb_y_dt_cs[2] = rtDW.DiscreteTimeIntegratory_DSTAT_k[2];

  // MATLAB Function: '<S108>/Lean Vector Derivative Trafo Delay'
  LeanVectorDerivativeTrafo(rtb_s_g_dt2_ref, rtb_n_dt, rtb_n_dt2, rtb_Sum2_p,
    rtb_y_dt_cs, rtb_y_p, rtb_n_b, rtb_n_b_dt, rtb_Sum2_e);

  // MATLAB Function: '<S108>/Lean Vector Derivative Trafo'
  LeanVectorDerivativeTrafo(rtb_n, rtb_n_dt_o, rtb_n_dt2_i, rtb_Sum2_p,
    rtb_y_dt_cs, rtb_y_p, rtb_n_b_c, rtb_n_b_dt_p, rtb_n_b_dt2);

  // MATLAB Function: '<S108>/MATLAB Function'
  rtb_y_dt_cs[0] = rtb_n_b[0];
  rtb_y_dt_cs[1] = rtb_n_b[1];
  if (rtb_n_b[2] > 0.0F) {
    if (rtb_n_b[2] > 0.999) {
      scale = 0.0F;
      rtb_n_b[0] = 0.0F;
      t = -1.0F;
      rtb_n_b[1] = 0.0F;
    } else {
      scale = 1.29246971E-26F;
      absxk = std::abs(rtb_n_b[0]);
      if (absxk > 1.29246971E-26F) {
        q1_q1 = 1.0F;
        scale = absxk;
      } else {
        t = absxk / 1.29246971E-26F;
        q1_q1 = t * t;
      }

      absxk = std::abs(rtb_n_b[1]);
      if (absxk > scale) {
        t = scale / absxk;
        q1_q1 = q1_q1 * t * t + 1.0F;
        scale = absxk;
      } else {
        t = absxk / scale;
        q1_q1 += t * t;
      }

      q1_q1 = scale * std::sqrt(q1_q1);
      if (q1_q1 < 2.22044605E-16F) {
        q1_q1 = 2.22044605E-16F;
      }

      scale = rtb_n_b[0] / q1_q1;
      t = rtb_n_b[1] / q1_q1;
    }

    rtb_y_dt_cs[0] = 2.0F * scale - rtb_n_b[0];
    rtb_n_b_dt[0] = -rtb_n_b_dt[0];
    rtb_Sum2_e[0] = -rtb_Sum2_e[0];
    rtb_y_dt_cs[1] = 2.0F * t - rtb_n_b[1];
    rtb_n_b_dt[1] = -rtb_n_b_dt[1];
    rtb_Sum2_e[1] = -rtb_Sum2_e[1];
  }

  // MATLAB Function: '<S102>/Reduced Attitude Weighting Factors' incorporates:
  //   MATLAB Function: '<S108>/MATLAB Function'

  scale = -rtb_n_b[2];
  if (-rtb_n_b[2] < 0.0F) {
    scale = 0.0F;
  }

  // Gain: '<S126>/1//T' incorporates:
  //   DiscreteIntegrator: '<S126>/Discrete-Time Integrator'
  //   Gain: '<S108>/r_max'
  //   Sum: '<S126>/Sum2'
  //   Sum: '<S4>/Add'

  rtb_y_lm = ((rtDW.Gain1 + rtDW.Merge[2]) * 6.28318548F -
              rtDW.DiscreteTimeIntegrator_DSTATE) * 2.34082174F;

  // MATLAB Function: '<S108>/Simulink Trickster' incorporates:
  //   DiscreteIntegrator: '<S126>/Discrete-Time Integrator'

  t = rtDW.DiscreteTimeIntegrator_DSTATE;

  // MATLAB Function: '<S108>/Desired Roll Pitch' incorporates:
  //   DiscreteIntegrator: '<S95>/Discrete-Time Integrator y'

  rtb_n_b[0] = rtb_s_g_dt2_ref[1] * rtb_n_dt[2] - rtb_s_g_dt2_ref[2] * rtb_n_dt
    [1];
  rtb_n_b[1] = rtb_s_g_dt2_ref[2] * rtb_n_dt[0] - rtb_s_g_dt2_ref[0] * rtb_n_dt
    [2];
  rtb_n_b[2] = rtb_s_g_dt2_ref[0] * rtb_n_dt[1] - rtb_s_g_dt2_ref[1] * rtb_n_dt
    [0];
  for (i = 0; i < 3; i++) {
    rtb_n_dt2_i[i] = rtDW.DiscreteTimeIntegratory_DSTAT_p[i + 6] * rtb_n_b[2] +
      (rtDW.DiscreteTimeIntegratory_DSTAT_p[i + 3] * rtb_n_b[1] +
       rtDW.DiscreteTimeIntegratory_DSTAT_p[i] * rtb_n_b[0]);
  }

  // MATLAB Function: '<S99>/DCM to quaternions'
  DCMtoquaternions(rtb_Sum2_p, rtb_Add_a);

  // MATLAB Function: '<S99>/Quaternion Reduced'
  QuaternionReduced(rtb_Add_a, rtb_q_red_b, &q1_q1);

  // DiscreteIntegrator: '<S108>/Discrete-Time Integrator2'
  if (rtDW.DiscreteTimeIntegrator2_IC_LOAD != 0) {
    rtDW.DiscreteTimeIntegrator2_DSTATE = q1_q1;
  }

  // MATLAB Function: '<S103>/wrap angle' incorporates:
  //   DiscreteIntegrator: '<S108>/Discrete-Time Integrator2'

  wrapangle(rtDW.DiscreteTimeIntegrator2_DSTATE, &rtb_Gain6);

  // MATLAB Function: '<S104>/DCM to quaternions'
  DCMtoquaternions(rtb_Sum2_p, rtb_Add_a);

  // MATLAB Function: '<S104>/Quaternion Reduced'
  QuaternionReduced(rtb_Add_a, rtb_q_red_b, &q1_q1);

  // MATLAB Function: '<S103>/wrap angle1'
  wrapangle(q1_q1, &q0_q1);

  // MATLAB Function: '<S103>/angle error'
  q1_q1 = rtb_Gain6 - q0_q1;
  if (q1_q1 > 3.1415926535897931) {
    q1_q1 -= 6.28318548F;
  } else {
    if (q1_q1 < -3.1415926535897931) {
      q1_q1 += 6.28318548F;
    }
  }

  // End of MATLAB Function: '<S103>/angle error'

  // MATLAB Function: '<S108>/Pseudo-Control Roll Pitch' incorporates:
  //   DiscreteIntegrator: '<S98>/Discrete-Time Integrator y'
  //   MATLAB Function: '<S121>/n ref norm'

  rtb_n_b_c[0] = rtb_n[1] * (real32_T)tmp[2] - rtb_n[2] * (real32_T)tmp[1];
  rtb_n_b_c[1] = rtb_n[2] * (real32_T)tmp[0] - rtb_n[0] * (real32_T)tmp[2];
  rtb_n_b_c[2] = rtb_n[0] * (real32_T)tmp[1] - rtb_n[1] * (real32_T)tmp[0];
  t_0[0] = 0.0F;
  t_0[3] = -rtDW.DiscreteTimeIntegratory_DSTAT_k[2];
  t_0[6] = rtDW.DiscreteTimeIntegratory_DSTAT_k[1];
  t_0[1] = rtDW.DiscreteTimeIntegratory_DSTAT_k[2];
  t_0[4] = 0.0F;
  t_0[7] = -rtDW.DiscreteTimeIntegratory_DSTAT_k[0];
  t_0[2] = -rtDW.DiscreteTimeIntegratory_DSTAT_k[1];
  t_0[5] = rtDW.DiscreteTimeIntegratory_DSTAT_k[0];
  t_0[8] = 0.0F;
  rtb_n_b_dt_p[0] = rtb_n[1] * rtb_n_dt_o[2] - rtb_n[2] * rtb_n_dt_o[1];
  rtb_n_b_dt_p[1] = rtb_n[2] * rtb_n_dt_o[0] - rtb_n[0] * rtb_n_dt_o[2];
  rtb_n_b_dt_p[2] = rtb_n[0] * rtb_n_dt_o[1] - rtb_n[1] * rtb_n_dt_o[0];

  // MATLAB Function: '<S108>/Desired Roll Pitch' incorporates:
  //   DiscreteIntegrator: '<S98>/Discrete-Time Integrator y'

  q0_q1 = rtb_s_g_dt2_ref[1] * rtb_n_dt2[2] - rtb_s_g_dt2_ref[2] * rtb_n_dt2[1];
  rtb_Gain6 = rtb_s_g_dt2_ref[2] * rtb_n_dt2[0] - rtb_s_g_dt2_ref[0] *
    rtb_n_dt2[2];
  q2_q3 = rtb_s_g_dt2_ref[0] * rtb_n_dt2[1] - rtb_s_g_dt2_ref[1] * rtb_n_dt2[0];
  tmp_1[0] = 0.0F;
  tmp_1[3] = -rtDW.DiscreteTimeIntegratory_DSTAT_k[2];
  tmp_1[6] = rtDW.DiscreteTimeIntegratory_DSTAT_k[1];
  tmp_1[1] = rtDW.DiscreteTimeIntegratory_DSTAT_k[2];
  tmp_1[4] = 0.0F;
  tmp_1[7] = -rtDW.DiscreteTimeIntegratory_DSTAT_k[0];
  tmp_1[2] = -rtDW.DiscreteTimeIntegratory_DSTAT_k[1];
  tmp_1[5] = rtDW.DiscreteTimeIntegratory_DSTAT_k[0];
  tmp_1[8] = 0.0F;
  for (i = 0; i < 3; i++) {
    // MATLAB Function: '<S108>/Pseudo-Control Roll Pitch' incorporates:
    //   DiscreteIntegrator: '<S95>/Discrete-Time Integrator y'
    //   MATLAB Function: '<S108>/Desired Roll Pitch'

    rtb_n_dt_o[i] = 0.0F;
    rtb_s_g_dt2_ref[i] = 0.0F;
    for (db = 0; db < 3; db++) {
      // MATLAB Function: '<S108>/Desired Roll Pitch'
      loop_ub = i + 3 * db;
      tmp_0[loop_ub] = 0.0F;

      // MATLAB Function: '<S108>/Desired Roll Pitch' incorporates:
      //   DiscreteIntegrator: '<S95>/Discrete-Time Integrator y'

      b_v_size = 3 * db + i;
      absxk = rtDW.DiscreteTimeIntegratory_DSTAT_p[3 * db];
      tmp_0[loop_ub] = tmp_0[b_v_size] + absxk * t_0[3 * i];

      // MATLAB Function: '<S108>/Desired Roll Pitch' incorporates:
      //   DiscreteIntegrator: '<S95>/Discrete-Time Integrator y'

      rtb_G1_tmp = 3 * i + 1;
      q0_q3 = rtDW.DiscreteTimeIntegratory_DSTAT_p[3 * db + 1];
      tmp_0[loop_ub] = q0_q3 * t_0[rtb_G1_tmp] + tmp_0[b_v_size];

      // MATLAB Function: '<S108>/Desired Roll Pitch' incorporates:
      //   DiscreteIntegrator: '<S95>/Discrete-Time Integrator y'

      b_k = 3 * i + 2;
      q1_q2 = rtDW.DiscreteTimeIntegratory_DSTAT_p[3 * db + 2];
      tmp_0[loop_ub] = q1_q2 * t_0[b_k] + tmp_0[b_v_size];

      // MATLAB Function: '<S108>/Desired Roll Pitch'
      tmp_2[loop_ub] = 0.0F;
      tmp_2[loop_ub] = tmp_2[b_v_size] + absxk * tmp_1[3 * i];
      tmp_2[loop_ub] = q0_q3 * tmp_1[rtb_G1_tmp] + tmp_2[b_v_size];
      tmp_2[loop_ub] = q1_q2 * tmp_1[b_k] + tmp_2[b_v_size];
      rtb_n_dt_o[i] += rtDW.DiscreteTimeIntegratory_DSTAT_p[b_v_size] *
        rtb_n_b_c[db];
      rtb_s_g_dt2_ref[i] += tmp_0[b_v_size] * rtb_n_b_dt_p[db];
    }

    rtb_n_dt2[i] = rtb_n_dt_o[i] + rtb_s_g_dt2_ref[i];

    // MATLAB Function: '<S108>/Desired Roll Pitch' incorporates:
    //   DiscreteIntegrator: '<S95>/Discrete-Time Integrator y'

    tmp_3[i] = (rtDW.DiscreteTimeIntegratory_DSTAT_p[i + 6] * q2_q3 +
                (rtDW.DiscreteTimeIntegratory_DSTAT_p[i + 3] * rtb_Gain6 +
                 rtDW.DiscreteTimeIntegratory_DSTAT_p[i] * q0_q1)) + (tmp_2[i +
      6] * rtb_n_b[2] + (tmp_2[i + 3] * rtb_n_b[1] + tmp_2[i] * rtb_n_b[0]));
  }

  // Sum: '<S35>/Add2' incorporates:
  //   DiscreteIntegrator: '<S126>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S98>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S98>/Discrete-Time Integrator y_dt'
  //   Gain: '<S105>/Gain'
  //   Gain: '<S105>/Gain1'
  //   Gain: '<S105>/Gain2'
  //   Gain: '<S105>/Gain3'
  //   Gain: '<S105>/Gain4'
  //   Gain: '<S105>/Gain5'
  //   Gain: '<S105>/Gain6'
  //   Gain: '<S105>/Gain7'
  //   Gain: '<S105>/Gain8'
  //   Gain: '<S106>/Gain'
  //   MATLAB Function: '<S102>/Reduced Attitude Weighting Factors'
  //   MATLAB Function: '<S108>/Desired Roll Pitch'
  //   MATLAB Function: '<S108>/MATLAB Function'
  //   MATLAB Function: '<S108>/Pseudo-Control Roll Pitch'
  //   MATLAB Function: '<S108>/Simulink Trickster'
  //   Product: '<S105>/Product'
  //   Product: '<S105>/Product1'
  //   Product: '<S105>/Product3'
  //   Product: '<S105>/Product4'
  //   Product: '<S106>/Product'
  //   Product: '<S106>/Product1'
  //   Product: '<S107>/Product'
  //   Sum: '<S103>/error1 1'
  //   Sum: '<S103>/error1 2'
  //   Sum: '<S103>/error1 4'
  //   Sum: '<S103>/error1 5'
  //   Sum: '<S103>/error1 6'
  //   Sum: '<S103>/error1 8'
  //   Sum: '<S103>/error1 9'
  //   Sum: '<S105>/Add'
  //   Sum: '<S105>/Add1'
  //   Sum: '<S105>/Add2'
  //   Sum: '<S105>/Add3'
  //   Sum: '<S106>/Add'
  //   Sum: '<S107>/Add'
  //   Sum: '<S35>/Add1'

  rtb_n_dt[0] = (((((1.0F - scale) * (rtb_n_dt2_i[0] -
    rtDW.DiscreteTimeIntegratory_DSTAT_k[0]) + scale * rtb_n_b_dt[1]) *
                   26.7747803F + 94.5895F * rtb_y_dt_cs[1]) + ((1.0F - scale) *
    (tmp_3[0] - rtDW.DiscreteTimeIntegratory_dt_DSTA[0]) + scale * rtb_Sum2_e[1])
                  * 1.52631581F) + ((1.0F - scale) * rtb_n_dt2[0] + scale *
    rtb_n_b_dt2[1])) - (1.0F - scale) * rtDW.DiscreteTimeIntegratory_dt_DSTA[0];
  rtb_n_dt[1] = (((((1.0F - scale) * (rtb_n_dt2_i[1] -
    rtDW.DiscreteTimeIntegratory_DSTAT_k[1]) + scale * -rtb_n_b_dt[0]) *
                   26.7747803F + 94.5895F * -rtb_y_dt_cs[0]) + ((1.0F - scale) *
    (tmp_3[1] - rtDW.DiscreteTimeIntegratory_dt_DSTA[1]) + scale * -rtb_Sum2_e[0])
                  * 1.52631581F) + ((1.0F - scale) * rtb_n_dt2[1] + scale *
    -rtb_n_b_dt2[0])) - (1.0F - scale) * rtDW.DiscreteTimeIntegratory_dt_DSTA[1];
  rtb_n_dt[2] = ((((rtDW.DiscreteTimeIntegrator_DSTATE -
                    rtDW.DiscreteTimeIntegratory_DSTAT_k[2]) * 1.30612898F +
                   1.01913834F * q1_q1) + (rtb_y_lm -
    rtDW.DiscreteTimeIntegratory_dt_DSTA[2]) * -0.442021161F) + rtb_y_lm) -
    rtDW.DiscreteTimeIntegratory_dt_DSTA[2];

  // Outputs for Enabled SubSystem: '<S4>/Vertical Acc to Specific Thrust' incorporates:
  //   EnablePort: '<S41>/Enable'

  if (rtb_state_isVertPscEnabled) {
    // MATLAB Function: '<S41>/MATLAB Function' incorporates:
    //   Constant: '<S41>/Constant'
    //   Constant: '<S4>/Constant2'
    //   DiscreteIntegrator: '<S95>/Discrete-Time Integrator y'

    if (rtDW.DiscreteTimeIntegratory_DSTAT_p[8] < 0.437750697F) {
      // SignalConversion: '<S41>/OutportBufferForT_spec_des'
      rtDW.Merge1_e = 0.0F;
    } else {
      q1_q1 = rtDW.DiscreteTimeIntegratory_DSTAT_p[8];
      if (std::abs(rtDW.DiscreteTimeIntegratory_DSTAT_p[8]) < 2.22044605E-16F) {
        q1_q1 = 2.22044605E-16F;
      }

      // SignalConversion: '<S41>/OutportBufferForT_spec_des'
      rtDW.Merge1_e = (-rtb_Add1[2] + 9.81F) / q1_q1;
    }

    // End of MATLAB Function: '<S41>/MATLAB Function'
  }

  // End of Outputs for SubSystem: '<S4>/Vertical Acc to Specific Thrust'

  // Outputs for Enabled SubSystem: '<S4>/Incremental specific thrust' incorporates:
  //   EnablePort: '<S33>/Enable'

  if (rtb_is_slowing) {
    // DiscreteIntegrator: '<S89>/Discrete-Time Integrator y'
    if (rtDW.DiscreteTimeIntegratory_IC_LO_e != 0) {
      rtDW.DiscreteTimeIntegratory_DSTA_lk[0] = rtDW.n_g_des[0];
      rtDW.DiscreteTimeIntegratory_DSTA_lk[1] = rtDW.n_g_des[1];
      rtDW.DiscreteTimeIntegratory_DSTA_lk[2] = rtDW.n_g_des[2];
    }

    rtb_Sum2_e[0] = rtDW.DiscreteTimeIntegratory_DSTA_lk[0];

    // DiscreteIntegrator: '<S89>/Discrete-Time Integrator y_dt'
    rtb_n[0] = rtDW.DiscreteTimeIntegratory_dt_DS_d[0];

    // Gain: '<S89>/omega^2' incorporates:
    //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y_dt'
    //   Gain: '<S89>/2*d//omega'
    //   Sum: '<S89>/Sum2'
    //   Sum: '<S89>/Sum3'

    rtb_Add1[0] = (rtDW.n_g_des[0] - (0.188708603F *
      rtDW.DiscreteTimeIntegratory_dt_DS_d[0] +
      rtDW.DiscreteTimeIntegratory_DSTA_lk[0])) * 112.325035F;

    // DiscreteIntegrator: '<S89>/Discrete-Time Integrator y'
    rtb_Sum2_e[1] = rtDW.DiscreteTimeIntegratory_DSTA_lk[1];

    // DiscreteIntegrator: '<S89>/Discrete-Time Integrator y_dt'
    rtb_n[1] = rtDW.DiscreteTimeIntegratory_dt_DS_d[1];

    // Gain: '<S89>/omega^2' incorporates:
    //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y_dt'
    //   Gain: '<S89>/2*d//omega'
    //   Sum: '<S89>/Sum2'
    //   Sum: '<S89>/Sum3'

    rtb_Add1[1] = (rtDW.n_g_des[1] - (0.188708603F *
      rtDW.DiscreteTimeIntegratory_dt_DS_d[1] +
      rtDW.DiscreteTimeIntegratory_DSTA_lk[1])) * 112.325035F;

    // DiscreteIntegrator: '<S89>/Discrete-Time Integrator y'
    rtb_Sum2_e[2] = rtDW.DiscreteTimeIntegratory_DSTA_lk[2];

    // DiscreteIntegrator: '<S89>/Discrete-Time Integrator y_dt'
    rtb_n[2] = rtDW.DiscreteTimeIntegratory_dt_DS_d[2];

    // Gain: '<S89>/omega^2' incorporates:
    //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y_dt'
    //   Gain: '<S89>/2*d//omega'
    //   Sum: '<S89>/Sum2'
    //   Sum: '<S89>/Sum3'

    rtb_Add1[2] = (rtDW.n_g_des[2] - (0.188708603F *
      rtDW.DiscreteTimeIntegratory_dt_DS_d[2] +
      rtDW.DiscreteTimeIntegratory_DSTA_lk[2])) * 112.325035F;

    // MATLAB Function: '<S89>/n ref norm'
    nrefnorm(rtb_Add1, rtb_n, rtb_Sum2_e, rtb_n_dt_o, rtb_n_dt2_i,
             rtb_s_g_dt2_ref);

    // MATLAB Function: '<S33>/DCM 2 Lean Vector'
    DCM2LeanVector(rtb_Sum2_p, rtb_Sum2_e);

    // MATLAB Function: '<S33>/desired and measured specific thrust' incorporates:
    //   DiscreteIntegrator: '<S94>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S95>/Discrete-Time Integrator y'

    for (i = 0; i < 3; i++) {
      absxk = rtDW.DiscreteTimeIntegratory_DSTAT_p[i + 6];
      rtb_n_dt2_i[i] = absxk * rtDW.DiscreteTimeIntegratory_DSTA_al[2] +
        (rtDW.DiscreteTimeIntegratory_DSTAT_p[i + 3] *
         rtDW.DiscreteTimeIntegratory_DSTA_al[1] +
         rtDW.DiscreteTimeIntegratory_DSTAT_p[i] *
         rtDW.DiscreteTimeIntegratory_DSTA_al[0]);
      rtb_n_dt_o[i] = absxk * 9.81F;
    }

    rtDW.a_T_meas = rtb_n_dt2_i[2] - rtb_n_dt_o[2];

    // End of MATLAB Function: '<S33>/desired and measured specific thrust'

    // Sum: '<S33>/Add' incorporates:
    //   Abs: '<S33>/Abs'
    //   DotProduct: '<S33>/Dot Product'
    //   Gain: '<S33>/Gain'
    //   Product: '<S33>/Product'

    rtDW.Delta_nu_a_T = std::abs((rtDW.n_g_des[0] * rtb_s_g_dt2_ref[0] +
      rtDW.n_g_des[1] * rtb_s_g_dt2_ref[1]) + rtDW.n_g_des[2] * rtb_s_g_dt2_ref
      [2]) * -rtDW.Merge1_e - rtDW.a_T_meas;

    // Update for DiscreteIntegrator: '<S89>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTA_lk[0] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_d[0];

    // Update for DiscreteIntegrator: '<S89>/Discrete-Time Integrator y_dt'
    rtDW.DiscreteTimeIntegratory_dt_DS_d[0] += 0.0025F * rtb_Add1[0];

    // Update for DiscreteIntegrator: '<S89>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTA_lk[1] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_d[1];

    // Update for DiscreteIntegrator: '<S89>/Discrete-Time Integrator y_dt'
    rtDW.DiscreteTimeIntegratory_dt_DS_d[1] += 0.0025F * rtb_Add1[1];

    // MATLAB Function: '<S33>/incremental thrust atti correction'
    q1_q1 = (rtb_Sum2_e[0] * rtb_s_g_dt2_ref[0] + rtb_Sum2_e[1] *
             rtb_s_g_dt2_ref[1]) + rtb_Sum2_e[2] * rtb_s_g_dt2_ref[2];

    // Update for DiscreteIntegrator: '<S89>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTA_lk[2] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_d[2];

    // Update for DiscreteIntegrator: '<S89>/Discrete-Time Integrator y_dt'
    rtDW.DiscreteTimeIntegratory_dt_DS_d[2] += 0.0025F * rtb_Add1[2];

    // MATLAB Function: '<S33>/incremental thrust atti correction'
    if (q1_q1 < 0.0F) {
      rtDW.Delta_nu_a_T = 0.0F;
    } else {
      rtDW.Delta_nu_a_T *= q1_q1;
    }

    // Update for DiscreteIntegrator: '<S89>/Discrete-Time Integrator y'
    rtDW.DiscreteTimeIntegratory_IC_LO_e = 0U;
  }

  // End of Outputs for SubSystem: '<S4>/Incremental specific thrust'

  // MinMax: '<S68>/Max' incorporates:
  //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator'

  if (rtDW.DiscreteTimeIntegrator_DSTATE_e[0] > 0.1F) {
    rtb_Add_a[0] = rtDW.DiscreteTimeIntegrator_DSTATE_e[0];
  } else {
    rtb_Add_a[0] = 0.1F;
  }

  if (rtDW.DiscreteTimeIntegrator_DSTATE_e[1] > 0.1F) {
    rtb_Add_a[1] = rtDW.DiscreteTimeIntegrator_DSTATE_e[1];
  } else {
    rtb_Add_a[1] = 0.1F;
  }

  if (rtDW.DiscreteTimeIntegrator_DSTATE_e[2] > 0.1F) {
    rtb_Add_a[2] = rtDW.DiscreteTimeIntegrator_DSTATE_e[2];
  } else {
    rtb_Add_a[2] = 0.1F;
  }

  if (rtDW.DiscreteTimeIntegrator_DSTATE_e[3] > 0.1F) {
    rtb_Add_a[3] = rtDW.DiscreteTimeIntegrator_DSTATE_e[3];
  } else {
    rtb_Add_a[3] = 0.1F;
  }

  // End of MinMax: '<S68>/Max'

  // MATLAB Function: '<S68>/MATLAB Function'
  q1_q1 = std::sqrt(1.45291104E-8F * rtb_Add_a[0] + 1.16073799E-8F);
  rtb_q_red_b[1] = (std::sqrt(1.45291104E-8F * rtb_Add_a[1] + 1.16073799E-8F) -
                    0.000107737556F) / 4.72893973E-8F;
  rtb_q_red_b[2] = (std::sqrt(1.45291104E-8F * rtb_Add_a[2] + 1.16073799E-8F) -
                    0.000107737556F) / 4.72893973E-8F;
  rtb_Add_a[1] = std::sqrt(1.45291104E-8F * rtb_Add_a[1] + 1.16073799E-8F);
  rtb_Add_a[2] = std::sqrt(1.45291104E-8F * rtb_Add_a[2] + 1.16073799E-8F);
  b_k = 0;
  rtb_Add_a[1] = 0.153619125F / rtb_Add_a[1];
  rtb_Add_a[2] = 0.153619125F / rtb_Add_a[2];
  memset(&G_omega[0], 0, sizeof(real32_T) << 4U);
  memset(&rtb_G2[0], 0, sizeof(real32_T) << 4U);
  G_omega[0] = 0.153619125F / q1_q1;
  G_omega[5] = rtb_Add_a[1];
  G_omega[10] = rtb_Add_a[2];
  scale = std::sqrt(1.45291104E-8F * rtb_Add_a[3] + 1.16073799E-8F);
  G_omega[15] = 0.153619125F / scale;
  rtb_G2[0] = (q1_q1 - 0.000107737556F) / 4.72893973E-8F;
  rtb_G2[5] = rtb_q_red_b[1];
  rtb_G2[10] = rtb_q_red_b[2];
  rtb_G2[15] = (scale - 0.000107737556F) / 4.72893973E-8F;
  for (i = 0; i < 4; i++) {
    for (db = 0; db < 4; db++) {
      loop_ub = db << 2;
      b_v_size = i + loop_ub;
      tmp_4[b_v_size] = 0.0F;
      rtb_G1_tmp = loop_ub + i;
      tmp_4[b_v_size] = tmp_4[rtb_G1_tmp] + rtb_G2[loop_ub] *
        rtConstP.MATLABFunction_G10[i];
      tmp_4[b_v_size] = rtb_G2[loop_ub + 1] * rtConstP.MATLABFunction_G10[i + 4]
        + tmp_4[rtb_G1_tmp];
      tmp_4[b_v_size] = rtb_G2[loop_ub + 2] * rtConstP.MATLABFunction_G10[i + 8]
        + tmp_4[rtb_G1_tmp];
      tmp_4[b_v_size] = rtb_G2[loop_ub + 3] * rtConstP.MATLABFunction_G10[i + 12]
        + tmp_4[rtb_G1_tmp];
    }

    for (db = 0; db < 4; db++) {
      loop_ub = db << 2;
      b_v_size = i + loop_ub;
      rtb_G1[b_v_size] = 0.0F;
      rtb_G1_tmp = loop_ub + i;
      rtb_G1[b_v_size] = rtb_G1[rtb_G1_tmp] + G_omega[loop_ub] * tmp_4[i];
      rtb_G1[b_v_size] = G_omega[loop_ub + 1] * tmp_4[i + 4] + rtb_G1[rtb_G1_tmp];
      rtb_G1[b_v_size] = G_omega[loop_ub + 2] * tmp_4[i + 8] + rtb_G1[rtb_G1_tmp];
      rtb_G1[b_v_size] = G_omega[loop_ub + 3] * tmp_4[i + 12] +
        rtb_G1[rtb_G1_tmp];
    }
  }

  for (i = 0; i < 4; i++) {
    for (db = 0; db < 4; db++) {
      loop_ub = i << 2;
      b_v_size = db + loop_ub;
      rtb_G2[b_v_size] = 0.0F;
      rtb_G1_tmp = loop_ub + db;
      rtb_G2[b_v_size] = rtb_G2[rtb_G1_tmp] + G_omega[loop_ub] *
        rtConstP.MATLABFunction_G20[db];
      rtb_G2[b_v_size] = G_omega[loop_ub + 1] * rtConstP.MATLABFunction_G20[db +
        4] + rtb_G2[rtb_G1_tmp];
      rtb_G2[b_v_size] = G_omega[loop_ub + 2] * rtConstP.MATLABFunction_G20[db +
        8] + rtb_G2[rtb_G1_tmp];
      rtb_G2[b_v_size] = G_omega[loop_ub + 3] * rtConstP.MATLABFunction_G20[db +
        12] + rtb_G2[rtb_G1_tmp];
    }
  }

  // End of MATLAB Function: '<S68>/MATLAB Function'

  // MATLAB Function: '<S44>/create diag' incorporates:
  //   Delay: '<S48>/Delay'

  memset(&rtb_y_bn[0], 0, sizeof(real32_T) << 4U);
  rtb_y_bn[0] = rtDW.Delay_DSTATE[0];
  rtb_y_bn[5] = rtDW.Delay_DSTATE[1];
  rtb_y_bn[10] = rtDW.Delay_DSTATE[2];
  rtb_y_bn[15] = rtDW.Delay_DSTATE[3];

  // Product: '<S44>/correct G1'
  for (i = 0; i < 4; i++) {
    for (db = 0; db < 4; db++) {
      loop_ub = i << 2;
      b_v_size = db + loop_ub;
      G_omega[b_v_size] = 0.0F;
      rtb_G1_tmp = loop_ub + db;
      G_omega[b_v_size] = G_omega[rtb_G1_tmp] + rtb_G1[loop_ub] * rtb_y_bn[db];
      G_omega[b_v_size] = rtb_G1[loop_ub + 1] * rtb_y_bn[db + 4] +
        G_omega[rtb_G1_tmp];
      G_omega[b_v_size] = rtb_G1[loop_ub + 2] * rtb_y_bn[db + 8] +
        G_omega[rtb_G1_tmp];
      G_omega[b_v_size] = rtb_G1[loop_ub + 3] * rtb_y_bn[db + 12] +
        G_omega[rtb_G1_tmp];
    }
  }

  // End of Product: '<S44>/correct G1'

  // Product: '<S44>/correct G2' incorporates:
  //   Delay: '<S49>/Delay1'

  for (i = 0; i < 16; i++) {
    rtb_y_bn[i] = rtDW.Delay1_DSTATE * rtb_G2[i];
  }

  // End of Product: '<S44>/correct G2'

  // MATLAB Function: '<S40>/throttle_-1_1 to throttle_0_1' incorporates:
  //   Gain: '<Root>/Gain5'

  q1_q1 = 0.5F * -rtDW.Merge[3] + 0.5F;
  if (1.0F <= q1_q1) {
    q1_q1 = 1.0F;
  }

  if (0.0F < q1_q1) {
    rtb_Gain6 = q1_q1;
  } else {
    rtb_Gain6 = 0.0F;
  }

  q1_q1 = 0.587109566F * rtb_Gain6 + 0.311070889F;

  // MATLAB Function: '<S64>/Set Desired Motor Command' incorporates:
  //   MATLAB Function: '<S40>/throttle_-1_1 to throttle_0_1'

  b_section_idx = 0.0;
  rtb_Output_idx_1 = 0.0;
  rtb_Output_idx_2 = 0.0;
  rtb_Output_idx_3 = 0.0;
  rtb_is_intercept_arc = true;
  exitg1 = false;
  while ((!exitg1) && (b_k < 4)) {
    if (!rtb_UnitDelay4) {
      rtb_is_intercept_arc = false;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  if (rtb_is_intercept_arc) {
    if (1.0F < q1_q1) {
      absxk = 1.0F;
    } else {
      absxk = q1_q1;
    }

    b_section_idx = absxk - 0.1F;
    if (1.0F < q1_q1) {
      absxk = 1.0F;
    } else {
      absxk = q1_q1;
    }

    rtb_Output_idx_1 = absxk - 0.1F;
    if (1.0F < q1_q1) {
      absxk = 1.0F;
      q1_q1 = 1.0F;
    } else {
      absxk = q1_q1;
    }

    rtb_Output_idx_2 = absxk - 0.1F;
    rtb_Output_idx_3 = q1_q1 - 0.1F;
  }

  // MATLAB Function: '<S34>/MATLAB Function'
  DCM2LeanVector(rtb_Sum2_p, rtb_s_g_dt2_ref);

  // MATLAB Function: '<S31>/Control Allocation Vertical Acc Weighting'
  rtb_Add_a[0] = 0.0F;
  rtb_Add_a[1] = 0.0F;
  q1_q1 = (rtb_s_g_dt2_ref[0] * rtDW.n_g_des[0] + rtb_s_g_dt2_ref[1] *
           rtDW.n_g_des[1]) + rtb_s_g_dt2_ref[2] * rtDW.n_g_des[2];
  if (q1_q1 < 0.0F) {
    q1_q1 = 0.0F;
  } else {
    q1_q1 = (rtConstP.ControlAllocationVerticalAccWei.W_v[15] - 10.0F) * (q1_q1 *
      q1_q1) + 10.0F;
  }

  rtb_Add_a[3] = q1_q1 - rtConstP.ControlAllocationVerticalAccWei.W_v[15];

  // End of MATLAB Function: '<S31>/Control Allocation Vertical Acc Weighting'

  // MATLAB Function: '<S64>/Set Vertical Acc Weight To Zero'
  rtb_is_intercept_arc = true;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 4)) {
    if (!rtb_UnitDelay4) {
      rtb_is_intercept_arc = false;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  if (rtb_is_intercept_arc) {
    rtb_Add_a[3] = -rtConstP.SetVerticalAccWeightToZero_ca.W_v[15];
    rtb_Add_a[0] = -rtConstP.SetVerticalAccWeightToZero_ca.W_v[0] + 1.0F;
    rtb_Add_a[1] = -rtConstP.SetVerticalAccWeightToZero_ca.W_v[5] + 1.0F;
  }

  // End of MATLAB Function: '<S64>/Set Vertical Acc Weight To Zero'

  // MATLAB Function: '<S69>/caIndiWls' incorporates:
  //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator'

  Delta_diag_W_v_idx_0 = rtb_Add_a[0];
  if (0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[0] > -0.9F) {
    rtb_Sum2_a[0] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[0];
  } else {
    rtb_Sum2_a[0] = -0.9F;
  }

  rtb_Add_a[0] = (0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[0]) + (1.0F -
    rtDW.DiscreteTimeIntegrator_DSTATE_e[0]);
  Delta_diag_W_v_idx_1 = rtb_Add_a[1];
  if (0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[1] > -0.9F) {
    rtb_Sum2_a[1] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[1];
  } else {
    rtb_Sum2_a[1] = -0.9F;
  }

  rtb_Add_a[1] = (0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[1]) + (1.0F -
    rtDW.DiscreteTimeIntegrator_DSTATE_e[1]);
  if (0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[2] > -0.9F) {
    rtb_Sum2_a[2] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[2];
  } else {
    rtb_Sum2_a[2] = -0.9F;
  }

  rtb_Add_a[2] = (0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[2]) + (1.0F -
    rtDW.DiscreteTimeIntegrator_DSTATE_e[2]);
  Delta_diag_W_v_idx_3 = rtb_Add_a[3];
  if (0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[3] > -0.9F) {
    rtb_Sum2_a[3] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[3];
  } else {
    rtb_Sum2_a[3] = -0.9F;
  }

  rtb_Add_a[3] = (0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[3]) + (1.0F -
    rtDW.DiscreteTimeIntegrator_DSTATE_e[3]);
  if (1.0F - rtDW.DiscreteTimeIntegrator_DSTATE_e[0] < 0.9F) {
    rtb_q_red_b[0] = 1.0F - rtDW.DiscreteTimeIntegrator_DSTATE_e[0];
  } else {
    rtb_q_red_b[0] = 0.9F;
  }

  if (1.0F - rtDW.DiscreteTimeIntegrator_DSTATE_e[1] < 0.9F) {
    rtb_q_red_b[1] = 1.0F - rtDW.DiscreteTimeIntegrator_DSTATE_e[1];
  } else {
    rtb_q_red_b[1] = 0.9F;
  }

  if (1.0F - rtDW.DiscreteTimeIntegrator_DSTATE_e[2] < 0.9F) {
    rtb_q_red_b[2] = 1.0F - rtDW.DiscreteTimeIntegrator_DSTATE_e[2];
  } else {
    rtb_q_red_b[2] = 0.9F;
  }

  if (1.0F - rtDW.DiscreteTimeIntegrator_DSTATE_e[3] < 0.9F) {
    rtb_q_red_b[3] = 1.0F - rtDW.DiscreteTimeIntegrator_DSTATE_e[3];
  } else {
    rtb_q_red_b[3] = 0.9F;
  }

  memset(&d[0], 0, sizeof(real_T) << 4U);
  d[0] = Delta_diag_W_v_idx_0;
  umax[0] = 0.0F;
  rtb_Add_a[0] *= 0.5F;
  d[5] = Delta_diag_W_v_idx_1;
  umax[1] = 0.0F;
  rtb_Add_a[1] *= 0.5F;
  d[10] = 0.0;
  umax[2] = 0.0F;
  rtb_Add_a[2] *= 0.5F;
  d[15] = Delta_diag_W_v_idx_3;
  umax[3] = 0.0F;
  rtb_Add_a[3] *= 0.5F;

  // Sum: '<S64>/Add1' incorporates:
  //   Switch: '<S31>/Switch'
  //   Switch: '<S31>/Switch1'

  for (i = 0; i < 16; i++) {
    G_omega_0[i] = G_omega[i] + rtb_y_bn[i];
  }

  // End of Sum: '<S64>/Add1'

  // Product: '<S64>/MatrixMultiply2' incorporates:
  //   Switch: '<S31>/Switch1'
  //   UnitDelay: '<S64>/Unit Delay1'

  for (i = 0; i < 4; i++) {
    q1_q1 = rtb_y_bn[i + 12] * rtDW.UnitDelay1_DSTATE[3] + (rtb_y_bn[i + 8] *
      rtDW.UnitDelay1_DSTATE[2] + (rtb_y_bn[i + 4] * rtDW.UnitDelay1_DSTATE[1] +
      rtb_y_bn[i] * rtDW.UnitDelay1_DSTATE[0]));
    rtb_y_l[i] = q1_q1;
  }

  // End of Product: '<S64>/MatrixMultiply2'

  // Sum: '<S64>/Add2'
  rtb_n_dt_0[0] = rtb_n_dt[0] + rtb_y_l[0];
  rtb_n_dt_0[1] = rtb_n_dt[1] + rtb_y_l[1];
  rtb_n_dt_0[2] = rtb_n_dt[2] + rtb_y_l[2];
  rtb_n_dt_0[3] = rtDW.Delta_nu_a_T + rtb_y_l[3];

  // MATLAB Function: '<S69>/caIndiWls' incorporates:
  //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator'
  //   MATLAB Function: '<S64>/Set Desired Motor Command'

  for (i = 0; i < 16; i++) {
    tmp_4[i] = rtConstP.caIndiWls_ca.W_v[i] + (real32_T)d[i];
  }

  rtb_y_l[0] = (0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[0]) + (real32_T)
    b_section_idx;
  rtb_y_l[1] = (0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[1]) + (real32_T)
    rtb_Output_idx_1;
  rtb_y_l[2] = (0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[2]) + (real32_T)
    rtb_Output_idx_2;
  rtb_y_l[3] = (0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[3]) + (real32_T)
    rtb_Output_idx_3;
  wls_alloc(G_omega_0, rtb_n_dt_0, rtb_Sum2_a, rtb_q_red_b, tmp_4,
            &rtConstP.caIndiWls_ca.W_u[0], rtb_y_l, 1000.0F, rtb_Add_a, umax,
            100.0F);

  // Sum: '<S31>/Add6' incorporates:
  //   DiscreteIntegrator: '<S67>/Discrete-Time Integrator y'

  q1_q1 = rtb_Add_a[0] + rtDW.DiscreteTimeIntegratory_DSTAT_n[0];

  // Saturate: '<S31>/Saturation3'
  if (q1_q1 > 1.0F) {
    q1_q1 = 1.0F;
  } else {
    if (q1_q1 < 0.1F) {
      q1_q1 = 0.1F;
    }
  }

  // Outport: '<Root>/logs'
  rtY.logs[0] = q1_q1;

  // Saturate: '<S31>/Saturation3'
  rtb_q_red_b[0] = q1_q1;

  // Sum: '<S31>/Add6' incorporates:
  //   DiscreteIntegrator: '<S67>/Discrete-Time Integrator y'

  q1_q1 = rtb_Add_a[1] + rtDW.DiscreteTimeIntegratory_DSTAT_n[1];

  // Saturate: '<S31>/Saturation3'
  if (q1_q1 > 1.0F) {
    q1_q1 = 1.0F;
  } else {
    if (q1_q1 < 0.1F) {
      q1_q1 = 0.1F;
    }
  }

  // Outport: '<Root>/logs'
  rtY.logs[1] = q1_q1;

  // Saturate: '<S31>/Saturation3'
  rtb_q_red_b[1] = q1_q1;

  // Sum: '<S31>/Add6' incorporates:
  //   DiscreteIntegrator: '<S67>/Discrete-Time Integrator y'

  q1_q1 = rtb_Add_a[2] + rtDW.DiscreteTimeIntegratory_DSTAT_n[2];

  // Saturate: '<S31>/Saturation3'
  if (q1_q1 > 1.0F) {
    q1_q1 = 1.0F;
  } else {
    if (q1_q1 < 0.1F) {
      q1_q1 = 0.1F;
    }
  }

  // Outport: '<Root>/logs'
  rtY.logs[2] = q1_q1;

  // Saturate: '<S31>/Saturation3'
  rtb_q_red_b[2] = q1_q1;

  // Sum: '<S31>/Add6' incorporates:
  //   DiscreteIntegrator: '<S67>/Discrete-Time Integrator y'

  q1_q1 = rtb_Add_a[3] + rtDW.DiscreteTimeIntegratory_DSTAT_n[3];

  // Saturate: '<S31>/Saturation3'
  if (q1_q1 > 1.0F) {
    q1_q1 = 1.0F;
  } else {
    if (q1_q1 < 0.1F) {
      q1_q1 = 0.1F;
    }
  }

  // Outport: '<Root>/logs' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion'
  //   DataTypeConversion: '<Root>/Data Type Conversion2'
  //   Delay: '<S48>/Delay'
  //   Delay: '<S49>/Delay1'
  //   DiscreteIntegrator: '<S98>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/cmd'
  //   Inport: '<Root>/measure'
  //   MATLAB Function: '<S8>/Auxiliary function to define log_config in generated C++ code'
  //   SignalConversion: '<Root>/BusConversion_InsertedFor_cmd_at_outport_0'
  //   SignalConversion: '<S144>/TmpSignal ConversionAt SFunction Inport2'

  rtY.logs[3] = q1_q1;
  for (i = 0; i < 9; i++) {
    rtY.logs[i + 4] = rtDW.Add[i];
  }

  rtY.logs[13] = rtb_y_dt_cs[0];
  rtY.logs[14] = rtb_y_dt_cs[1];
  rtY.logs[30] = rtDW.Delta_nu_a_T;
  rtY.logs[15] = rtDW.s_g_ref_f[0];
  rtY.logs[18] = rtDW.s_g[0];
  rtY.logs[21] = rtDW.s_g_dt[0];
  rtY.logs[24] = rtDW.s_g_dt2[0];
  rtY.logs[27] = rtb_n_dt[0];
  rtY.logs[31] = rtU.measure.omega_Kb[0];
  rtY.logs[16] = rtDW.s_g_ref_f[1];
  rtY.logs[19] = rtDW.s_g[1];
  rtY.logs[22] = rtDW.s_g_dt[1];
  rtY.logs[25] = rtDW.s_g_dt2[1];
  rtY.logs[28] = rtb_n_dt[1];
  rtY.logs[32] = rtU.measure.omega_Kb[1];
  rtY.logs[17] = rtDW.s_g_ref_f[2];
  rtY.logs[20] = rtDW.s_g[2];
  rtY.logs[23] = rtDW.s_g_dt[2];
  rtY.logs[26] = rtDW.s_g_dt2[2];
  rtY.logs[29] = rtb_n_dt[2];
  rtY.logs[33] = rtU.measure.omega_Kb[2];
  rtY.logs[34] = rtU.measure.omega_mot[0];
  rtY.logs[38] = rtU.measure.q_bg[0];
  rtY.logs[35] = rtU.measure.omega_mot[1];
  rtY.logs[39] = rtU.measure.q_bg[1];
  rtY.logs[36] = rtU.measure.omega_mot[2];
  rtY.logs[40] = rtU.measure.q_bg[2];
  rtY.logs[37] = rtU.measure.omega_mot[3];
  rtY.logs[41] = rtU.measure.q_bg[3];
  rtY.logs[42] = rtDW.DiscreteTimeIntegratory_dt_DSTA[0];
  rtY.logs[43] = rtDW.DiscreteTimeIntegratory_dt_DSTA[1];
  rtY.logs[44] = rtDW.DiscreteTimeIntegratory_dt_DSTA[2];
  rtY.logs[45] = rtDW.Delay_DSTATE[0];
  rtY.logs[46] = rtDW.Delay_DSTATE[1];
  rtY.logs[47] = rtDW.Delay_DSTATE[2];
  rtY.logs[48] = rtDW.Delay_DSTATE[3];
  rtY.logs[49] = rtDW.Delay1_DSTATE;
  rtY.logs[50] = rtU.cmd.mission_change;
  rtY.logs[51] = rtb_Compare;
  for (i = 0; i < 6; i++) {
    rtY.logs[i + 52] = state_vec[i];
  }

  // Outputs for Enabled SubSystem: '<S44>/Adaptive INDI G1 and G2 correction' incorporates:
  //   EnablePort: '<S45>/Enable'

  // Outputs for Enabled SubSystem: '<S44>/INDI Inversion Check' incorporates:
  //   EnablePort: '<S47>/Enable'

  if (rtb_is_descent) {
    // DiscreteIntegrator: '<S62>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S67>/Discrete-Time Integrator y'

    if (rtDW.DiscreteTimeIntegratory_IC_L_eh != 0) {
      rtDW.DiscreteTimeIntegratory_DSTA_n3[0] =
        rtDW.DiscreteTimeIntegratory_DSTAT_n[0];
      rtDW.DiscreteTimeIntegratory_DSTA_n3[1] =
        rtDW.DiscreteTimeIntegratory_DSTAT_n[1];
      rtDW.DiscreteTimeIntegratory_DSTA_n3[2] =
        rtDW.DiscreteTimeIntegratory_DSTAT_n[2];
      rtDW.DiscreteTimeIntegratory_DSTA_n3[3] =
        rtDW.DiscreteTimeIntegratory_DSTAT_n[3];
    }

    // Sum: '<S62>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S62>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S62>/Discrete-Time Integrator y_dt'
    //   DiscreteIntegrator: '<S67>/Discrete-Time Integrator y'
    //   Product: '<S62>/Product2'
    //   Sum: '<S62>/Sum3'

    rtb_Sum2_a[0] = rtDW.DiscreteTimeIntegratory_DSTAT_n[0] -
      (rtDW.DiscreteTimeIntegratory_dt_DS_f[0] * 0.0794562623F +
       rtDW.DiscreteTimeIntegratory_DSTA_n3[0]);
    rtb_Sum2_a[1] = rtDW.DiscreteTimeIntegratory_DSTAT_n[1] -
      (rtDW.DiscreteTimeIntegratory_dt_DS_f[1] * 0.0794562623F +
       rtDW.DiscreteTimeIntegratory_DSTA_n3[1]);
    rtb_Sum2_a[2] = rtDW.DiscreteTimeIntegratory_DSTAT_n[2] -
      (rtDW.DiscreteTimeIntegratory_dt_DS_f[2] * 0.0794562623F +
       rtDW.DiscreteTimeIntegratory_DSTA_n3[2]);
    rtb_Sum2_a[3] = rtDW.DiscreteTimeIntegratory_DSTAT_n[3] -
      (rtDW.DiscreteTimeIntegratory_dt_DS_f[3] * 0.0794562623F +
       rtDW.DiscreteTimeIntegratory_DSTA_n3[3]);

    // Gain: '<S58>/Gain' incorporates:
    //   DiscreteIntegrator: '<S62>/Discrete-Time Integrator y_dt'
    //   DiscreteIntegrator: '<S67>/Discrete-Time Integrator y_dt'
    //   Sum: '<S60>/Add1'

    scale = (rtDW.DiscreteTimeIntegratory_dt_DS_l[0] -
             rtDW.DiscreteTimeIntegratory_dt_DS_f[0]) * 0.0025F;
    absxk = (rtDW.DiscreteTimeIntegratory_dt_DS_l[1] -
             rtDW.DiscreteTimeIntegratory_dt_DS_f[1]) * 0.0025F;
    q0_q1 = (rtDW.DiscreteTimeIntegratory_dt_DS_l[2] -
             rtDW.DiscreteTimeIntegratory_dt_DS_f[2]) * 0.0025F;
    rtb_Gain6 = (rtDW.DiscreteTimeIntegratory_dt_DS_l[3] -
                 rtDW.DiscreteTimeIntegratory_dt_DS_f[3]) * 0.0025F;
    for (i = 0; i < 4; i++) {
      // Product: '<S58>/Matrix Multiply3'
      rtDW.MatrixMultiply3[i] = 0.0F;
      rtDW.MatrixMultiply3[i] += rtb_y_bn[i] * scale;
      rtDW.MatrixMultiply3[i] += rtb_y_bn[i + 4] * absxk;
      rtDW.MatrixMultiply3[i] += rtb_y_bn[i + 8] * q0_q1;
      rtDW.MatrixMultiply3[i] += rtb_y_bn[i + 12] * rtb_Gain6;

      // Sum: '<S60>/Add3' incorporates:
      //   DiscreteIntegrator: '<S62>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S67>/Discrete-Time Integrator y'

      rtDW.Add3[i] = rtDW.DiscreteTimeIntegratory_DSTAT_n[i] -
        rtDW.DiscreteTimeIntegratory_DSTA_n3[i];
    }

    // Product: '<S58>/Matrix Multiply2'
    for (i = 0; i < 4; i++) {
      rtDW.MatrixMultiply2[i] = 0.0F;
      rtDW.MatrixMultiply2[i] += G_omega[i] * rtDW.Add3[0];
      rtDW.MatrixMultiply2[i] += G_omega[i + 4] * rtDW.Add3[1];
      rtDW.MatrixMultiply2[i] += G_omega[i + 8] * rtDW.Add3[2];
      rtDW.MatrixMultiply2[i] += G_omega[i + 12] * rtDW.Add3[3];
    }

    // End of Product: '<S58>/Matrix Multiply2'

    // DiscreteIntegrator: '<S61>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S98>/Discrete-Time Integrator y_dt'

    if (rtDW.DiscreteTimeIntegratory_IC_L_ir != 0) {
      rtDW.DiscreteTimeIntegratory_DSTAT_c[0] =
        rtDW.DiscreteTimeIntegratory_dt_DSTA[0];
      rtDW.DiscreteTimeIntegratory_DSTAT_c[1] =
        rtDW.DiscreteTimeIntegratory_dt_DSTA[1];
      rtDW.DiscreteTimeIntegratory_DSTAT_c[2] =
        rtDW.DiscreteTimeIntegratory_dt_DSTA[2];
      rtDW.DiscreteTimeIntegratory_DSTAT_c[3] = rtDW.a_T_meas;
    }

    // Sum: '<S61>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt'
    //   DiscreteIntegrator: '<S98>/Discrete-Time Integrator y_dt'
    //   Product: '<S61>/Product2'
    //   Sum: '<S61>/Sum3'

    umax[3] = rtDW.a_T_meas - (rtDW.DiscreteTimeIntegratory_dt_D_f1[3] *
      0.0794562623F + rtDW.DiscreteTimeIntegratory_DSTAT_c[3]);
    umax[0] = rtDW.DiscreteTimeIntegratory_dt_DSTA[0] -
      (rtDW.DiscreteTimeIntegratory_dt_D_f1[0] * 0.0794562623F +
       rtDW.DiscreteTimeIntegratory_DSTAT_c[0]);

    // Sum: '<S59>/Add2' incorporates:
    //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S98>/Discrete-Time Integrator y_dt'

    rtDW.Delta_nu_measure[0] = rtDW.DiscreteTimeIntegratory_dt_DSTA[0] -
      rtDW.DiscreteTimeIntegratory_DSTAT_c[0];

    // Sum: '<S61>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt'
    //   DiscreteIntegrator: '<S98>/Discrete-Time Integrator y_dt'
    //   Product: '<S61>/Product2'
    //   Sum: '<S61>/Sum3'

    umax[1] = rtDW.DiscreteTimeIntegratory_dt_DSTA[1] -
      (rtDW.DiscreteTimeIntegratory_dt_D_f1[1] * 0.0794562623F +
       rtDW.DiscreteTimeIntegratory_DSTAT_c[1]);

    // Sum: '<S59>/Add2' incorporates:
    //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S98>/Discrete-Time Integrator y_dt'

    rtDW.Delta_nu_measure[1] = rtDW.DiscreteTimeIntegratory_dt_DSTA[1] -
      rtDW.DiscreteTimeIntegratory_DSTAT_c[1];

    // Sum: '<S61>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt'
    //   DiscreteIntegrator: '<S98>/Discrete-Time Integrator y_dt'
    //   Product: '<S61>/Product2'
    //   Sum: '<S61>/Sum3'

    umax[2] = rtDW.DiscreteTimeIntegratory_dt_DSTA[2] -
      (rtDW.DiscreteTimeIntegratory_dt_D_f1[2] * 0.0794562623F +
       rtDW.DiscreteTimeIntegratory_DSTAT_c[2]);

    // Sum: '<S59>/Add2' incorporates:
    //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S98>/Discrete-Time Integrator y_dt'

    rtDW.Delta_nu_measure[2] = rtDW.DiscreteTimeIntegratory_dt_DSTA[2] -
      rtDW.DiscreteTimeIntegratory_DSTAT_c[2];
    rtDW.Delta_nu_measure[3] = rtDW.a_T_meas -
      rtDW.DiscreteTimeIntegratory_DSTAT_c[3];

    // Update for DiscreteIntegrator: '<S62>/Discrete-Time Integrator y'
    rtDW.DiscreteTimeIntegratory_IC_L_eh = 0U;

    // Update for DiscreteIntegrator: '<S61>/Discrete-Time Integrator y'
    rtDW.DiscreteTimeIntegratory_IC_L_ir = 0U;

    // Update for DiscreteIntegrator: '<S62>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S62>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTA_n3[0] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_f[0];

    // Update for DiscreteIntegrator: '<S62>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S62>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_DS_f[0] += rtb_Sum2_a[0] * 633.583374F *
      0.0025F;

    // Update for DiscreteIntegrator: '<S61>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_c[0] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_D_f1[0];

    // Update for DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S61>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_D_f1[0] += umax[0] * 633.583374F * 0.0025F;

    // Update for DiscreteIntegrator: '<S62>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S62>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTA_n3[1] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_f[1];

    // Update for DiscreteIntegrator: '<S62>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S62>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_DS_f[1] += rtb_Sum2_a[1] * 633.583374F *
      0.0025F;

    // Update for DiscreteIntegrator: '<S61>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_c[1] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_D_f1[1];

    // Update for DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S61>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_D_f1[1] += umax[1] * 633.583374F * 0.0025F;

    // Update for DiscreteIntegrator: '<S62>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S62>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTA_n3[2] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_f[2];

    // Update for DiscreteIntegrator: '<S62>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S62>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_DS_f[2] += rtb_Sum2_a[2] * 633.583374F *
      0.0025F;

    // Update for DiscreteIntegrator: '<S61>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_c[2] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_D_f1[2];

    // Update for DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S61>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_D_f1[2] += umax[2] * 633.583374F * 0.0025F;

    // Update for DiscreteIntegrator: '<S62>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S62>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTA_n3[3] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_f[3];

    // Update for DiscreteIntegrator: '<S62>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S62>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_DS_f[3] += rtb_Sum2_a[3] * 633.583374F *
      0.0025F;

    // Update for DiscreteIntegrator: '<S61>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_c[3] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_D_f1[3];

    // Update for DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S61>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_D_f1[3] += umax[3] * 633.583374F * 0.0025F;
    if (!rtDW.AdaptiveINDIG1andG2correction_M) {
      rtDW.AdaptiveINDIG1andG2correction_M = true;
    }

    // MATLAB Function: '<S45>/G1 learn rate' incorporates:
    //   Constant: '<S45>/Constant10'

    umax[0] = std::abs(rtb_G1[0]);
    rtb_Sum2_a[0] = std::abs(rtb_G1[1]);
    rtb_n_dt_0[0] = std::abs(rtb_G1[2]);
    rtb_y_l[0] = std::abs(rtb_G1[3]);
    umax[1] = std::abs(rtb_G1[4]);
    rtb_Sum2_a[1] = std::abs(rtb_G1[5]);
    rtb_n_dt_0[1] = std::abs(rtb_G1[6]);
    rtb_y_l[1] = std::abs(rtb_G1[7]);
    umax[2] = std::abs(rtb_G1[8]);
    rtb_Sum2_a[2] = std::abs(rtb_G1[9]);
    rtb_n_dt_0[2] = std::abs(rtb_G1[10]);
    rtb_y_l[2] = std::abs(rtb_G1[11]);
    umax[3] = std::abs(rtb_G1[12]);
    rtb_Sum2_a[3] = std::abs(rtb_G1[13]);
    rtb_n_dt_0[3] = std::abs(rtb_G1[14]);
    rtb_y_l[3] = std::abs(rtb_G1[15]);
    learn_rate[0] = 0.251710802F / (13.7400656F * mean(umax));
    learn_rate[1] = 0.251710802F / (13.7400656F * mean(rtb_Sum2_a));
    learn_rate[2] = 0.283174664F / (14.7078171F * mean(rtb_n_dt_0));
    learn_rate[3] = 0.125855401F / (1.2F * mean(rtb_y_l));

    // Sum: '<S54>/Add' incorporates:
    //   UnitDelay: '<S54>/Unit Delay'

    scale = rtDW.Add3[0] - rtDW.UnitDelay_DSTATE_a[0];
    absxk = rtDW.Add3[1] - rtDW.UnitDelay_DSTATE_a[1];
    q0_q1 = rtDW.Add3[2] - rtDW.UnitDelay_DSTATE_a[2];
    rtb_Gain6 = rtDW.Add3[3] - rtDW.UnitDelay_DSTATE_a[3];
    for (b_k = 0; b_k < 4; b_k++) {
      // Product: '<S53>/Matrix Multiply5'
      q2_q3 = rtb_G1[b_k + 12] * rtDW.Add3[3] + (rtb_G1[b_k + 8] * rtDW.Add3[2]
        + (rtb_G1[b_k + 4] * rtDW.Add3[1] + rtb_G1[b_k] * rtDW.Add3[0]));

      // Product: '<S45>/apply learn rate' incorporates:
      //   Gain: '<S45>/Gain'
      //   MATLAB Function: '<S45>/G1 learn rate'
      //   Product: '<S45>/Product'
      //   Sum: '<S55>/Add2'
      //   Sum: '<S55>/Add3'

      rtDW.Delta_factors_G1[b_k] = -((rtDW.MatrixMultiply2[b_k] -
        (rtDW.Delta_nu_measure[b_k] - rtDW.MatrixMultiply3[b_k])) * q2_q3) *
        (real32_T)learn_rate[b_k];

      // MATLAB Function: '<S45>/G2 learn rate'
      umax[b_k] = std::abs(rtb_G2[(b_k << 2) + 2]);

      // Product: '<S54>/Matrix Multiply7'
      q2_q3 = rtb_G2[b_k + 12] * rtb_Gain6 + (rtb_G2[b_k + 8] * q0_q1 +
        (rtb_G2[b_k + 4] * absxk + rtb_G2[b_k] * scale));

      // Update for UnitDelay: '<S54>/Unit Delay'
      rtDW.UnitDelay_DSTATE_a[b_k] = rtDW.Add3[b_k];

      // Product: '<S54>/Matrix Multiply7'
      rtb_n_dt_0[b_k] = q2_q3;
    }

    // Product: '<S45>/apply learn rate2' incorporates:
    //   Constant: '<S45>/Constant10'
    //   Gain: '<S45>/Gain1'
    //   MATLAB Function: '<S45>/G2 learn rate'
    //   Product: '<S45>/Product4'
    //   Sum: '<S55>/Add4'
    //   Sum: '<S55>/Add5'

    rtDW.Delta_factors_G2 = 0.188783109F / ((((umax[0] + umax[1]) + umax[2]) +
      umax[3]) / 4.0F * 0.462764531F) * -((rtDW.MatrixMultiply3[2] -
      (rtDW.Delta_nu_measure[2] - rtDW.MatrixMultiply2[2])) * rtb_n_dt_0[2]);
  } else {
    if (rtDW.AdaptiveINDIG1andG2correction_M) {
      // Disable for Outport: '<S45>/Delta_factors_G1'
      rtDW.Delta_factors_G1[0] = 0.0F;
      rtDW.Delta_factors_G1[1] = 0.0F;
      rtDW.Delta_factors_G1[2] = 0.0F;
      rtDW.Delta_factors_G1[3] = 0.0F;

      // Disable for Outport: '<S45>/Delta_factors_G2'
      rtDW.Delta_factors_G2 = 0.0F;
      rtDW.AdaptiveINDIG1andG2correction_M = false;
    }
  }

  // End of Outputs for SubSystem: '<S44>/INDI Inversion Check'
  // End of Outputs for SubSystem: '<S44>/Adaptive INDI G1 and G2 correction'

  // Sum: '<S95>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S95>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S95>/Discrete-Time Integrator y_dt'
  //   Product: '<S95>/Product2'
  //   Sum: '<S95>/Sum3'

  for (i = 0; i < 9; i++) {
    rtb_M_bg[i] -= rtDW.DiscreteTimeIntegratory_dt_DS_b[i] * 0.0269397534F +
      rtDW.DiscreteTimeIntegratory_DSTAT_p[i];
  }

  // End of Sum: '<S95>/Sum2'

  // Sum: '<S67>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S67>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S67>/Discrete-Time Integrator y_dt'
  //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator'
  //   Product: '<S67>/Product2'
  //   Sum: '<S67>/Sum3'

  rtb_Sum2_a[0] = rtDW.DiscreteTimeIntegrator_DSTATE_e[0] -
    (rtDW.DiscreteTimeIntegratory_dt_DS_l[0] * 0.0269397534F +
     rtDW.DiscreteTimeIntegratory_DSTAT_n[0]);
  rtb_Sum2_a[1] = rtDW.DiscreteTimeIntegrator_DSTATE_e[1] -
    (rtDW.DiscreteTimeIntegratory_dt_DS_l[1] * 0.0269397534F +
     rtDW.DiscreteTimeIntegratory_DSTAT_n[1]);
  rtb_Sum2_a[2] = rtDW.DiscreteTimeIntegrator_DSTATE_e[2] -
    (rtDW.DiscreteTimeIntegratory_dt_DS_l[2] * 0.0269397534F +
     rtDW.DiscreteTimeIntegratory_DSTAT_n[2]);
  rtb_Sum2_a[3] = rtDW.DiscreteTimeIntegrator_DSTATE_e[3] -
    (rtDW.DiscreteTimeIntegratory_dt_DS_l[3] * 0.0269397534F +
     rtDW.DiscreteTimeIntegratory_DSTAT_n[3]);

  // Outport: '<Root>/u' incorporates:
  //   Gain: '<Root>/Gain1'
  //   Gain: '<Root>/Gain2'
  //   Gain: '<Root>/Gain3'
  //   Gain: '<Root>/Gain4'

  rtY.u[0] = rtb_q_red_b[1];
  rtY.u[1] = q1_q1;
  rtY.u[2] = rtb_q_red_b[0];
  rtY.u[3] = rtb_q_red_b[2];
  rtY.u[4] = 0.0F;
  rtY.u[5] = 0.0F;
  rtY.u[6] = 0.0F;
  rtY.u[7] = 0.0F;

  // Product: '<S96>/Product1' incorporates:
  //   DiscreteIntegrator: '<S96>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S96>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/measure'
  //   Product: '<S96>/Product2'
  //   Sum: '<S96>/Sum2'
  //   Sum: '<S96>/Sum3'

  rtb_s_g_dt2_ref[0] = (rtU.measure.V_Kg[0] -
                        (rtDW.DiscreteTimeIntegratory_dt_DS_n[0] * 0.0269397534F
    + rtDW.DiscreteTimeIntegratory_DSTAT_a[0])) * 5511.5376F;

  // Sum: '<S97>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S97>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S97>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/measure'
  //   Product: '<S97>/Product2'
  //   Sum: '<S97>/Sum3'

  rtb_Sum2_e[0] = rtU.measure.s_Kg[0] - (rtDW.DiscreteTimeIntegratory_dt_D_nb[0]
    * 0.0269397534F + rtDW.DiscreteTimeIntegratory_DSTATE[0]);

  // Sum: '<S94>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S94>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S94>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/measure'
  //   Product: '<S94>/Product2'
  //   Sum: '<Root>/Add'
  //   Sum: '<S94>/Sum3'

  scale = rtU.measure.a_Kg[0] - (rtDW.DiscreteTimeIntegratory_dt_D_o1[0] *
    0.0269397534F + rtDW.DiscreteTimeIntegratory_DSTA_al[0]);

  // Product: '<S96>/Product1' incorporates:
  //   DiscreteIntegrator: '<S96>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S96>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/measure'
  //   Product: '<S96>/Product2'
  //   Sum: '<S96>/Sum2'
  //   Sum: '<S96>/Sum3'

  rtb_s_g_dt2_ref[1] = (rtU.measure.V_Kg[1] -
                        (rtDW.DiscreteTimeIntegratory_dt_DS_n[1] * 0.0269397534F
    + rtDW.DiscreteTimeIntegratory_DSTAT_a[1])) * 5511.5376F;

  // Sum: '<S97>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S97>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S97>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/measure'
  //   Product: '<S97>/Product2'
  //   Sum: '<S97>/Sum3'

  rtb_Sum2_e[1] = rtU.measure.s_Kg[1] - (rtDW.DiscreteTimeIntegratory_dt_D_nb[1]
    * 0.0269397534F + rtDW.DiscreteTimeIntegratory_DSTATE[1]);

  // Sum: '<S94>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S94>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S94>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/measure'
  //   Product: '<S94>/Product2'
  //   Sum: '<Root>/Add'
  //   Sum: '<S94>/Sum3'

  q0_q1 = rtU.measure.a_Kg[1] - (rtDW.DiscreteTimeIntegratory_dt_D_o1[1] *
    0.0269397534F + rtDW.DiscreteTimeIntegratory_DSTA_al[1]);
  rtb_Gain6 = (rtU.measure.a_Kg[2] + 9.81F) -
    (rtDW.DiscreteTimeIntegratory_dt_D_o1[2] * 0.0269397534F +
     rtDW.DiscreteTimeIntegratory_DSTA_al[2]);

  // Product: '<S96>/Product1' incorporates:
  //   DiscreteIntegrator: '<S96>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S96>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/measure'
  //   Product: '<S96>/Product2'
  //   Sum: '<S96>/Sum2'
  //   Sum: '<S96>/Sum3'

  rtb_s_g_dt2_ref[2] = (rtU.measure.V_Kg[2] -
                        (rtDW.DiscreteTimeIntegratory_dt_DS_n[2] * 0.0269397534F
    + rtDW.DiscreteTimeIntegratory_DSTAT_a[2])) * 5511.5376F;

  // Sum: '<S97>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S97>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S97>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/measure'
  //   Product: '<S97>/Product2'
  //   Sum: '<S97>/Sum3'

  rtb_Sum2_e[2] = rtU.measure.s_Kg[2] - (rtDW.DiscreteTimeIntegratory_dt_D_nb[2]
    * 0.0269397534F + rtDW.DiscreteTimeIntegratory_DSTATE[2]);

  // Saturate: '<S48>/Saturation1'
  if (rtDW.Delta_factors_G1[0] > 0.1F) {
    absxk = 0.1F;
  } else if (rtDW.Delta_factors_G1[0] < -0.1F) {
    absxk = -0.1F;
  } else {
    absxk = rtDW.Delta_factors_G1[0];
  }

  // Saturate: '<S48>/Saturation2' incorporates:
  //   Delay: '<S48>/Delay'
  //   Saturate: '<S48>/Saturation1'
  //   Sum: '<S48>/Add'

  absxk += rtDW.Delay_DSTATE[0];
  if (absxk > 4.0F) {
    // Update for Delay: '<S48>/Delay'
    rtDW.Delay_DSTATE[0] = 4.0F;
  } else if (absxk < 0.25F) {
    // Update for Delay: '<S48>/Delay'
    rtDW.Delay_DSTATE[0] = 0.25F;
  } else {
    // Update for Delay: '<S48>/Delay'
    rtDW.Delay_DSTATE[0] = absxk;
  }

  // Saturate: '<S48>/Saturation1'
  if (rtDW.Delta_factors_G1[1] > 0.1F) {
    absxk = 0.1F;
  } else if (rtDW.Delta_factors_G1[1] < -0.1F) {
    absxk = -0.1F;
  } else {
    absxk = rtDW.Delta_factors_G1[1];
  }

  // Saturate: '<S48>/Saturation2' incorporates:
  //   Delay: '<S48>/Delay'
  //   Saturate: '<S48>/Saturation1'
  //   Sum: '<S48>/Add'

  absxk += rtDW.Delay_DSTATE[1];
  if (absxk > 4.0F) {
    // Update for Delay: '<S48>/Delay'
    rtDW.Delay_DSTATE[1] = 4.0F;
  } else if (absxk < 0.25F) {
    // Update for Delay: '<S48>/Delay'
    rtDW.Delay_DSTATE[1] = 0.25F;
  } else {
    // Update for Delay: '<S48>/Delay'
    rtDW.Delay_DSTATE[1] = absxk;
  }

  // Saturate: '<S48>/Saturation1'
  if (rtDW.Delta_factors_G1[2] > 0.1F) {
    absxk = 0.1F;
  } else if (rtDW.Delta_factors_G1[2] < -0.1F) {
    absxk = -0.1F;
  } else {
    absxk = rtDW.Delta_factors_G1[2];
  }

  // Saturate: '<S48>/Saturation2' incorporates:
  //   Delay: '<S48>/Delay'
  //   Saturate: '<S48>/Saturation1'
  //   Sum: '<S48>/Add'

  absxk += rtDW.Delay_DSTATE[2];
  if (absxk > 4.0F) {
    // Update for Delay: '<S48>/Delay'
    rtDW.Delay_DSTATE[2] = 4.0F;
  } else if (absxk < 0.25F) {
    // Update for Delay: '<S48>/Delay'
    rtDW.Delay_DSTATE[2] = 0.25F;
  } else {
    // Update for Delay: '<S48>/Delay'
    rtDW.Delay_DSTATE[2] = absxk;
  }

  // Saturate: '<S48>/Saturation1'
  if (rtDW.Delta_factors_G1[3] > 0.1F) {
    absxk = 0.1F;
  } else if (rtDW.Delta_factors_G1[3] < -0.1F) {
    absxk = -0.1F;
  } else {
    absxk = rtDW.Delta_factors_G1[3];
  }

  // Saturate: '<S48>/Saturation2' incorporates:
  //   Delay: '<S48>/Delay'
  //   Saturate: '<S48>/Saturation1'
  //   Sum: '<S48>/Add'

  absxk += rtDW.Delay_DSTATE[3];
  if (absxk > 4.0F) {
    // Update for Delay: '<S48>/Delay'
    rtDW.Delay_DSTATE[3] = 4.0F;
  } else if (absxk < 0.25F) {
    // Update for Delay: '<S48>/Delay'
    rtDW.Delay_DSTATE[3] = 0.25F;
  } else {
    // Update for Delay: '<S48>/Delay'
    rtDW.Delay_DSTATE[3] = absxk;
  }

  // Saturate: '<S49>/Saturation1'
  if (rtDW.Delta_factors_G2 > 0.1F) {
    absxk = 0.1F;
  } else if (rtDW.Delta_factors_G2 < -0.1F) {
    absxk = -0.1F;
  } else {
    absxk = rtDW.Delta_factors_G2;
  }

  // End of Saturate: '<S49>/Saturation1'

  // Sum: '<S49>/Add1' incorporates:
  //   Delay: '<S49>/Delay1'

  absxk += rtDW.Delay1_DSTATE;

  // Saturate: '<S49>/Saturation3'
  if (absxk > 4.0F) {
    // Update for Delay: '<S49>/Delay1'
    rtDW.Delay1_DSTATE = 4.0F;
  } else if (absxk < 0.25F) {
    // Update for Delay: '<S49>/Delay1'
    rtDW.Delay1_DSTATE = 0.25F;
  } else {
    // Update for Delay: '<S49>/Delay1'
    rtDW.Delay1_DSTATE = absxk;
  }

  // End of Saturate: '<S49>/Saturation3'

  // Update for UnitDelay: '<S39>/Unit Delay' incorporates:
  //   MATLAB Function: '<S32>/pick if traj is valid'

  rtDW.UnitDelay_DSTATE = state_vec[4];

  // Update for DiscreteIntegrator: '<S97>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LOAD = 0U;

  // Update for DiscreteIntegrator: '<S96>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_n = 0U;

  // Update for UnitDelay: '<S32>/Unit Delay' incorporates:
  //   MATLAB Function: '<S32>/pick if traj is valid'

  rtDW.UnitDelay_DSTATE_c = state_vec[4];

  // Update for DiscreteIntegrator: '<S97>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S97>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTATE[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_D_nb[0];

  // Update for DiscreteIntegrator: '<S96>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S96>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_a[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_n[0];

  // Update for DiscreteIntegrator: '<S94>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S94>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTA_al[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_D_o1[0];

  // Update for DiscreteIntegrator: '<S98>/Discrete-Time Integrator y_dt' incorporates:
  //   DiscreteIntegrator: '<S98>/Discrete-Time Integrator y'
  //   Inport: '<Root>/measure'
  //   Product: '<S98>/Product1'
  //   Product: '<S98>/Product2'
  //   Sum: '<S98>/Sum2'
  //   Sum: '<S98>/Sum3'

  rtDW.DiscreteTimeIntegratory_dt_DSTA[0] += (rtU.measure.omega_Kb[0] -
    (rtDW.DiscreteTimeIntegratory_dt_DSTA[0] * 0.0269397534F +
     rtDW.DiscreteTimeIntegratory_DSTAT_k[0])) * 5511.5376F * 0.0025F;

  // Update for DiscreteIntegrator: '<S97>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S97>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTATE[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_D_nb[1];

  // Update for DiscreteIntegrator: '<S96>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S96>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_a[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_n[1];

  // Update for DiscreteIntegrator: '<S94>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S94>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTA_al[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_D_o1[1];

  // Update for DiscreteIntegrator: '<S98>/Discrete-Time Integrator y_dt' incorporates:
  //   DiscreteIntegrator: '<S98>/Discrete-Time Integrator y'
  //   Inport: '<Root>/measure'
  //   Product: '<S98>/Product1'
  //   Product: '<S98>/Product2'
  //   Sum: '<S98>/Sum2'
  //   Sum: '<S98>/Sum3'

  rtDW.DiscreteTimeIntegratory_dt_DSTA[1] += (rtU.measure.omega_Kb[1] -
    (rtDW.DiscreteTimeIntegratory_dt_DSTA[1] * 0.0269397534F +
     rtDW.DiscreteTimeIntegratory_DSTAT_k[1])) * 5511.5376F * 0.0025F;

  // Update for DiscreteIntegrator: '<S97>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S97>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTATE[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_D_nb[2];

  // Update for DiscreteIntegrator: '<S96>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S96>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_a[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_n[2];

  // Update for DiscreteIntegrator: '<S94>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S94>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTA_al[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_D_o1[2];

  // Update for DiscreteIntegrator: '<S98>/Discrete-Time Integrator y_dt' incorporates:
  //   DiscreteIntegrator: '<S98>/Discrete-Time Integrator y'
  //   Inport: '<Root>/measure'
  //   Product: '<S98>/Product1'
  //   Product: '<S98>/Product2'
  //   Sum: '<S98>/Sum2'
  //   Sum: '<S98>/Sum3'

  rtDW.DiscreteTimeIntegratory_dt_DSTA[2] += (rtU.measure.omega_Kb[2] -
    (rtDW.DiscreteTimeIntegratory_dt_DSTA[2] * 0.0269397534F +
     rtDW.DiscreteTimeIntegratory_DSTAT_k[2])) * 5511.5376F * 0.0025F;

  // Update for DiscreteIntegrator: '<S95>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_b = 0U;

  // Update for DiscreteIntegrator: '<S121>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_i = 0U;

  // Update for DiscreteIntegrator: '<S122>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_a = 0U;

  // Update for DiscreteIntegrator: '<S98>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_d = 0U;

  // Update for DiscreteIntegrator: '<S121>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S121>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_m[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_o[0];

  // Update for DiscreteIntegrator: '<S121>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_o[0] += 0.0025F * rtb_y_dt_g[0];

  // Update for DiscreteIntegrator: '<S122>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_e[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_j[0];

  // Update for DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_j[0] += 0.0025F * rtb_omega2_f[0];

  // Update for DiscreteIntegrator: '<S98>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_DSTAT_k[0] += 0.0025F * rtb_y_p[0];

  // Update for DiscreteIntegrator: '<S121>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S121>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_m[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_o[1];

  // Update for DiscreteIntegrator: '<S121>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_o[1] += 0.0025F * rtb_y_dt_g[1];

  // Update for DiscreteIntegrator: '<S122>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_e[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_j[1];

  // Update for DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_j[1] += 0.0025F * rtb_omega2_f[1];

  // Update for DiscreteIntegrator: '<S98>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_DSTAT_k[1] += 0.0025F * rtb_y_p[1];

  // Update for DiscreteIntegrator: '<S121>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S121>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_m[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_o[2];

  // Update for DiscreteIntegrator: '<S121>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_o[2] += 0.0025F * q1_q3;

  // Update for DiscreteIntegrator: '<S122>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_e[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_j[2];

  // Update for DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_j[2] += 0.0025F * rtb_omega2_f[2];

  // Update for DiscreteIntegrator: '<S98>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_DSTAT_k[2] += 0.0025F * rtb_y_p[2];

  // Update for DiscreteIntegrator: '<S126>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE += 0.0025F * rtb_y_lm;

  // Update for DiscreteIntegrator: '<S108>/Discrete-Time Integrator2'
  rtDW.DiscreteTimeIntegrator2_IC_LOAD = 0U;
  rtDW.DiscreteTimeIntegrator2_DSTATE += 0.0025F * t;

  // Update for DiscreteIntegrator: '<S73>/Discrete-Time Integrator' incorporates:
  //   Gain: '<S73>/1//T'
  //   Sum: '<S73>/Sum2'

  rtDW.DiscreteTimeIntegrator_DSTATE_e[0] += (rtb_q_red_b[0] -
    rtDW.DiscreteTimeIntegrator_DSTATE_e[0]) * 19.0416317F * 0.0025F;

  // Update for UnitDelay: '<S64>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[0] = rtb_Add_a[0];

  // Update for DiscreteIntegrator: '<S67>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S67>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_n[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_l[0];

  // Update for DiscreteIntegrator: '<S67>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S67>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_DS_l[0] += rtb_Sum2_a[0] * 5511.5376F *
    0.0025F;

  // Update for DiscreteIntegrator: '<S73>/Discrete-Time Integrator' incorporates:
  //   Gain: '<S73>/1//T'
  //   Sum: '<S73>/Sum2'

  rtDW.DiscreteTimeIntegrator_DSTATE_e[1] += (rtb_q_red_b[1] -
    rtDW.DiscreteTimeIntegrator_DSTATE_e[1]) * 19.0416317F * 0.0025F;

  // Update for UnitDelay: '<S64>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[1] = rtb_Add_a[1];

  // Update for DiscreteIntegrator: '<S67>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S67>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_n[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_l[1];

  // Update for DiscreteIntegrator: '<S67>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S67>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_DS_l[1] += rtb_Sum2_a[1] * 5511.5376F *
    0.0025F;

  // Update for DiscreteIntegrator: '<S73>/Discrete-Time Integrator' incorporates:
  //   Gain: '<S73>/1//T'
  //   Sum: '<S73>/Sum2'

  rtDW.DiscreteTimeIntegrator_DSTATE_e[2] += (rtb_q_red_b[2] -
    rtDW.DiscreteTimeIntegrator_DSTATE_e[2]) * 19.0416317F * 0.0025F;

  // Update for UnitDelay: '<S64>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[2] = rtb_Add_a[2];

  // Update for DiscreteIntegrator: '<S67>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S67>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_n[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_l[2];

  // Update for DiscreteIntegrator: '<S67>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S67>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_DS_l[2] += rtb_Sum2_a[2] * 5511.5376F *
    0.0025F;

  // Update for DiscreteIntegrator: '<S73>/Discrete-Time Integrator' incorporates:
  //   Gain: '<S73>/1//T'
  //   Sum: '<S73>/Sum2'

  rtDW.DiscreteTimeIntegrator_DSTATE_e[3] += (q1_q1 -
    rtDW.DiscreteTimeIntegrator_DSTATE_e[3]) * 19.0416317F * 0.0025F;

  // Update for UnitDelay: '<S64>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[3] = rtb_Add_a[3];

  // Update for DiscreteIntegrator: '<S67>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S67>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_n[3] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_l[3];

  // Update for DiscreteIntegrator: '<S67>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S67>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_DS_l[3] += rtb_Sum2_a[3] * 5511.5376F *
    0.0025F;
  for (i = 0; i < 9; i++) {
    // Update for DiscreteIntegrator: '<S95>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S95>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_p[i] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_b[i];

    // Update for DiscreteIntegrator: '<S95>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S95>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_DS_b[i] += rtb_M_bg[i] * 5511.5376F *
      0.0025F;
  }

  // Update for DiscreteIntegrator: '<S94>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S94>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_D_o1[0] += scale * 5511.5376F * 0.0025F;

  // Update for DiscreteIntegrator: '<S96>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_n[0] += 0.0025F * rtb_s_g_dt2_ref[0];

  // Update for DiscreteIntegrator: '<S97>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S97>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_D_nb[0] += rtb_Sum2_e[0] * 5511.5376F *
    0.0025F;

  // Update for DiscreteIntegrator: '<S94>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S94>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_D_o1[1] += q0_q1 * 5511.5376F * 0.0025F;

  // Update for DiscreteIntegrator: '<S96>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_n[1] += 0.0025F * rtb_s_g_dt2_ref[1];

  // Update for DiscreteIntegrator: '<S97>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S97>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_D_nb[1] += rtb_Sum2_e[1] * 5511.5376F *
    0.0025F;

  // Update for DiscreteIntegrator: '<S94>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S94>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_D_o1[2] += rtb_Gain6 * 5511.5376F * 0.0025F;

  // Update for DiscreteIntegrator: '<S96>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_n[2] += 0.0025F * rtb_s_g_dt2_ref[2];

  // Update for DiscreteIntegrator: '<S97>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S97>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_D_nb[2] += rtb_Sum2_e[2] * 5511.5376F *
    0.0025F;
}

// Model initialize function
void MatlabControllerClass::initialize()
{
  // InitializeConditions for Delay: '<S48>/Delay'
  rtDW.Delay_DSTATE[0] = 1.0F;
  rtDW.Delay_DSTATE[1] = 1.0F;
  rtDW.Delay_DSTATE[2] = 1.0F;
  rtDW.Delay_DSTATE[3] = 1.0F;

  // InitializeConditions for Delay: '<S49>/Delay1'
  rtDW.Delay1_DSTATE = 1.0F;

  // InitializeConditions for DiscreteIntegrator: '<S97>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LOAD = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S96>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_n = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S95>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_b = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S121>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_i = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S122>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_a = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S98>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_d = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S108>/Discrete-Time Integrator2' 
  rtDW.DiscreteTimeIntegrator2_IC_LOAD = 1U;

  // SystemInitialize for Iterator SubSystem: '<S32>/Trajectory from Waypoints'
  // InitializeConditions for Memory: '<S75>/TrajMemory'
  rtDW.TrajMemory_PreviousInput = rtConstP.pooled3;

  // End of SystemInitialize for SubSystem: '<S32>/Trajectory from Waypoints'

  // SystemInitialize for Enabled SubSystem: '<S3>/fast descent sequencer'
  // SystemInitialize for Enabled SubSystem: '<S11>/Duration in Intercept Arc'
  // InitializeConditions for DiscreteIntegrator: '<S17>/Discrete-Time Integrator' 
  rtDW.DiscreteTimeIntegrator_PrevRe_m = 2;

  // End of SystemInitialize for SubSystem: '<S11>/Duration in Intercept Arc'

  // SystemInitialize for Enabled SubSystem: '<S11>/Throttle Intercept Arc (Load factor controller)' 
  // InitializeConditions for DiscreteIntegrator: '<S27>/Discrete-Time Integrator' 
  rtDW.DiscreteTimeIntegrator_DSTATE_g = 0.7F;

  // End of SystemInitialize for SubSystem: '<S11>/Throttle Intercept Arc (Load factor controller)' 
  // End of SystemInitialize for SubSystem: '<S3>/fast descent sequencer'

  // SystemInitialize for Enabled SubSystem: '<S4>/NDI position controller for copters reference model' 
  // InitializeConditions for DiscreteIntegrator: '<S131>/Discrete-Time Integrator' 
  rtDW.DiscreteTimeIntegrator_IC_LOADI = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S36>/Discrete-Time Integrator' 
  rtDW.DiscreteTimeIntegrator_IC_LOA_o = 1U;

  // End of SystemInitialize for SubSystem: '<S4>/NDI position controller for copters reference model' 

  // SystemInitialize for Enabled SubSystem: '<S4>/NDI position controller for copters with reference input' 
  // InitializeConditions for DiscreteIntegrator: '<S134>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_L_b2 = 1U;

  // End of SystemInitialize for SubSystem: '<S4>/NDI position controller for copters with reference input' 

  // SystemInitialize for Enabled SubSystem: '<S44>/Copter Random Excitation'
  // InitializeConditions for RandomNumber: '<S56>/White Noise'
  rtDW.RandSeed[0] = 1529675776U;
  rtDW.NextOutput[0] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[0]);
  rtDW.RandSeed[1] = 1529741312U;
  rtDW.NextOutput[1] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[1]);
  rtDW.RandSeed[2] = 1529806848U;
  rtDW.NextOutput[2] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[2]);
  rtDW.RandSeed[3] = 1529872384U;
  rtDW.NextOutput[3] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[3]);

  // End of SystemInitialize for SubSystem: '<S44>/Copter Random Excitation'

  // SystemInitialize for Enabled SubSystem: '<S4>/Incremental specific thrust'
  // InitializeConditions for DiscreteIntegrator: '<S89>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_e = 1U;

  // End of SystemInitialize for SubSystem: '<S4>/Incremental specific thrust'

  // SystemInitialize for Enabled SubSystem: '<S44>/INDI Inversion Check'
  // InitializeConditions for DiscreteIntegrator: '<S62>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_L_eh = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S61>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_L_ir = 1U;

  // End of SystemInitialize for SubSystem: '<S44>/INDI Inversion Check'
}

// Constructor
MatlabControllerClass::MatlabControllerClass()
{
  // Currently there is no constructor body generated.
}

// Destructor
MatlabControllerClass::~MatlabControllerClass()
{
  // Currently there is no destructor body generated.
}

//
// File trailer for generated code.
//
// [EOF]
//

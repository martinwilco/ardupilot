//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController_data.cpp
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

// Constant parameters (default storage)
const ConstP rtConstP = {
  // Pooled Parameter (Mixed Expressions)
  //  Referenced by:
  //    '<S75>/Constant1'
  //    '<S75>/TrajMemory'

  {
    4.0F,
    0.0F,

    {
      {
        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },
        0.0F,
        0.0F,
        0.0F,
        5.0F
      }, {
        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },
        0.0F,
        0.0F,
        0.0F,
        5.0F
      }, {
        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },
        0.0F,
        0.0F,
        0.0F,
        5.0F
      }, {
        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },
        0.0F,
        0.0F,
        0.0F,
        5.0F
      } }
    ,
    0.0F,
    0.0F,
    0.0F,
    0.0F,
    0,
    5.0F
  },

  // Expression: ca
  //  Referenced by: '<S31>/Control Allocation Vertical Acc Weighting'

  {
    { 0.1F, 0.1F, 0.1F, 0.1F },

    { 1.0F, 1.0F, 1.0F, 1.0F },

    { 0.1F, 0.1F, 0.1F, 0.1F },

    { 30.0F, 0.0F, 0.0F, 0.0F, 0.0F, 30.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.01F, 0.0F,
      0.0F, 0.0F, 0.0F, 300.0F },

    { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 1.0F },
    1000.0F,

    { 0.0F, 0.0F, 0.0F, 0.0F },
    100.0F
  },

  // Expression: ca
  //  Referenced by: '<S69>/caIndiWls'

  {
    { 0.1F, 0.1F, 0.1F, 0.1F },

    { 1.0F, 1.0F, 1.0F, 1.0F },

    { 0.1F, 0.1F, 0.1F, 0.1F },

    { 30.0F, 0.0F, 0.0F, 0.0F, 0.0F, 30.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.01F, 0.0F,
      0.0F, 0.0F, 0.0F, 300.0F },

    { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 1.0F },
    1000.0F,

    { 0.0F, 0.0F, 0.0F, 0.0F },
    100.0F
  },

  // Expression: ca
  //  Referenced by: '<S64>/Set Vertical Acc Weight To Zero'

  {
    { 0.1F, 0.1F, 0.1F, 0.1F },

    { 1.0F, 1.0F, 1.0F, 1.0F },

    { 0.1F, 0.1F, 0.1F, 0.1F },

    { 30.0F, 0.0F, 0.0F, 0.0F, 0.0F, 30.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.01F, 0.0F,
      0.0F, 0.0F, 0.0F, 300.0F },

    { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 1.0F },
    1000.0F,

    { 0.0F, 0.0F, 0.0F, 0.0F },
    100.0F
  },

  // Expression: G10
  //  Referenced by: '<S68>/MATLAB Function'

  { 0.000231342128F, 0.000238571563F, -1.64946705E-5F, -1.51305894E-5F,
    -0.000231342128F, 0.000238571563F, 1.64946705E-5F, -1.51305894E-5F,
    -0.000231342128F, -0.000238571563F, -1.64946705E-5F, -1.51305894E-5F,
    0.000231342128F, -0.000238571563F, 1.64946705E-5F, -1.51305894E-5F },

  // Expression: G20
  //  Referenced by: '<S68>/MATLAB Function'

  { -6.76235966E-17F, -0.0F, -0.657841921F, 0.0F, 6.76235966E-17F, 0.0F,
    0.657841921F, 0.0F, -6.76235966E-17F, -0.0F, -0.657841921F, 0.0F,
    6.76235966E-17F, 0.0F, 0.657841921F, 0.0F },

  // Expression: [psc.rm.veldmax,0,-psc.rm.velumax]
  //  Referenced by: '<S36>/1-D Lookup Table'

  { 4.73770285F, 0.0F, -16.7692528F },

  // Computed Parameter: uDLookupTable_bp01Data
  //  Referenced by: '<S36>/1-D Lookup Table'

  { -1.0F, 0.0F, 1.0F }
};

//
// File trailer for generated code.
//
// [EOF]
//

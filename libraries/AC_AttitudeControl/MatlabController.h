//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.h
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
#ifndef RTW_HEADER_MatlabController_h_
#define RTW_HEADER_MatlabController_h_
#include "rtwtypes.h"
#include <cmath>
#include <string.h>
#ifndef ArduCopter_Fast_Descent_COMMON_INCLUDES_
# define ArduCopter_Fast_Descent_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // ArduCopter_Fast_Descent_COMMON_INCLUDES_ 

// Macros for accessing real-time model data structure
#ifndef DEFINED_TYPEDEF_FOR_cmdBus_
#define DEFINED_TYPEDEF_FOR_cmdBus_

typedef struct {
  real32_T roll;
  real32_T pitch;
  real32_T yaw;
  real32_T thr;
  real32_T s_Kg_init[3];
  real32_T yaw_init;
  uint16_T mission_change;
  real32_T waypoints[40];
  uint16_T num_waypoints;
  real32_T RC_pwm[16];
} cmdBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_measureBus_
#define DEFINED_TYPEDEF_FOR_measureBus_

typedef struct {
  real32_T omega_Kb[3];
  real32_T EulerAngles[3];
  real32_T q_bg[4];
  real32_T a_Kg[3];
  real32_T a_Kb[3];
  real32_T V_Kg[3];
  real32_T s_Kg[3];
  real32_T s_Kg_origin[3];
  real32_T lla[3];
  real32_T rangefinder[6];
  real32_T V_bat;
  real32_T omega_mot[4];
  real32_T airspeed;
} measureBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_dtoSgl_sections_bus_
#define DEFINED_TYPEDEF_FOR_dtoSgl_sections_bus_

typedef struct {
  real32_T pos_x[6];
  real32_T pos_y[6];
  real32_T pos_z[6];
  real32_T vel[6];
  real32_T t;
  real32_T arc_length;
  real32_T distance;
  real32_T polynomial_degree;
} dtoSgl_sections_bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_dtoSgl_trajectoryStructBus_
#define DEFINED_TYPEDEF_FOR_dtoSgl_trajectoryStructBus_

typedef struct {
  real32_T num_sections_max;
  real32_T num_sections_set;
  dtoSgl_sections_bus sections[4];
  real32_T active_section;
  real32_T current_time;
  real32_T arc_length;
  real32_T distance;
  boolean_T is_repeated_course;
  real32_T polynomial_degree;
} dtoSgl_trajectoryStructBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_lindiStateLogicBus_
#define DEFINED_TYPEDEF_FOR_lindiStateLogicBus_

typedef struct {
  boolean_T isPscEnabled;
  boolean_T isPosRmEnabled;
  boolean_T isVertPscEnabled;
  boolean_T isGdnceEnabled;
  boolean_T isAttiCmdEnabled;
  boolean_T isManThrEnabled;
  boolean_T isAutoTuneEnabled;
} lindiStateLogicBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_logConfigBus_
#define DEFINED_TYPEDEF_FOR_logConfigBus_

typedef struct {
  uint8_T num_signals;
  uint8_T signal_names[42];
  uint8_T batch_name[4];
} logConfigBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_sP1q3HDJE1lywg1PS8jMsG_
#define DEFINED_TYPEDEF_FOR_struct_sP1q3HDJE1lywg1PS8jMsG_

typedef struct {
  real32_T leanmax;
  real32_T leandamp;
  real32_T yawratemax;
  real32_T leanfreq;
  real32_T yawratetc;
} struct_sP1q3HDJE1lywg1PS8jMsG;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_fmp4nqBWC4rULgUM9YPHLD_
#define DEFINED_TYPEDEF_FOR_struct_fmp4nqBWC4rULgUM9YPHLD_

typedef struct {
  real32_T u_min[4];
  real32_T u_max[4];
  real32_T u_d[4];
  real32_T W_v[16];
  real32_T W_u[16];
  real32_T gamma;
  real32_T W[4];
  real32_T i_max;
} struct_fmp4nqBWC4rULgUM9YPHLD;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_p46j2fCGmBuwCvCy3vmiNE_
#define DEFINED_TYPEDEF_FOR_struct_p46j2fCGmBuwCvCy3vmiNE_

typedef struct {
  real32_T k;
  real32_T d;
  real32_T x[4];
  real32_T y[4];
  real32_T z[4];
  real32_T a[4];
  real32_T nx[4];
  real32_T ny[4];
  real32_T ip;
  real32_T kt;
  real32_T vb;
  real32_T ri;
} struct_p46j2fCGmBuwCvCy3vmiNE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_VAW73qcsmorMGOuFyCcx3B_
#define DEFINED_TYPEDEF_FOR_struct_VAW73qcsmorMGOuFyCcx3B_

typedef struct {
  real_T pos_x[6];
  real_T pos_y[6];
  real_T pos_z[6];
  real_T vel[6];
  real_T t;
  real_T arc_length;
  real_T distance;
  real32_T polynomial_degree;
} struct_VAW73qcsmorMGOuFyCcx3B;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_GcSYgHEScYXcSSyZ6ZxhGE_
#define DEFINED_TYPEDEF_FOR_struct_GcSYgHEScYXcSSyZ6ZxhGE_

typedef struct {
  real32_T num_sections_max;
  real_T num_sections_set;
  struct_VAW73qcsmorMGOuFyCcx3B sections[4];
  real_T active_section;
  real_T current_time;
  real_T arc_length;
  real_T distance;
  boolean_T is_repeated_course;
  real32_T polynomial_degree;
} struct_GcSYgHEScYXcSSyZ6ZxhGE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_hhWieQsLOg2nCzK7sanRFC_
#define DEFINED_TYPEDEF_FOR_struct_hhWieQsLOg2nCzK7sanRFC_

typedef struct {
  real32_T accumax;
  real32_T accdmax;
  real32_T veldmax;
  real32_T velumax;
  real32_T velxymax;
  real32_T accxymax;
  real32_T veltc;
} struct_hhWieQsLOg2nCzK7sanRFC;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_wtF2XFOtOELBemGDyRar7G_
#define DEFINED_TYPEDEF_FOR_struct_wtF2XFOtOELBemGDyRar7G_

typedef struct {
  real32_T pos;
  real32_T vel;
  real32_T acc;
} struct_wtF2XFOtOELBemGDyRar7G;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_U3vXfuAftwETLdXCHyHbn_
#define DEFINED_TYPEDEF_FOR_struct_U3vXfuAftwETLdXCHyHbn_

typedef struct {
  struct_hhWieQsLOg2nCzK7sanRFC rm;
  struct_wtF2XFOtOELBemGDyRar7G k;
} struct_U3vXfuAftwETLdXCHyHbn;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_drNsDinqwhrehoyPjCZYTD_
#define DEFINED_TYPEDEF_FOR_struct_drNsDinqwhrehoyPjCZYTD_

typedef struct {
  real32_T min[4];
  real32_T max[4];
} struct_drNsDinqwhrehoyPjCZYTD;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_t0r0dbw0Z6uQa2zAOkuWoH_
#define DEFINED_TYPEDEF_FOR_struct_t0r0dbw0Z6uQa2zAOkuWoH_

typedef struct {
  real32_T wpmax;
  boolean_T cycle;
  real32_T degree;
} struct_t0r0dbw0Z6uQa2zAOkuWoH;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_CVmGZLFPF0pRjVJ7mv3ro_
#define DEFINED_TYPEDEF_FOR_struct_CVmGZLFPF0pRjVJ7mv3ro_

typedef struct {
  real32_T lean;
  real32_T leanrate;
  real32_T leanacc;
  real32_T yaw;
  real32_T yawrate;
  real32_T yawacc;
} struct_CVmGZLFPF0pRjVJ7mv3ro;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_wPSyMEjWyBsuzVPD9drp3C_
#define DEFINED_TYPEDEF_FOR_struct_wPSyMEjWyBsuzVPD9drp3C_

typedef struct {
  struct_sP1q3HDJE1lywg1PS8jMsG rm;
  struct_CVmGZLFPF0pRjVJ7mv3ro k;
} struct_wPSyMEjWyBsuzVPD9drp3C;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_tuPgovHsB2lMDL0qC2CahD_
#define DEFINED_TYPEDEF_FOR_struct_tuPgovHsB2lMDL0qC2CahD_

typedef struct {
  real32_T m;
  real32_T ixx;
  real32_T iyy;
  real32_T izz;
  real32_T ixy;
  real32_T ixz;
  real32_T iyz;
} struct_tuPgovHsB2lMDL0qC2CahD;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_AjMJOgu1Si8jWnlljeCYkD_
#define DEFINED_TYPEDEF_FOR_struct_AjMJOgu1Si8jWnlljeCYkD_

typedef struct {
  real32_T D;
  real32_T omega;
} struct_AjMJOgu1Si8jWnlljeCYkD;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_1xfzvBCjDufmieZxJHR5mF_
#define DEFINED_TYPEDEF_FOR_struct_1xfzvBCjDufmieZxJHR5mF_

typedef struct {
  struct_fmp4nqBWC4rULgUM9YPHLD ca;
  struct_t0r0dbw0Z6uQa2zAOkuWoH traj;
  struct_drNsDinqwhrehoyPjCZYTD thr;
  struct_U3vXfuAftwETLdXCHyHbn psc;
  struct_wPSyMEjWyBsuzVPD9drp3C atc;
  struct_p46j2fCGmBuwCvCy3vmiNE cep;
  struct_tuPgovHsB2lMDL0qC2CahD ceb;
  real32_T mtc;
  real_T ts;
  struct_AjMJOgu1Si8jWnlljeCYkD sflt;
} struct_1xfzvBCjDufmieZxJHR5mF;

#endif

// Custom Type definition for MATLAB Function: '<S99>/DCM to quaternions'
#ifndef struct_tag_skA4KFEZ4HPkJJBOYCrevdH
#define struct_tag_skA4KFEZ4HPkJJBOYCrevdH

struct tag_skA4KFEZ4HPkJJBOYCrevdH
{
  uint32_T SafeEq;
  uint32_T Absolute;
  uint32_T NaNBias;
  uint32_T NaNWithFinite;
  uint32_T FiniteWithNaN;
  uint32_T NaNWithNaN;
};

#endif                                 //struct_tag_skA4KFEZ4HPkJJBOYCrevdH

#ifndef typedef_skA4KFEZ4HPkJJBOYCrevdH
#define typedef_skA4KFEZ4HPkJJBOYCrevdH

typedef struct tag_skA4KFEZ4HPkJJBOYCrevdH skA4KFEZ4HPkJJBOYCrevdH;

#endif                                 //typedef_skA4KFEZ4HPkJJBOYCrevdH

#ifndef struct_tag_sJCxfmxS8gBOONUZjbjUd9E
#define struct_tag_sJCxfmxS8gBOONUZjbjUd9E

struct tag_sJCxfmxS8gBOONUZjbjUd9E
{
  boolean_T CaseSensitivity;
  boolean_T StructExpand;
  char_T PartialMatching[6];
  boolean_T IgnoreNulls;
};

#endif                                 //struct_tag_sJCxfmxS8gBOONUZjbjUd9E

#ifndef typedef_sJCxfmxS8gBOONUZjbjUd9E
#define typedef_sJCxfmxS8gBOONUZjbjUd9E

typedef struct tag_sJCxfmxS8gBOONUZjbjUd9E sJCxfmxS8gBOONUZjbjUd9E;

#endif                                 //typedef_sJCxfmxS8gBOONUZjbjUd9E

#ifndef struct_emxArray_real32_T_1x5
#define struct_emxArray_real32_T_1x5

struct emxArray_real32_T_1x5
{
  real32_T data[5];
  int32_T size[2];
};

#endif                                 //struct_emxArray_real32_T_1x5

#ifndef typedef_emxArray_real32_T_1x5
#define typedef_emxArray_real32_T_1x5

typedef struct emxArray_real32_T_1x5 emxArray_real32_T_1x5;

#endif                                 //typedef_emxArray_real32_T_1x5

#ifndef struct_sKQLR94MqGzlZ6BjlviOSrB_tag
#define struct_sKQLR94MqGzlZ6BjlviOSrB_tag

struct sKQLR94MqGzlZ6BjlviOSrB_tag
{
  emxArray_real32_T_1x5 f1;
};

#endif                                 //struct_sKQLR94MqGzlZ6BjlviOSrB_tag

#ifndef typedef_b_cell_wrap_1
#define typedef_b_cell_wrap_1

typedef struct sKQLR94MqGzlZ6BjlviOSrB_tag b_cell_wrap_1;

#endif                                 //typedef_b_cell_wrap_1

// Block signals and states (default storage) for system '<Root>'
typedef struct {
  dtoSgl_trajectoryStructBus TrajMemory_PreviousInput;// '<S75>/TrajMemory'
  real_T NextOutput[4];                // '<S56>/White Noise'
  real32_T Merge[4];                   // '<S3>/Merge'
  real32_T s_g_ref[3];                 // '<S4>/Merge3'
  real32_T s_g_ref_dt[3];              // '<S4>/Merge3'
  real32_T s_g_ref_dt2[3];             // '<S4>/Merge3'
  real32_T n_g_des[3];                 // '<S4>/Merge'
  real32_T Add[9];                     // '<S133>/Add'
  real32_T nu[3];                      // '<S37>/Add1'
  real32_T s_g_ref_f[3];               // '<S37>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' 
  real32_T s_g[3];                     // '<S37>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' 
  real32_T s_g_dt[3];                  // '<S37>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' 
  real32_T s_g_dt2[3];                 // '<S37>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' 
  real32_T MatrixMultiply3[4];         // '<S58>/Matrix Multiply3'
  real32_T Add3[4];                    // '<S60>/Add3'
  real32_T MatrixMultiply2[4];         // '<S58>/Matrix Multiply2'
  real32_T Delta_nu_measure[4];        // '<S59>/Add2'
  real32_T Gain[3];                    // '<S46>/Gain'
  real32_T Delta_factors_G1[4];        // '<S45>/apply learn rate'
  real32_T Delay_DSTATE[4];            // '<S48>/Delay'
  real32_T DiscreteTimeIntegratory_DSTATE[3];// '<S97>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_a[3];// '<S96>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTA_al[3];// '<S94>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DSTA[3];// '<S98>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_p[9];// '<S95>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_m[3];// '<S121>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_o[3];// '<S121>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_e[3];// '<S122>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_j[3];// '<S122>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_k[3];// '<S98>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegrator_DSTATE_e[4];// '<S73>/Discrete-Time Integrator' 
  real32_T UnitDelay1_DSTATE[4];       // '<S64>/Unit Delay1'
  real32_T DiscreteTimeIntegratory_DSTAT_n[4];// '<S67>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_l[4];// '<S67>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_b[9];// '<S95>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_D_o1[3];// '<S94>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_n[3];// '<S96>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_D_nb[3];// '<S97>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_l[9];// '<S134>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_e[9];// '<S134>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegrator_DSTATE_l[3];// '<S131>/Discrete-Time Integrator' 
  real32_T DiscreteTimeIntegrator_DSTAT_em[3];// '<S36>/Discrete-Time Integrator' 
  real32_T DiscreteTimeIntegratory_DSTA_lk[3];// '<S89>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_d[3];// '<S89>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTA_n3[4];// '<S62>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_f[4];// '<S62>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_c[4];// '<S61>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_D_f1[4];// '<S61>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_d[4];// '<S57>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_h[4];// '<S57>/Discrete-Time Integrator y_dt' 
  real32_T UnitDelay_DSTATE_a[4];      // '<S54>/Unit Delay'
  real32_T uvwxbMemory_PreviousInput[120];// '<S75>/uvwxbMemory'
  real32_T AnormAlfaRhoPhiMemory_PreviousI[4];// '<S75>/AnormAlfaRhoPhiMemory'
  real32_T StateVecMemory_PreviousInput[6];// '<S75>/StateVecMemory '
  real32_T Merge1;                     // '<S3>/Merge1'
  real32_T cmd_lean_angle_01;          // '<S4>/Merge'
  real32_T lean_dir_angle_des;         // '<S4>/Merge'
  real32_T Merge1_e;                   // '<S4>/Merge1'
  real32_T Delta_nu_a_T;               // '<S33>/incremental thrust atti correction' 
  real32_T a_T_meas;                   // '<S33>/desired and measured specific thrust' 
  real32_T Gain1;                      // '<S46>/Gain1'
  real32_T Delta_factors_G2;           // '<S45>/apply learn rate2'
  real32_T Merge_m;                    // '<S11>/Merge'
  real32_T Merge1_k;                   // '<S11>/Merge1'
  real32_T t_abfang;                   // '<S19>/MATLAB Function'
  real32_T h_abfang;                   // '<S19>/MATLAB Function'
  real32_T DiscreteTimeIntegrator;     // '<S17>/Discrete-Time Integrator'
  real32_T Delay1_DSTATE;              // '<S49>/Delay1'
  real32_T UnitDelay_DSTATE;           // '<S39>/Unit Delay'
  real32_T UnitDelay_DSTATE_c;         // '<S32>/Unit Delay'
  real32_T DiscreteTimeIntegrator_DSTATE;// '<S126>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegrator2_DSTATE;// '<S108>/Discrete-Time Integrator2'
  real32_T UnitDelay3_DSTATE;          // '<S11>/Unit Delay3'
  real32_T DiscreteTimeIntegrator_DSTATE_g;// '<S27>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegrator_DSTAT_ld;// '<S17>/Discrete-Time Integrator'
  uint32_T RandSeed[4];                // '<S56>/White Noise'
  int8_T DiscreteTimeIntegrator_PrevRe_m;// '<S17>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegratory_IC_LOAD;// '<S97>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_n;// '<S96>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_b;// '<S95>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_i;// '<S121>/Discrete-Time Integrator y' 
  uint8_T DiscreteTimeIntegratory_IC_LO_a;// '<S122>/Discrete-Time Integrator y' 
  uint8_T DiscreteTimeIntegratory_IC_LO_d;// '<S98>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegrator2_IC_LOAD;// '<S108>/Discrete-Time Integrator2'
  uint8_T DiscreteTimeIntegratory_IC_L_b2;// '<S134>/Discrete-Time Integrator y' 
  uint8_T DiscreteTimeIntegrator_IC_LOADI;// '<S131>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegrator_IC_LOA_o;// '<S36>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegratory_IC_LO_e;// '<S89>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_L_eh;// '<S62>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_L_ir;// '<S61>/Discrete-Time Integrator y'
  boolean_T DelayInput1_DSTATE;        // '<S81>/Delay Input1'
  boolean_T UnitDelay_DSTATE_l;        // '<S11>/Unit Delay'
  boolean_T UnitDelay4_DSTATE;         // '<S11>/Unit Delay4'
  boolean_T UnitDelay2_DSTATE;         // '<S11>/Unit Delay2'
  boolean_T NDIpositioncontrollerforcopters;// '<S4>/NDI position controller for copters with reference input' 
  boolean_T NDIpositioncontrollerforcopte_k;// '<S4>/NDI position controller for copters reference model' 
  boolean_T CopterRandomExcitation_MODE;// '<S44>/Copter Random Excitation'
  boolean_T AdaptiveINDIG1andG2correction_M;// '<S44>/Adaptive INDI G1 and G2 correction' 
  boolean_T fastdescentsequencer_MODE; // '<S3>/fast descent sequencer'
  boolean_T ThrottleInterceptArcLoadfactorc;// '<S11>/Throttle Intercept Arc (Load factor controller)' 
  boolean_T Subsystem2_MODE;           // '<S11>/Subsystem2'
  boolean_T DurationinInterceptArc_MODE;// '<S11>/Duration in Intercept Arc'
} DW;

// Constant parameters (default storage)
typedef struct {
  // Pooled Parameter (Mixed Expressions)
  //  Referenced by:
  //    '<S75>/Constant1'
  //    '<S75>/TrajMemory'

  dtoSgl_trajectoryStructBus pooled3;

  // Expression: ca
  //  Referenced by: '<S31>/Control Allocation Vertical Acc Weighting'

  struct_fmp4nqBWC4rULgUM9YPHLD ControlAllocationVerticalAccWei;

  // Expression: ca
  //  Referenced by: '<S69>/caIndiWls'

  struct_fmp4nqBWC4rULgUM9YPHLD caIndiWls_ca;

  // Expression: ca
  //  Referenced by: '<S64>/Set Vertical Acc Weight To Zero'

  struct_fmp4nqBWC4rULgUM9YPHLD SetVerticalAccWeightToZero_ca;

  // Expression: G10
  //  Referenced by: '<S68>/MATLAB Function'

  real32_T MATLABFunction_G10[16];

  // Expression: G20
  //  Referenced by: '<S68>/MATLAB Function'

  real32_T MATLABFunction_G20[16];

  // Expression: [psc.rm.veldmax,0,-psc.rm.velumax]
  //  Referenced by: '<S36>/1-D Lookup Table'

  real32_T uDLookupTable_tableData[3];

  // Computed Parameter: uDLookupTable_bp01Data
  //  Referenced by: '<S36>/1-D Lookup Table'

  real32_T uDLookupTable_bp01Data[3];
} ConstP;

// External inputs (root inport signals with default storage)
typedef struct {
  cmdBus cmd;                          // '<Root>/cmd'
  measureBus measure;                  // '<Root>/measure'
} ExtU;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  real32_T u[8];                       // '<Root>/u'
  real32_T logs[58];                   // '<Root>/logs'
} ExtY;

// Constant parameters (default storage)
extern const ConstP rtConstP;

// Exported data declaration

// Const memory section
// Declaration for custom storage class: Const
extern const logConfigBus log_config[5];

// Class declaration for model ArduCopter_Fast_Descent
class MatlabControllerClass {
  // public data and function members
 public:
  // External inputs
  ExtU rtU;

  // External outputs
  ExtY rtY;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // Constructor
  MatlabControllerClass();

  // Destructor
  ~MatlabControllerClass();

  // private data and function members
 private:
  // Block signals and states
  DW rtDW;

  // private member function(s) for subsystem '<Root>'
  void trajSectionGetPos(const real32_T traj_section_pos_x[6], const real32_T
    traj_section_pos_y[6], const real32_T traj_section_pos_z[6], real_T
    varargin_1, real32_T pos[3]);
  void polyder_p(const real32_T u[6], real32_T a_data[], int32_T a_size[2]);
  real32_T polyVal(const real32_T p_data[], const int32_T p_size[2], real_T x);
  void trajSetArcLength(dtoSgl_trajectoryStructBus *traj);
  void polyder(const real_T u_data[], const int32_T u_size[2], real_T a_data[],
               int32_T a_size[2]);
  void polyInterpolationAx_f(real32_T num_of_splines, boolean_T cycle, const
    real32_T x_data[], const int32_T *x_size, real32_T b_data[], int32_T *b_size);
  real32_T norm(const real32_T x_data[], const int32_T *x_size);
  void polyInterpolationAx(real32_T num_of_splines, boolean_T cycle, const
    real32_T x_data[], const int32_T *x_size, real32_T b_data[], int32_T *b_size);
  real32_T norm_c(const real32_T x[2]);
  void ladacLsqrIterate(real32_T A_tunableEnvironment_f1, boolean_T
                        A_tunableEnvironment_f3, real32_T x_data[], int32_T
                        *x_size, real32_T w_data[], int32_T *w_size, real32_T
                        u_data[], int32_T *u_size, real32_T v_data[], int32_T
                        *v_size, real32_T *Anorm, real32_T *alfa, real32_T
                        *rhobar, real32_T *phibar);
  void polyInterpolationb(const real32_T points_data[], const int32_T
    points_size[2], boolean_T cycle, real32_T b_data[], int32_T *b_size,
    real32_T *num_of_splines);
  void ladacLsqrInit(real32_T A_tunableEnvironment_f1, boolean_T
                     A_tunableEnvironment_f3, const real32_T b_data[], const
                     int32_T *b_size, real32_T x_data[], int32_T *x_size,
                     real32_T w_data[], int32_T *w_size, real32_T u_data[],
                     int32_T *u_size, real32_T v_data[], int32_T *v_size,
                     real32_T *alfa, real32_T *rhobar, real32_T *phibar);
  boolean_T trajValidateWaypoints(uint16_T num_wp);
  real32_T norm_e(const real32_T x[3]);
  void trajSectionGetPos_i(const real32_T traj_section_pos_x[6], const real32_T
    traj_section_pos_y[6], const real32_T traj_section_pos_z[6], real32_T
    varargin_1, real32_T pos[3]);
  void polyder_mj(const real32_T u_data[], const int32_T u_size[2], real32_T
                  a_data[], int32_T a_size[2]);
  void trajGetMatchEnhanced(const dtoSgl_trajectoryStructBus *traj, const
    real32_T position[3], real32_T *section_idx, real32_T *error, real32_T *t);
  void trajSectionGetFrenetSerretWithG(const real32_T traj_section_pos_x[6],
    const real32_T traj_section_pos_y[6], const real32_T traj_section_pos_z[6],
    real32_T vel, real32_T g, real32_T varargin_1, real32_T T[3], real32_T B_6[3],
    real32_T N[3], real32_T *kappa, real32_T *tau);
  real32_T integralSimpson(const b_cell_wrap_1 func_tunableEnvironment[3],
    real_T B_5);
  void trajSectionGetArcLength(const real32_T traj_section_pos_x[6], const
    real32_T traj_section_pos_y[6], const real32_T traj_section_pos_z[6],
    real32_T varargin_1, real32_T *arc_length, real32_T *arc_length_dt);
  void trajSectionGetArcLength_j(const real32_T traj_section_pos_x[6], const
    real32_T traj_section_pos_y[6], const real32_T traj_section_pos_z[6], real_T
    varargin_1, real32_T *arc_length, real32_T *arc_length_dt);
  void trajSectionGetFrenetSerretWit_i(const real32_T traj_section_pos_x[6],
    const real32_T traj_section_pos_y[6], const real32_T traj_section_pos_z[6],
    real32_T vel, real_T varargin_1, real32_T T[3], real32_T B_7[3], real32_T N
    [3], real32_T *kappa, real32_T *tau);
  void leanVectorNormDeriv2_c(const real32_T nn[3], const real32_T nn_dt[3],
    const real32_T nn_dt2[3], real_T n_dt2[3]);
  void LSQFromQR(const real32_T A_data[], const int32_T A_size[2], const
                 real32_T tau_data[], const int32_T jpvt_data[], real32_T B_3[8],
                 int32_T rankA, real32_T Y_data[], int32_T *Y_size);
  real32_T xnrm2(int32_T n, const real32_T x_data[], int32_T ix0);
  void xzlarf(int32_T m, int32_T n, int32_T iv0, real32_T tau, real32_T C_data[],
              int32_T ic0, real32_T work_data[]);
  void qrsolve(const real32_T A_data[], const int32_T A_size[2], const real32_T
               B_1[8], real32_T Y_data[], int32_T *Y_size);
  void mldivide(const real32_T A_data[], const int32_T A_size[2], const real32_T
                B_0[8], real32_T Y_data[], int32_T *Y_size);
  boolean_T any(const boolean_T x_data[], const int32_T *x_size);
  real32_T wls_alloc(const real32_T B_4[16], const real32_T v[4], const real32_T
                     umin[4], const real32_T umax[4], const real32_T Wv[16],
                     const real32_T Wu[16], const real32_T ud[4], real32_T gam,
                     real32_T u[4], real32_T W[4], real32_T imax);
  real32_T mean(const real32_T x[4]);
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Constant2' : Unused code path elimination
//  Block '<Root>/Constant3' : Unused code path elimination
//  Block '<S11>/Scope' : Unused code path elimination
//  Block '<S11>/Scope2' : Unused code path elimination
//  Block '<S21>/Scope' : Unused code path elimination
//  Block '<S21>/Scope1' : Unused code path elimination
//  Block '<S11>/altitude' : Unused code path elimination
//  Block '<S11>/status' : Unused code path elimination
//  Block '<S58>/Add' : Unused code path elimination
//  Block '<S58>/Add1' : Unused code path elimination
//  Block '<S58>/Add2' : Unused code path elimination
//  Block '<S58>/Matrix Multiply' : Unused code path elimination
//  Block '<S58>/Matrix Multiply1' : Unused code path elimination
//  Block '<S58>/Scope' : Unused code path elimination
//  Block '<S58>/Scope1' : Unused code path elimination
//  Block '<S58>/Scope2' : Unused code path elimination
//  Block '<S58>/Unit Delay' : Unused code path elimination
//  Block '<S60>/Constant' : Unused code path elimination
//  Block '<S60>/Scope' : Unused code path elimination
//  Block '<S4>/Constant' : Unused code path elimination
//  Block '<S31>/Gain' : Unused code path elimination
//  Block '<S76>/Add' : Unused code path elimination
//  Block '<S37>/Gain' : Unused code path elimination
//  Block '<S37>/Gain1' : Unused code path elimination
//  Block '<S135>/Data Type Duplicate' : Unused code path elimination
//  Block '<S135>/Data Type Propagation' : Unused code path elimination
//  Block '<S136>/Data Type Duplicate' : Unused code path elimination
//  Block '<S136>/Data Type Propagation' : Unused code path elimination
//  Block '<Root>/Data Type Conversion1' : Eliminate redundant data type conversion
//  Block '<S27>/Saturation' : Eliminated Saturate block
//  Block '<Root>/Gain7' : Eliminated nontunable gain of 1
//  Block '<S45>/Reshape3' : Reshape block reduction
//  Block '<S57>/Saturation' : Eliminated Saturate block
//  Block '<S61>/Saturation' : Eliminated Saturate block
//  Block '<S62>/Saturation' : Eliminated Saturate block
//  Block '<S73>/Saturation' : Eliminated Saturate block
//  Block '<S67>/Saturation' : Eliminated Saturate block
//  Block '<S89>/Saturation' : Eliminated Saturate block
//  Block '<S94>/Saturation' : Eliminated Saturate block
//  Block '<S95>/Saturation' : Eliminated Saturate block
//  Block '<S96>/Saturation' : Eliminated Saturate block
//  Block '<S97>/Saturation' : Eliminated Saturate block
//  Block '<S98>/Saturation' : Eliminated Saturate block
//  Block '<S35>/Reshape' : Reshape block reduction
//  Block '<S35>/Reshape1' : Reshape block reduction
//  Block '<S35>/Reshape2' : Reshape block reduction
//  Block '<S108>/Gain1' : Eliminated nontunable gain of 1
//  Block '<S108>/Gain2' : Eliminated nontunable gain of 1
//  Block '<S126>/Saturation' : Eliminated Saturate block
//  Block '<S121>/Saturation' : Eliminated Saturate block
//  Block '<S122>/Saturation' : Eliminated Saturate block
//  Block '<S108>/Reshape' : Reshape block reduction
//  Block '<S134>/Saturation' : Eliminated Saturate block
//  Block '<S31>/Constant' : Unused code path elimination


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'ArduCopter_Fast_Descent'
//  '<S1>'   : 'ArduCopter_Fast_Descent/Actuator muxer'
//  '<S2>'   : 'ArduCopter_Fast_Descent/Compare To Constant'
//  '<S3>'   : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1'
//  '<S4>'   : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1'
//  '<S5>'   : 'ArduCopter_Fast_Descent/MATLAB Function'
//  '<S6>'   : 'ArduCopter_Fast_Descent/MATLAB Function1'
//  '<S7>'   : 'ArduCopter_Fast_Descent/Quaternions to Rotation Matrix'
//  '<S8>'   : 'ArduCopter_Fast_Descent/log muxer'
//  '<S9>'   : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/bypass flightmode'
//  '<S10>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/bypass stick commands'
//  '<S11>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer'
//  '<S12>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Convert to rpyt'
//  '<S13>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Desired Lean Descent'
//  '<S14>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Desired Lean Intercept Arc'
//  '<S15>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Desired Lean finished'
//  '<S16>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Desired Lean slowing'
//  '<S17>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Duration in Intercept Arc'
//  '<S18>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Mission Status'
//  '<S19>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Subsystem2'
//  '<S20>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Throttle Descent'
//  '<S21>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Throttle Intercept Arc (Load factor controller)'
//  '<S22>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Throttle finished'
//  '<S23>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Throttle slowing'
//  '<S24>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Desired Lean Intercept Arc/MATLAB Function6'
//  '<S25>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Subsystem2/MATLAB Function'
//  '<S26>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Throttle Intercept Arc (Load factor controller)/MATLAB Function'
//  '<S27>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Throttle Intercept Arc (Load factor controller)/PT1 discrete with saturations'
//  '<S28>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Throttle Intercept Arc (Load factor controller)/measured specific thrust'
//  '<S29>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Accelerations to Reduced Attitude and Thrust'
//  '<S30>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Autotune'
//  '<S31>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation'
//  '<S32>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Flight path guidance with position controller reference'
//  '<S33>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Incremental specific thrust'
//  '<S34>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Measurement Filtering'
//  '<S35>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller'
//  '<S36>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/NDI position controller for copters reference model'
//  '<S37>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/NDI position controller for copters with reference input'
//  '<S38>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Pilot Reduced Attitude Commands'
//  '<S39>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/State Logic Bus'
//  '<S40>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Stick Command Bus'
//  '<S41>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Vertical Acc to Specific Thrust'
//  '<S42>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Accelerations to Reduced Attitude and Thrust/INDI Copter Acc 2 Lean Vector'
//  '<S43>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Accelerations to Reduced Attitude and Thrust/MATLAB Function4'
//  '<S44>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Autotune/Copter INDI Autotune'
//  '<S45>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/Adaptive INDI G1 and G2 correction'
//  '<S46>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/Copter Random Excitation'
//  '<S47>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/INDI Inversion Check'
//  '<S48>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/accumulate G1 correction factors'
//  '<S49>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/accumulate G2 correction factor'
//  '<S50>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/create diag'
//  '<S51>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/Adaptive INDI G1 and G2 correction/G1 learn rate'
//  '<S52>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/Adaptive INDI G1 and G2 correction/G2 learn rate'
//  '<S53>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/Adaptive INDI G1 and G2 correction/Gradient of Delta_nu w.r.t. G1 correction factors'
//  '<S54>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/Adaptive INDI G1 and G2 correction/Gradient of Delta_nu w.r.t. G2 correction factor'
//  '<S55>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/Adaptive INDI G1 and G2 correction/Inversion error'
//  '<S56>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/Copter Random Excitation/Band-Limited White Noise'
//  '<S57>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/Copter Random Excitation/PT2 discrete with saturation'
//  '<S58>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/INDI Inversion Check/Inversion forward'
//  '<S59>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/INDI Inversion Check/filtered Delta nu'
//  '<S60>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/INDI Inversion Check/filtered Delta u'
//  '<S61>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/INDI Inversion Check/filtered Delta nu/PT2 discrete with saturation1'
//  '<S62>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/INDI Inversion Check/filtered Delta u/PT2 discrete with saturation'
//  '<S63>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation/Control Allocation Vertical Acc Weighting'
//  '<S64>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation/INDI high level wls control allocation'
//  '<S65>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation/Is Manual Throttle Repmat'
//  '<S66>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation/Motor dynamics model'
//  '<S67>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation/Sensor filter model'
//  '<S68>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation/control effectiveness'
//  '<S69>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation/INDI high level wls control allocation/INDI control allocation'
//  '<S70>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation/INDI high level wls control allocation/Set Desired Motor Command'
//  '<S71>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation/INDI high level wls control allocation/Set Vertical Acc Weight To Zero'
//  '<S72>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation/INDI high level wls control allocation/INDI control allocation/caIndiWls'
//  '<S73>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation/Motor dynamics model/PT1 discrete with saturations'
//  '<S74>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation/control effectiveness/MATLAB Function'
//  '<S75>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Flight path guidance with position controller reference/Trajectory from Waypoints'
//  '<S76>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Flight path guidance with position controller reference/flight-path velocity'
//  '<S77>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Flight path guidance with position controller reference/look ahead'
//  '<S78>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Flight path guidance with position controller reference/matching'
//  '<S79>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Flight path guidance with position controller reference/pick if traj is valid'
//  '<S80>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Flight path guidance with position controller reference/reference'
//  '<S81>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Flight path guidance with position controller reference/Trajectory from Waypoints/Detect Rise Positive'
//  '<S82>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Flight path guidance with position controller reference/Trajectory from Waypoints/trajFromWaypoints'
//  '<S83>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Flight path guidance with position controller reference/Trajectory from Waypoints/Detect Rise Positive/Positive'
//  '<S84>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Flight path guidance with position controller reference/flight-path velocity/absolute flight-path velocity'
//  '<S85>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Flight path guidance with position controller reference/look ahead/flight path look ahead'
//  '<S86>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Flight path guidance with position controller reference/matching/flight path matching'
//  '<S87>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Flight path guidance with position controller reference/reference/position controller reference from flight path'
//  '<S88>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Incremental specific thrust/DCM 2 Lean Vector'
//  '<S89>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Incremental specific thrust/PT2 Lean Vector'
//  '<S90>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Incremental specific thrust/desired and measured specific thrust'
//  '<S91>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Incremental specific thrust/incremental thrust atti correction'
//  '<S92>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Incremental specific thrust/PT2 Lean Vector/n ref norm'
//  '<S93>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Measurement Filtering/MATLAB Function'
//  '<S94>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Measurement Filtering/PT2 discrete with saturation'
//  '<S95>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Measurement Filtering/PT2 discrete with saturation1'
//  '<S96>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Measurement Filtering/PT2 discrete with saturation2'
//  '<S97>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Measurement Filtering/PT2 discrete with saturation3'
//  '<S98>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Measurement Filtering/PT2 discrete with saturation4'
//  '<S99>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Measurement Filtering/measured yaw'
//  '<S100>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Measurement Filtering/measured yaw/DCM to quaternions'
//  '<S101>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Measurement Filtering/measured yaw/Quaternion Reduced'
//  '<S102>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/Reduced Attitude Scheduler'
//  '<S103>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/error computation'
//  '<S104>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/measure'
//  '<S105>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/ny control'
//  '<S106>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/ny from reference'
//  '<S107>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/ny measured'
//  '<S108>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model'
//  '<S109>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/Reduced Attitude Scheduler/Reduced Attitude Weighting Factors'
//  '<S110>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/error computation/angle error'
//  '<S111>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/error computation/wrap angle'
//  '<S112>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/error computation/wrap angle1'
//  '<S113>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/measure/DCM to quaternions'
//  '<S114>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/measure/Quaternion Reduced'
//  '<S115>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/Desired Roll Pitch'
//  '<S116>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/Lean Vector Derivative Trafo'
//  '<S117>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/Lean Vector Derivative Trafo Delay'
//  '<S118>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/Lean Vector Derivative to Omega'
//  '<S119>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/MATLAB Function'
//  '<S120>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/PT1 discrete reference model2'
//  '<S121>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/PT2 Lean Vector'
//  '<S122>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/PT2 Lean Vector1'
//  '<S123>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/Pseudo-Control Roll Pitch'
//  '<S124>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/Simulink Trickster'
//  '<S125>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/lean angles 2 lean vector'
//  '<S126>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/PT1 discrete reference model2/PT1 discrete with saturations'
//  '<S127>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/PT2 Lean Vector/n ref norm'
//  '<S128>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/PT2 Lean Vector1/n ref norm'
//  '<S129>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/NDI position controller for copters reference model/MATLAB Function1'
//  '<S130>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/NDI position controller for copters reference model/PT1 discrete reference model'
//  '<S131>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/NDI position controller for copters reference model/PT1 discrete reference model/PT1 discrete with saturations'
//  '<S132>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/NDI position controller for copters with reference input/measures'
//  '<S133>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/NDI position controller for copters with reference input/position controller'
//  '<S134>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/NDI position controller for copters with reference input/position controller/PT2 discrete with saturation'
//  '<S135>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/NDI position controller for copters with reference input/position controller/Saturation Dynamic'
//  '<S136>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/NDI position controller for copters with reference input/position controller/Saturation Dynamic1'
//  '<S137>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/NDI position controller for copters with reference input/position controller/acccntrlmax'
//  '<S138>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/NDI position controller for copters with reference input/position controller/eposmax'
//  '<S139>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Pilot Reduced Attitude Commands/MATLAB Function'
//  '<S140>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Pilot Reduced Attitude Commands/MATLAB Function1'
//  '<S141>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/State Logic Bus/LindiCopter State Logic'
//  '<S142>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Stick Command Bus/throttle_-1_1 to throttle_0_1'
//  '<S143>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot1/Vertical Acc to Specific Thrust/MATLAB Function'
//  '<S144>' : 'ArduCopter_Fast_Descent/log muxer/Auxiliary function to define log_config in generated C++ code'

#endif                                 // RTW_HEADER_MatlabController_h_

//
// File trailer for generated code.
//
// [EOF]
//

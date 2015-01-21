/*
 * HyEst_types.h
 *
 * Code generation for function 'propagate'
 *
 * C source code generated on: Fri Jul 11 14:42:13 2014
 *
 */

#ifndef __HYEST_TYPES_H__
#define __HYEST_TYPES_H__

/* Type Definitions */
#ifndef typedef_airplane_T
#define typedef_airplane_T
typedef struct
{
    real32_T LD_v[3];
    real32_T c_L_alpha[4];
    real32_T K;
    real32_T rho0;
    real32_T ref_mass[2];
    real32_T A;
    real32_T alpha_c_L[5];
    real32_T alpha_min;
    real32_T alpha_max;
    real32_T c_L_min;
    real32_T c_L_max;
    real32_T vmin;
} airplane_T;
#endif /*typedef_airplane_T*/
#ifndef typedef_b_struct_T
#define typedef_b_struct_T
typedef struct
{
    char_T MODEL[8];
    real_T Epoch;
    real_T MAXORD;
    real_T Max2;
    real_T Max3;
    real_T minYear;
    real_T maxYear;
    real_T minAlt;
    real_T maxAlt;
    real_T gh[195];
    real_T max2;
    real_T sv[195];
} b_struct_T;
#endif /*typedef_b_struct_T*/
#ifndef struct_emxArray__common
#define struct_emxArray__common
struct emxArray__common
{
    void *data;
    int32_T *size;
    int32_T allocatedSize;
    int32_T numDimensions;
    boolean_T canFreeData;
};
#endif /*struct_emxArray__common*/
#ifndef typedef_emxArray__common
#define typedef_emxArray__common
typedef struct emxArray__common emxArray__common;
#endif /*typedef_emxArray__common*/
#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T
struct emxArray_int32_T
{
    int32_T *data;
    int32_T *size;
    int32_T allocatedSize;
    int32_T numDimensions;
    boolean_T canFreeData;
};
#endif /*struct_emxArray_int32_T*/
#ifndef typedef_emxArray_int32_T
#define typedef_emxArray_int32_T
typedef struct emxArray_int32_T emxArray_int32_T;
#endif /*typedef_emxArray_int32_T*/
#ifndef struct_emxArray_real32_T
#define struct_emxArray_real32_T
struct emxArray_real32_T
{
    real32_T *data;
    int32_T *size;
    int32_T allocatedSize;
    int32_T numDimensions;
    boolean_T canFreeData;
};
#endif /*struct_emxArray_real32_T*/
#ifndef typedef_emxArray_real32_T
#define typedef_emxArray_real32_T
typedef struct emxArray_real32_T emxArray_real32_T;
#endif /*typedef_emxArray_real32_T*/
#ifndef typedef_states_T
#define typedef_states_T
typedef struct
{
    real_T p[3];
    real32_T q_NS[4];
    real32_T v_N[3];
    real32_T b_g[3];
    real32_T b_a[3];
    real32_T QFF;
    real32_T w[3];
    real32_T K;
} states_T;
#endif /*typedef_states_T*/
#ifndef typedef_struct_T
#define typedef_struct_T
typedef struct
{
    real32_T tau;
    real32_T tau_w;
    real32_T sigma_a_c;
    real32_T sigma_a_d;
    real32_T sigma_aw_c;
    real32_T sigma_g_c;
    real32_T sigma_gw_c;
    real32_T sigma_pw_c;
    real32_T sigma_w_c;
    real32_T sigma_psmd;
    real32_T sigma_pdmd;
    real32_T sigma_Td;
    real32_T sigma_md;
} struct_T;
#endif /*typedef_struct_T*/

#endif
/* End of code generation (HyEst_types.h) */

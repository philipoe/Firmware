/*	Explicit Fixed-Wing Model Predictive Control Law exported Code
 * 	
 * 	controller computation function: 	fw_mpc_roll_v1
 * 	controller parameters: 				RollMPC_Tables.h
 * 	Inputs: 							X (state vector)
 * 										U (control sequence - dims = prediction horizon)
 * 	Outputs:							by referencing-> U
 * 										region -> region of the explicit controller
 * 
 * 	Authors: Kostas Alexis (konstantinos.alexis@mavt.ethz.ch)
 * */

#include <float.h>
#include "tables/PitchMPC_Tables_FastModel_Q36_0o2_R0p32_N6.h"

/* main evaluation function using sequential search */
static unsigned long fw_mpc_pitch_v1( double *X, double *U)

{
    int ix, jx, ic, nc, isinside;
    unsigned long ireg, abspos, iregmin, region;
    double hx, sx, obj, objmin;

    abspos = 0;
    region = 0;
    iregmin = 0;

   /* initialize values of the tie-break function */
    obj = 0;
    objmin = DBL_MAX;


    for (ireg=0; ireg<MPT_NR_PITCH; ireg++) {

        isinside = 1;
        nc = MPT_NC_PITCH[ireg];
        for (ic=0; ic<nc; ic++) {
            hx = 0;
            for (ix=0; ix<MPT_DOMAIN_PITCH; ix++) {
                hx += MPT_A_PITCH[abspos*MPT_DOMAIN_PITCH+ic*MPT_DOMAIN_PITCH+ix]*X[ix];
            }
            if ((hx - MPT_B_PITCH[abspos+ic]) > MPT_ABSTOL_PITCH) {
                /* constraint is violated, continue with next region */
                isinside = 0;
                break;
            } 
        }
        if (isinside==1) {
            /* state belongs to this region, evaluate the tie-breaking function */
            obj = 0;
            for (ix=0; ix<MPT_DOMAIN_PITCH; ix++) {
                sx = 0;
                for (jx=0; jx<MPT_DOMAIN_PITCH; jx++) {
                    sx += MPT_HTB_PITCH[ireg*MPT_DOMAIN_PITCH*MPT_DOMAIN_PITCH + ix*MPT_DOMAIN_PITCH + jx]*X[jx];
                }
                obj += sx*X[ix];
            }
            for (ix=0; ix<MPT_DOMAIN_PITCH; ix++) {
                obj += MPT_FTB_PITCH[ireg*MPT_DOMAIN_PITCH + ix]*X[ix];
            }
            obj += MPT_GTB_PITCH[ireg];
            if (obj<objmin) {
                objmin = obj;
                region = ireg + 1;
                iregmin = ireg;
            }
        }
        abspos = abspos + MPT_NC_PITCH[ireg];
    }
    for (ix=0; ix<MPT_RANGE_PITCH; ix++) {
        sx = 0;
        for (jx=0; jx<MPT_DOMAIN_PITCH; jx++) {
            sx += MPT_F_PITCH[iregmin*MPT_DOMAIN_PITCH*MPT_RANGE_PITCH + ix*MPT_DOMAIN_PITCH + jx]*X[jx];
        }
        U[ix] = sx + MPT_G_PITCH[iregmin*MPT_RANGE_PITCH + ix];
    }
    return region;
}

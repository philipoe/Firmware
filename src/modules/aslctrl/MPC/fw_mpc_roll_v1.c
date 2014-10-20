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
#include "tables/RollMPC_Tables_FastModel_Q40_0p41_R0p5_N5_sXmax.h"

/* main evaluation function using sequential search */
static unsigned long fw_mpc_roll_v1( double *X, double *U)

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


    for (ireg=0; ireg<MPT_NR; ireg++) {

        isinside = 1;
        nc = MPT_NC[ireg];
        for (ic=0; ic<nc; ic++) {
            hx = 0;
            for (ix=0; ix<MPT_DOMAIN; ix++) {
                hx += MPT_A[abspos*MPT_DOMAIN+ic*MPT_DOMAIN+ix]*X[ix];
            }
            if ((hx - MPT_B[abspos+ic]) > MPT_ABSTOL) {
                /* constraint is violated, continue with next region */
                isinside = 0;
                break;
            } 
        }
        if (isinside==1) {
            /* state belongs to this region, evaluate the tie-breaking function */
            obj = 0;
            for (ix=0; ix<MPT_DOMAIN; ix++) {
                sx = 0;
                for (jx=0; jx<MPT_DOMAIN; jx++) {
                    sx += MPT_HTB[ireg*MPT_DOMAIN*MPT_DOMAIN + ix*MPT_DOMAIN + jx]*X[jx];
                }
                obj += sx*X[ix];
            }
            for (ix=0; ix<MPT_DOMAIN; ix++) {
                obj += MPT_FTB[ireg*MPT_DOMAIN + ix]*X[ix];
            }
            obj += MPT_GTB[ireg];
            if (obj<objmin) {
                objmin = obj;
                region = ireg + 1;
                iregmin = ireg;
            }
        }
        abspos = abspos + MPT_NC[ireg];
    }
    for (ix=0; ix<MPT_RANGE; ix++) {
        sx = 0;
        for (jx=0; jx<MPT_DOMAIN; jx++) {
            sx += MPT_F[iregmin*MPT_DOMAIN*MPT_RANGE + ix*MPT_DOMAIN + jx]*X[jx];
        }
        U[ix] = sx + MPT_G[iregmin*MPT_RANGE + ix];
    }
    return region;
}

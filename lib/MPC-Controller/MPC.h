//
// Created by lucaz on 11/3/2026.
//

#ifndef MPC_H
#define MPC_H

#include <stdint.h>


void Cd_MPC(double rho_0, double Vel, double Pos, double tgt_apogee, double t,
            double *Cd, double *ApogeeH_est, double *t_apogee_est);


double fabs(double x) {
//    return (x < 0) ? -x : x; // branch-based impl
    union {
        double d;
        uint64_t u;
    } u = { .d = x };
    u.u &= 0x7FFFFFFFFFFFFFFFULL;   // mask out sign bit
    return u.d;
}





#endif //MPC_H

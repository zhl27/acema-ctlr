//
// Created by lucaz on 11/3/2026.
//

#include "MPC.h"

// TODO: Revisar e integrarlo al resto del sistema

/**
 * Model Predictive Controller for Cd calculation (continuous update).
 *
 * Inputs:
 *   rho_0        - air density at launch altitude [kg/m^3]
 *   Vel          - current vertical velocity [m/s] (positive upward)
 *   Pos          - current altitude [m]
 *   tgt_apogee   - desired apogee altitude [m]
 *   t            - elapsed time since launch [s]
 *
 * Outputs (passed as pointers):
 *   Cd            - selected drag coefficient
 *   ApogeeH_est   - predicted apogee altitude using the best Cd
 *   t_apogee_est  - predicted time from current moment to apogee
 */
void Cd_MPC(double rho_0, double Vel, double Pos, double tgt_apogee, double t,
            double *Cd, double *ApogeeH_est, double *t_apogee_est)
{
    // --- Persistent flag (retains value between calls) ---
    static bool descending = false;   // becomes true after apogee is passed

    // --- Hardcoded parameters ---
    const double A = 0.00283;        // [m^2] front cross‑section
    const double Cd_clean = 0.533;   // baseline drag (no airbrakes)
    const double m_0 = 1.6;          // [kg] launch mass (constant after burnout)
    const double deltaT = 0.01;      // [s] integration step for prediction
    const int horizon = 2000;         // number of integration steps (20 s look‑ahead)
    const double t_prop = 1.2;        // [s] powered flight duration
    const double Cd_max = 1.5;        // maximum achievable drag
    const double g = 9.81;            // [m/s^2] gravity acceleration

    // --- Current state ---
    double w = Vel;       // vertical velocity
    double z = Pos;       // altitude
    double m = m_0;       // mass (constant)

    // --- Initialize outputs to safe defaults ---
    double best_error = 1e9;   // "inf" approximation
    double Cd_best = Cd_clean;
    *ApogeeH_est = z;
    *t_apogee_est = 0.0;

    // --- If still under power, no control applied ---
    if (t < t_prop) {
        *Cd = Cd_clean;
        return;
    }

    // --- Detect if apogee has just been passed ---
    if (!descending && (Vel <= 0.0)) {
        descending = true;
    }

    // --- Brute‑force search over candidate Cd values ---
    // Iterate from Cd_clean to Cd_max with step 0.01
    int num_steps = (int)((Cd_max - Cd_clean) / 0.01 + 0.5); // +0.5 for rounding
    for (int idx = 0; idx <= num_steps; ++idx) {
        double Cd_sim = Cd_clean + idx * 0.01;   // ensure last value is <= Cd_max
        if (Cd_sim > Cd_max) Cd_sim = Cd_max;

        // Simulation state
        double w_sim = w;
        double alt_sim = z;
        double t_sim = 0.0;
        double rho = rho_0;   // initial density

        // Predict future trajectory
        for (int i = 0; i < horizon; ++i) {
            // Drag force (opposes motion)
            double sign_w = (w_sim > 0.0) - (w_sim < 0.0);   // signum function
            double F_drag = -0.5 * Cd_sim * rho * A * w_sim * w_sim * sign_w;

            // Update density using linear model (as in original code)
            // rho = rho_0 - ((1.225-1.173)/450) * alt_sim
            // Note: This model is flawed but kept for exact translation.
            rho = rho_0 - ((1.225 - 1.173) / 450.0) * alt_sim;

            // Acceleration (vertical)
            double w_dot_sim = F_drag / m - g;

            // Euler integration
            w_sim = w_sim + w_dot_sim * deltaT;
            alt_sim = alt_sim + w_sim * deltaT;
            t_sim = t_sim + deltaT;

            // Stop if downward velocity exceeds 0.5 m/s (apogee already passed)
            if (w_sim < -0.5) {
                break;
            }
        }

        // Evaluate this candidate Cd
        double apogeeAlt = alt_sim;   // final altitude used as predicted apogee
        double error = fabs(apogeeAlt - tgt_apogee);

        if (error < best_error) {
            best_error = error;
            Cd_best = Cd_sim;
            *ApogeeH_est = apogeeAlt;
            *t_apogee_est = t_sim;
        }
    }

    // --- Select the best Cd found ---
    *Cd = Cd_best;

    // --- After apogee, force airbrakes retracted (Cd = 0.4) ---
    if (descending) {
        *Cd = 0.4;
    }
}
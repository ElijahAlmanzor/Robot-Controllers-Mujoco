#pragma once

#include <Eigen/Dense>
#include <cassert>

/*
SELF NOTES
What this PID owns:
- persistent error state (integral, previous error)
- scalar gains (same for all joints, intentional for now)
- vector size consistency

Design assumptions:
- caller passes correct dt every step
- this class does not know about MuJoCo or robot timing
*/

/*
IMPORTANT USAGE NOTE
P_term, I_term, and D_term are stateful.
They must be called exactly once per control step
and in a consistent order (P -> I -> D).
*/


class PID
{
public:
    // initialise vector sizes and reset state
    void init(int size);

    // individual PID terms (stateful for I and D)
    Eigen::VectorXd P_term(const Eigen::VectorXd& error);
    Eigen::VectorXd I_term(const Eigen::VectorXd& error, double dt);
    Eigen::VectorXd D_term(const Eigen::VectorXd& error, double dt);
    // Convenience: compute and sum P, I, D once per control step
    Eigen::VectorXd all_terms(const Eigen::VectorXd& error, double dt);


    // optional helpers
    void reset();
    void set_gains(double kp, double ki, double kd);
    void set_integral_limit(double limit);

private:
    // PID configuration
    int error_size = 0;
    double P_gain = 0.0;
    double I_gain = 0.0;
    double D_gain = 0.0;

    // anti windup bound (symmetric)
    double integral_limit = 0.0; // 0 means no clamping

    // error state
    Eigen::VectorXd error_integral;
    Eigen::VectorXd error_derivative;
    Eigen::VectorXd prev_error;
};

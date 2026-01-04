#include "PID.hpp"

void PID::init(int size)
{
    assert(size > 0);

    error_size = size;

    error_integral.resize(error_size);
    error_derivative.resize(error_size);
    prev_error.resize(error_size);

    error_integral.setZero();
    error_derivative.setZero();
    prev_error.setZero();
}

void PID::reset()
{
    error_integral.setZero();
    error_derivative.setZero();
    prev_error.setZero();
}

void PID::set_gains(double kp, double ki, double kd)
{
    P_gain = kp;
    I_gain = ki;
    D_gain = kd;
}

void PID::set_integral_limit(double limit)
{
    integral_limit = std::abs(limit);
}

// ---------------- PID terms ----------------

Eigen::VectorXd PID::P_term(const Eigen::VectorXd& error)
{
    assert(error.size() == error_size);
    return P_gain * error;
}

Eigen::VectorXd PID::I_term(const Eigen::VectorXd& error, double dt)
{
    assert(error.size() == error_size);
    assert(dt > 0.0);

    // integrate error
    error_integral += error * dt;

    // simple anti windup
    if (integral_limit > 0.0)
    {
        error_integral =
            error_integral.cwiseMax(-integral_limit)
                          .cwiseMin( integral_limit);
    }

    return I_gain * error_integral;
}

Eigen::VectorXd PID::D_term(const Eigen::VectorXd& error, double dt)
{
    assert(error.size() == error_size);
    assert(dt > 0.0);

    error_derivative = (error - prev_error) / dt;
    prev_error = error;

    return D_gain * error_derivative;
}

Eigen::VectorXd PID::all_terms(const Eigen::VectorXd& error, double dt)
{
    // basic sanity checks
    assert(error.size() == error_size);
    assert(dt > 0.0);

    // compute each term exactly once
    Eigen::VectorXd p = P_term(error);
    Eigen::VectorXd i = I_term(error, dt);
    Eigen::VectorXd d = D_term(error, dt);

    // return full PID output
    return p + i + d;
}

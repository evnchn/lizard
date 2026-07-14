#pragma once

#include "foc_motor.h"
#include "module.h"

class FocArm;
using FocArm_ptr = std::shared_ptr<FocArm>;

/* 1-DOF Cartesian arm, 507-Mechanical-Movements style: a long arm on the
 * FocMotor pivot sweeps across a horizontal line h cm below the pivot.
 * The end effector is the arm/line intersection: x = h*tan(theta).
 * goto(x) plans a trapezoidal profile in Cartesian space; every step runs
 * the IK (theta = atan(x/h)) and the chain-rule velocity feedforward
 * (theta_dot = x_dot * cos^2(theta) / h). Swapping in 2-DOF IK later only
 * replaces these two formulas; trajectory machinery stays identical. */
class FocArm : public Module {
public:
    static inline constexpr const char *TYPE = "FocArm";

    FocArm(const std::string name, const FocMotor_ptr motor,
           const double height_cm, const double v_max, const double a_max, const double gear);
    void step() override;
    void call(const std::string method_name, const std::vector<ConstExpression_ptr> arguments) override;
    static const std::map<std::string, Variable_ptr> get_defaults();

private:
    const FocMotor_ptr motor;
    const double height_cm;
    const double v_max; // cm/s
    const double a_max; // cm/s^2
    const double gear;  // motor rad per arm rad (gearbox fiction or real)

    bool moving = false;
    bool joint_mode = false; // true: trapezoid planned in joint space (the "RMD way", for contrast)
    int64_t t0_us = 0;
    double x0 = 0, dist = 0, dir = 1, t_acc = 0, t_cruise = 0, v_peak = 0, profile_a = 0;

    double forward_x() const;
    void plan(const double x_target, const bool in_joint_space);
    void evaluate(const double t, double &x, double &v) const;
};

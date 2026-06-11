#pragma once

#include "foc_drive.h"
#include "module.h"
#include "motor.h"

class FocMotor;
using FocMotor_ptr = std::shared_ptr<FocMotor>;

class FocMotor : public Module, virtual public Motor {
public:
    static inline constexpr const char *TYPE = "FocMotor";

    FocMotor(const std::string name,
             const int pwm_a, const int pwm_b, const int pwm_c, const int enable_pin,
             const int sda, const int scl, const int pole_pairs,
             const float supply_voltage, const float voltage_limit);
    void step() override;
    void call(const std::string method_name, const std::vector<ConstExpression_ptr> arguments) override;
    static const std::map<std::string, Variable_ptr> get_defaults();

    /* C++-level target interface for trajectory modules (FocArm) */
    void set_joint_target(const float angle, const float velocity_ff);
    float get_joint_angle() const;
    float get_joint_velocity() const;

    void stop() override;
    double get_position() override;
    void position(const double position, const double speed, const double acceleration) override;
    double get_speed() override;
    void speed(const double speed, const double acceleration) override;
    void enable() override;
    void disable() override;

private:
    FocDrive drive;
};

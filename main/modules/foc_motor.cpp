#include "foc_motor.h"
#include <algorithm>

static Module_ptr create_foc_motor(const std::string &name, const std::vector<ConstExpression_ptr> &arguments, MessageHandler) {
    if (arguments.size() == 7) {
        Module::expect(arguments, 7, integer, integer, integer, integer, integer, integer, integer);
    } else {
        Module::expect(arguments, 9, integer, integer, integer, integer, integer, integer, integer, numbery, numbery);
    }
    return std::make_shared<FocMotor>(name,
                                      arguments[0]->evaluate_integer(),
                                      arguments[1]->evaluate_integer(),
                                      arguments[2]->evaluate_integer(),
                                      arguments[3]->evaluate_integer(),
                                      arguments[4]->evaluate_integer(),
                                      arguments[5]->evaluate_integer(),
                                      arguments[6]->evaluate_integer(),
                                      arguments.size() > 7 ? arguments[7]->evaluate_number() : 24.0f,
                                      arguments.size() > 8 ? arguments[8]->evaluate_number() : 6.0f);
}
REGISTER_MODULE(FocMotor, &create_foc_motor)

const std::map<std::string, Variable_ptr> FocMotor::get_defaults() {
    return {
        {"position", std::make_shared<NumberVariable>()},
        {"velocity", std::make_shared<NumberVariable>()},
        {"enabled", std::make_shared<BooleanVariable>(false)},
        {"loop_rate", std::make_shared<IntegerVariable>()},
        {"sensor_errors", std::make_shared<IntegerVariable>()},
    };
}

FocMotor::FocMotor(const std::string name,
                   const int pwm_a, const int pwm_b, const int pwm_c, const int enable_pin,
                   const int sda, const int scl, const int pole_pairs,
                   const float supply_voltage, const float voltage_limit)
    : Module(name),
      drive(pwm_a, pwm_b, pwm_c, enable_pin, sda, scl, pole_pairs, supply_voltage, voltage_limit) {
    this->properties = FocMotor::get_defaults();
    this->drive.start();
}

void FocMotor::step() {
    this->properties.at("position")->number_value = this->drive.get_angle();
    this->properties.at("velocity")->number_value = this->drive.get_velocity();
    this->properties.at("enabled")->boolean_value = this->drive.is_enabled();
    this->properties.at("loop_rate")->integer_value = this->drive.get_loop_rate();
    this->properties.at("sensor_errors")->integer_value = this->drive.get_sensor_errors();
    Module::step();
}

void FocMotor::call(const std::string method_name, const std::vector<ConstExpression_ptr> arguments) {
    if (method_name == "enable") {
        Module::expect(arguments, 0);
        this->enable();
    } else if (method_name == "disable") {
        Module::expect(arguments, 0);
        this->disable();
    } else if (method_name == "target") {
        Module::expect(arguments, 1, numbery);
        this->set_joint_target(arguments[0]->evaluate_number(), 0.0f);
    } else {
        Module::call(method_name, arguments);
    }
}

void FocMotor::set_joint_target(const float angle, const float velocity_ff) {
    this->drive.set_target(angle, velocity_ff);
}

float FocMotor::get_joint_angle() const {
    return this->drive.get_angle();
}

float FocMotor::get_joint_velocity() const {
    return this->drive.get_velocity();
}

void FocMotor::stop() {
    this->drive.set_target(this->drive.get_angle(), 0.0f);
}

double FocMotor::get_position() {
    return this->drive.get_angle();
}

void FocMotor::position(const double position, const double speed, const double acceleration) {
    this->drive.set_target(position, 0.0f);
}

double FocMotor::get_speed() {
    return this->drive.get_velocity();
}

void FocMotor::speed(const double speed, const double acceleration) {
    // velocity passthrough: feedforward at given speed around the live angle
    this->drive.set_target(this->drive.get_angle(), speed);
}

void FocMotor::enable() {
    this->drive.enable();
}

void FocMotor::disable() {
    this->drive.disable();
}

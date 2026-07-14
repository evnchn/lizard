#include "foc_arm.h"
#include "module_helpers.h"
#include "esp_timer.h"
#include <cmath>
#include <stdexcept>

static Module_ptr create_foc_arm(const std::string &name, const std::vector<ConstExpression_ptr> &arguments, MessageHandler) {
    if (arguments.size() == 4) {
        Module::expect(arguments, 4, identifier, numbery, numbery, numbery);
    } else {
        Module::expect(arguments, 5, identifier, numbery, numbery, numbery, numbery);
    }
    FocMotor_ptr motor = get_module_argument<FocMotor>(arguments[0]);
    return std::make_shared<FocArm>(name, motor,
                                    arguments[1]->evaluate_number(),
                                    arguments[2]->evaluate_number(),
                                    arguments[3]->evaluate_number(),
                                    arguments.size() > 4 ? arguments[4]->evaluate_number() : 1.0);
}
REGISTER_MODULE(FocArm, &create_foc_arm)

const std::map<std::string, Variable_ptr> FocArm::get_defaults() {
    return {
        {"x", std::make_shared<NumberVariable>()},
        {"moving", std::make_shared<BooleanVariable>(false)},
    };
}

FocArm::FocArm(const std::string name, const FocMotor_ptr motor,
               const double height_cm, const double v_max, const double a_max, const double gear)
    : Module(name), motor(motor), height_cm(height_cm), v_max(v_max), a_max(a_max), gear(gear) {
    if (height_cm <= 0 || v_max <= 0 || a_max <= 0 || gear == 0) {
        throw std::runtime_error("FocArm: require height > 0, v_max > 0, a_max > 0, gear != 0");
    }
    this->properties = FocArm::get_defaults();
}

double FocArm::forward_x() const {
    return this->height_cm * std::tan(this->motor->get_joint_angle() / this->gear);
}

void FocArm::plan(const double x_target, const bool in_joint_space) {
    const double max_angle = 1.2; // ~69 deg arm swing keeps the intersection finite
    const double limit = this->height_cm * std::tan(max_angle);
    if (x_target < -limit || x_target > limit) {
        throw std::runtime_error("goto target outside workspace");
    }
    this->joint_mode = in_joint_space;
    /* profile space: Cartesian cm for goto, joint rad for goto_joint
     * (joint limits scaled by 1/h: same numbers near theta=0, distortion shows at the edges) */
    double s0, s1, vm, am;
    if (in_joint_space) {
        s0 = this->motor->get_joint_angle() / this->gear;
        s1 = std::atan(x_target / this->height_cm);
        vm = this->v_max / this->height_cm;
        am = this->a_max / this->height_cm;
    } else {
        s0 = this->forward_x();
        s1 = x_target;
        vm = this->v_max;
        am = this->a_max;
    }
    this->x0 = s0;
    const double d = s1 - s0;
    this->dir = d >= 0 ? 1.0 : -1.0;
    this->dist = std::abs(d);
    this->t_acc = vm / am;
    const double d_acc = 0.5 * am * this->t_acc * this->t_acc;
    if (2 * d_acc >= this->dist) { // triangle profile
        this->t_acc = std::sqrt(this->dist / am);
        this->t_cruise = 0;
        this->v_peak = am * this->t_acc;
    } else {
        this->t_cruise = (this->dist - 2 * d_acc) / vm;
        this->v_peak = vm;
    }
    this->profile_a = am;
    this->t0_us = esp_timer_get_time();
    this->moving = true;
}

void FocArm::evaluate(const double t, double &x, double &v) const {
    const double t_total = 2 * this->t_acc + this->t_cruise;
    double p, vel;
    if (t >= t_total) {
        p = this->dist;
        vel = 0;
    } else if (t < this->t_acc) {
        p = 0.5 * this->profile_a * t * t;
        vel = this->profile_a * t;
    } else if (t < this->t_acc + this->t_cruise) {
        p = 0.5 * this->profile_a * this->t_acc * this->t_acc + this->v_peak * (t - this->t_acc);
        vel = this->v_peak;
    } else {
        const double td = t_total - t;
        p = this->dist - 0.5 * this->profile_a * td * td;
        vel = this->profile_a * td;
    }
    x = this->x0 + this->dir * p;
    v = this->dir * vel;
}

void FocArm::step() {
    if (this->moving) {
        const double t = (esp_timer_get_time() - this->t0_us) * 1e-6;
        double s, v;
        this->evaluate(t, s, v);
        double theta, theta_dot;
        if (this->joint_mode) {
            theta = s; // trapezoid ran in joint space; Cartesian profile is whatever falls out
            theta_dot = v;
        } else {
            /* inverse kinematics: x = h*tan(theta) -> theta = atan(x/h)
             * chain rule for joint velocity ff: theta_dot = x_dot * cos^2(theta) / h */
            theta = std::atan(s / this->height_cm);
            const double cos_theta = std::cos(theta);
            theta_dot = v * cos_theta * cos_theta / this->height_cm;
        }
        this->motor->set_joint_target(theta * this->gear, theta_dot * this->gear);
        if (t >= 2 * this->t_acc + this->t_cruise) {
            this->moving = false;
        }
    }
    this->properties.at("x")->number_value = this->forward_x();
    this->properties.at("moving")->boolean_value = this->moving;
    Module::step();
}

void FocArm::call(const std::string method_name, const std::vector<ConstExpression_ptr> arguments) {
    if (method_name == "goto") {
        Module::expect(arguments, 1, numbery);
        this->motor->enable();
        this->plan(arguments[0]->evaluate_number(), false);
    } else if (method_name == "goto_joint") { // "RMD-style" joint-space trapezoid, for contrast
        Module::expect(arguments, 1, numbery);
        this->motor->enable();
        this->plan(arguments[0]->evaluate_number(), true);
    } else if (method_name == "stop") {
        Module::expect(arguments, 0);
        this->moving = false;
        this->motor->stop();
    } else {
        Module::call(method_name, arguments);
    }
}

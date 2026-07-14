#pragma once

#include <cstdint>

/* Include firewall around esp_simplefoc: its Arduino-compat layer defines
 * `typedef bool boolean` and a global `Serial`, both of which collide with
 * Lizard's global `boolean` enumerator (compilation/type.h) and `Serial`
 * module class. This header exposes the FOC drive with plain types only;
 * foc_drive.cpp is the single translation unit that sees esp_simplefoc. */
class FocDrive {
public:
    FocDrive(const int pwm_a, const int pwm_b, const int pwm_c, const int enable_pin,
             const int sda, const int scl, const int pole_pairs,
             const float supply_voltage, const float voltage_limit);
    void start(); // init FOC + spawn the control task on core 1; throws on sensor failure
    void set_target(const float angle, const float velocity_ff);
    void set_target_limited(const float angle, const float velocity_limit);
    void set_velocity(const float velocity);
    float get_angle() const;
    float get_velocity() const;
    uint32_t get_loop_rate() const;
    uint32_t get_sensor_errors() const;
    bool is_enabled() const;
    bool is_fault() const;
    void enable();
    void disable();

    struct Impl; // public so the FreeRTOS task trampoline can use it

private:
    Impl *impl; // intentionally never freed: Lizard modules live for the firmware's lifetime
};

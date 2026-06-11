#include "foc_drive.h"
/* selective esp_simplefoc includes: the umbrella esp_simplefoc.h drags in
 * angle_sensor/as5600.h etc., which require the i2c_bus v2 API that we disable
 * (CONFIG_I2C_BUS_BACKWARD_CONFIG) for IDF 5.3 compatibility. We bring our own sensor. */
#include "esp_platform.h"
#include "BLDCMotor.h"
#include "esp_hal_bldc_3pwm.h"
#include "common/base_classes/Sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "soc/gpio_struct.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

/* Velocity-feedforward tracking, gains validated on DengFOC V3+ with dangling load:
 * feedforward carries the motion, weak P cleans drift. Stiffer outer P limit-cycles. */
static constexpr float KP_TRACK = 8.0f;
static constexpr float VEL_MAX = 6.0f;

/* AS5600 reader using direct GPIO register access (~50 ns/edge).
 * The ESP32 hardware I2C peripheral wedges on some FOC boards (e.g. DengFOC V3+);
 * bit-banging is reliable and fast enough for a multi-kHz FOC loop. */
class BitBangAS5600 : public Sensor {
public:
    BitBangAS5600(const int sda, const int scl);
    void init();
    float getSensorAngle() override;
    uint32_t errors = 0;

private:
    const int sda;
    const int scl;
    uint32_t sda_bit;
    uint32_t scl_bit;
    uint16_t last_raw = 0;
    bool write_byte(uint8_t b);
    uint8_t read_byte(bool ack);
    void start();
    void stop();
};

BitBangAS5600::BitBangAS5600(const int sda, const int scl) : sda(sda), scl(scl) {
}

void BitBangAS5600::init() {
    if (this->sda < 0 || this->sda > 31 || this->scl < 0 || this->scl > 31) {
        // direct-register open-drain below uses GPIO.out/enable/in, which cover GPIO 0-31 only;
        // 1UL << pin would silently wrap for pins >= 32 (Xtensa shifts are mod 32)
        throw std::runtime_error("BitBangAS5600: sda/scl must be GPIO 0-31");
    }
    gpio_reset_pin((gpio_num_t)this->sda);
    gpio_reset_pin((gpio_num_t)this->scl);
    gpio_set_direction((gpio_num_t)this->sda, GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)this->scl, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)this->sda, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)this->scl, GPIO_PULLUP_ONLY);
    this->sda_bit = 1UL << this->sda;
    this->scl_bit = 1UL << this->scl;
    GPIO.out_w1tc = this->sda_bit | this->scl_bit; // out latched low; enable bit = drive low
    this->Sensor::init();
}

#define BB_LOW(bit) (GPIO.enable_w1ts = (bit))
#define BB_REL(bit) (GPIO.enable_w1tc = (bit))
#define BB_RD(bit) ((GPIO.in & (bit)) ? 1 : 0)
#define BB_DLY() ets_delay_us(1)

void BitBangAS5600::start() {
    BB_REL(sda_bit);
    BB_REL(scl_bit);
    BB_DLY();
    BB_LOW(sda_bit);
    BB_DLY();
    BB_LOW(scl_bit);
    BB_DLY();
}

void BitBangAS5600::stop() {
    BB_LOW(sda_bit);
    BB_DLY();
    BB_REL(scl_bit);
    BB_DLY();
    BB_REL(sda_bit);
    BB_DLY();
}

bool BitBangAS5600::write_byte(uint8_t b) {
    for (int i = 7; i >= 0; i--) {
        if (b & (1 << i)) {
            BB_REL(sda_bit);
        } else {
            BB_LOW(sda_bit);
        }
        BB_DLY();
        BB_REL(scl_bit);
        BB_DLY();
        for (int k = 0; k < 200 && !BB_RD(scl_bit); k++) {
            BB_DLY(); // clock stretch
        }
        BB_LOW(scl_bit);
        BB_DLY();
    }
    BB_REL(sda_bit);
    BB_DLY();
    BB_REL(scl_bit);
    BB_DLY();
    bool ack = !BB_RD(sda_bit);
    BB_LOW(scl_bit);
    BB_DLY();
    return ack;
}

uint8_t BitBangAS5600::read_byte(bool ack) {
    uint8_t b = 0;
    BB_REL(sda_bit);
    for (int i = 7; i >= 0; i--) {
        BB_DLY();
        BB_REL(scl_bit);
        BB_DLY();
        for (int k = 0; k < 200 && !BB_RD(scl_bit); k++) {
            BB_DLY();
        }
        if (BB_RD(sda_bit)) {
            b |= (1 << i);
        }
        BB_LOW(scl_bit);
        BB_DLY();
    }
    if (ack) {
        BB_LOW(sda_bit);
    } else {
        BB_REL(sda_bit);
    }
    BB_DLY();
    BB_REL(scl_bit);
    BB_DLY();
    BB_LOW(scl_bit);
    BB_DLY();
    BB_REL(sda_bit);
    return b;
}

float BitBangAS5600::getSensorAngle() {
    this->start();
    if (!this->write_byte(0x36 << 1)) {
        this->errors++;
        this->stop();
        return this->last_raw * (_2PI / 4096.0f);
    }
    if (!this->write_byte(0x0C)) {
        this->errors++;
        this->stop();
        return this->last_raw * (_2PI / 4096.0f);
    }
    this->start();
    if (!this->write_byte((0x36 << 1) | 1)) {
        this->errors++;
        this->stop();
        return this->last_raw * (_2PI / 4096.0f);
    }
    uint8_t msb = this->read_byte(true);
    uint8_t lsb = this->read_byte(false);
    this->stop();
    this->last_raw = ((msb & 0x0F) << 8) | lsb;
    return this->last_raw * (_2PI / 4096.0f);
}


struct FocDrive::Impl {
    BLDCMotor motor;
    BLDCDriver3PWM driver;
    BitBangAS5600 sensor;
    volatile float target_angle = 0.0f;
    volatile float target_velocity_ff = 0.0f;
    volatile bool drive_enabled = false;
    volatile uint32_t loop_rate_hz = 0;
    const float supply_voltage;
    const float voltage_limit;

    Impl(const int pwm_a, const int pwm_b, const int pwm_c, const int enable_pin,
         const int sda, const int scl, const int pole_pairs,
         const float supply_voltage, const float voltage_limit)
        : motor(pole_pairs), driver(pwm_a, pwm_b, pwm_c, enable_pin), sensor(sda, scl),
          supply_voltage(supply_voltage), voltage_limit(voltage_limit) {
    }

    void run() {
        uint32_t count = 0;
        int64_t window_start = esp_timer_get_time();
        while (true) {
            this->motor.loopFOC();
            float vel_cmd = 0.0f;
            if (this->drive_enabled) {
                float error = this->target_angle - this->motor.shaft_angle;
                vel_cmd = this->target_velocity_ff + KP_TRACK * error;
                vel_cmd = std::min(std::max(vel_cmd, -VEL_MAX), VEL_MAX);
            }
            this->motor.move(vel_cmd);
            count++;
            int64_t now = esp_timer_get_time();
            if (now - window_start >= 1000000) {
                this->loop_rate_hz = count;
                count = 0;
                window_start = now;
            }
        }
    }
};

static void foc_task_trampoline(void *arg) {
    static_cast<FocDrive::Impl *>(arg)->run();
}

FocDrive::FocDrive(const int pwm_a, const int pwm_b, const int pwm_c, const int enable_pin,
                   const int sda, const int scl, const int pole_pairs,
                   const float supply_voltage, const float voltage_limit)
    : impl(new Impl(pwm_a, pwm_b, pwm_c, enable_pin, sda, scl, pole_pairs, supply_voltage, voltage_limit)) {
    if (pole_pairs < 1) {
        throw std::runtime_error("FocDrive: pole_pairs must be >= 1");
    }
    if (supply_voltage <= 0 || voltage_limit <= 0 || voltage_limit > supply_voltage) {
        throw std::runtime_error("FocDrive: require 0 < voltage_limit <= supply_voltage");
    }
}

void FocDrive::start() {
    this->impl->sensor.init();
    this->impl->motor.linkSensor(&this->impl->sensor);
    this->impl->driver.voltage_power_supply = this->impl->supply_voltage;
    this->impl->driver.voltage_limit = this->impl->voltage_limit;
    this->impl->driver.init();
    this->impl->motor.linkDriver(&this->impl->driver);
    this->impl->motor.voltage_sensor_align = 1.0f;
    this->impl->motor.voltage_limit = this->impl->voltage_limit;
    this->impl->motor.controller = MotionControlType::velocity;
    this->impl->motor.PID_velocity.P = 0.4f;
    this->impl->motor.PID_velocity.I = 5.0f;
    this->impl->motor.PID_velocity.output_ramp = 100;
    this->impl->motor.LPF_velocity.Tf = 0.05f;
    this->impl->motor.velocity_limit = VEL_MAX + 2;
    this->impl->motor.init();
    if (!this->impl->motor.initFOC()) {
        throw std::runtime_error("FocDrive: initFOC failed (check sensor wiring)");
    }
    this->impl->target_angle = this->impl->motor.shaft_angle;
    this->impl->motor.disable();
    xTaskCreatePinnedToCore(foc_task_trampoline, "foc_drive", 4096, this->impl, 5, nullptr, 1);
}

void FocDrive::set_target(const float angle, const float velocity_ff) {
    this->impl->target_angle = angle;
    this->impl->target_velocity_ff = velocity_ff;
}

float FocDrive::get_angle() const {
    return this->impl->motor.shaft_angle;
}

float FocDrive::get_velocity() const {
    return this->impl->motor.shaft_velocity;
}

uint32_t FocDrive::get_loop_rate() const {
    return this->impl->loop_rate_hz;
}

uint32_t FocDrive::get_sensor_errors() const {
    return this->impl->sensor.errors;
}

bool FocDrive::is_enabled() const {
    return this->impl->drive_enabled;
}

void FocDrive::enable() {
    this->impl->target_angle = this->impl->motor.shaft_angle;
    this->impl->target_velocity_ff = 0.0f;
    this->impl->motor.enable();
    this->impl->drive_enabled = true;
}

void FocDrive::disable() {
    this->impl->drive_enabled = false;
    this->impl->motor.disable();
}

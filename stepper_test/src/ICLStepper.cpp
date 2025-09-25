#include "stepper_test/ICLStepper.h"

#include <iostream>
#include <unistd.h>
#include <cerrno>
#include <modbus/modbus.h>

ICLStepper::ICLStepper(int slave_id, modbus_t* ctx)
    : slave_id_(slave_id),
      ctx_(ctx),
      current_position_(0),
      delay_us_(10) {}

int ICLStepper::initialize() {
    modbus_set_slave(ctx_, slave_id_);
    usleep(delay_us_);

    // Enable motor
    if (modbus_write_register(ctx_, 0x000F, 1) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to enable motor: "
                  << modbus_strerror(errno) << std::endl;
        return -1;
    }
    usleep(delay_us_);

    // // set jog acceleration/deceleration
    // if (modbus_write_register(ctx_, 0x01E7, 10) == -1) {
    //     std::cerr << "[Slave " << slave_id_ << "] Failed to set jog acc/dec: "
    //               << modbus_strerror(errno) << std::endl;
    //     return -1;
    // }
    // usleep(delay_us_);

    // Example: set a target speed (100)
    if (modbus_write_register(ctx_, 0x01E1, 100) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to set target speed: "
                  << modbus_strerror(errno) << std::endl;
        return -1;
    }
    usleep(delay_us_);

    return 0;
}

int ICLStepper::set_position(int position, int velocity_rpm, int acc, int dec) {
    modbus_set_slave(ctx_, slave_id_);
    usleep(delay_us_);

    // 1) PR0 mode: absolute positioning
    if (modbus_write_register(ctx_, 0x6200, 0x0001) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to set PR0 mode: "
                  << modbus_strerror(errno) << std::endl;
        return -1;
    }
    std::cout << "Set PR0 mode to absolute positioning" << std::endl;
    usleep(delay_us_);

    // 2) Position high/low
    uint16_t pos_high = static_cast<uint16_t>((position >> 16) & 0xFFFF);
    uint16_t pos_low  = static_cast<uint16_t>(position & 0xFFFF);

    if (modbus_write_register(ctx_, 0x6201, pos_high) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to set PR0 high position: "
                  << modbus_strerror(errno) << std::endl;
        return -1;
    }
    std::cout << "Set PR0 high position to " << pos_high << std::endl;
    usleep(delay_us_);

    if (modbus_write_register(ctx_, 0x6202, pos_low) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to set PR0 low position: "
                  << modbus_strerror(errno) << std::endl;
        return -1;
    }
    std::cout << "Set PR0 low position to " << pos_low << std::endl;
    usleep(delay_us_);

    // 3) Velocity
    if (modbus_write_register(ctx_, 0x6203, static_cast<uint16_t>(velocity_rpm)) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to set PR0 velocity: "
                  << modbus_strerror(errno) << std::endl;
        return -1;
    }
    std::cout << "Set PR0 velocity to " << velocity_rpm << std::endl;
    usleep(delay_us_);

    // 4) Acc/Dec
    if (modbus_write_register(ctx_, 0x6204, static_cast<uint16_t>(acc)) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to set PR0 acceleration: "
                  << modbus_strerror(errno) << std::endl;
        return -1;
    }
    std::cout << "Set PR0 acceleration to " << acc << std::endl;
    usleep(delay_us_);

    if (modbus_write_register(ctx_, 0x6205, static_cast<uint16_t>(dec)) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to set PR0 deceleration: "
                  << modbus_strerror(errno) << std::endl;
        return -1;
    }
    std::cout << "Set PR0 deceleration to " << dec << std::endl;
    usleep(delay_us_);

    // 5) Trigger PR0 motion
    if (modbus_write_register(ctx_, 0x6002, 0x0010) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to trigger PR0 motion: "
                  << modbus_strerror(errno) << std::endl;
        return -1;
    }
    std::cout << "Triggering PR0 motion" << std::endl;

    // Update cached position (optional; ideally, poll actual position register here)
    current_position_ = position;

    return 0;
}

int ICLStepper::get_position() const {
    // Placeholder: in a real implementation, read back from the drive’s position registers.
    return current_position_;
}

int ICLStepper::jog(bool clockwise) {
    modbus_set_slave(ctx_, slave_id_);
    usleep(delay_us_);
    int command = clockwise ? 0x4001 : 0x4002;
    if (modbus_write_register(ctx_, 0x1801, command) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to jog motor: " << modbus_strerror(errno) << std::endl;
        return -1;
    }
    // Wait delay_us microseconds
    usleep(delay_us_);

    return 0;
}

int ICLStepper::set_jog_velocity(int velocity_rpm) {
    modbus_set_slave(ctx_, slave_id_);
    usleep(delay_us_);
    if (modbus_write_register(ctx_, 0x01E1, velocity_rpm) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to set target jog velocity: "
                  << modbus_strerror(errno) << std::endl;
        return -1;
    }
    usleep(delay_us_);
    return 0;
}

int ICLStepper::set_slave_id(int slave_id) {
    slave_id_ = slave_id;
    modbus_set_slave(ctx_, slave_id_);
    usleep(delay_us_);
    return 0;
}
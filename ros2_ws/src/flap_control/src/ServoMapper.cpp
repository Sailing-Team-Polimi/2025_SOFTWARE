#include "servo_mapper/servo_mapper.hpp"

namespace servo_utils {

ServoMapper::ServoMapper(float minV, float maxV)
    : min_valid(minV), max_valid(maxV) {}

float ServoMapper::saturate_angle(float angle) const {
    if (angle < min_valid) return min_valid;
    if (angle > max_valid) return max_valid;
    return angle;
}

uint16_t ServoMapper::map_to_pwm(float angle) const {
    angle = saturate_angle(angle);
    return static_cast<uint16_t>(
        (angle - min_angle) * (max_pwm - min_pwm) / (max_angle - min_angle) + min_pwm
    );
}

std::pair<uint8_t, uint8_t> ServoMapper::get_pwm_as_bytes(float angle) const {
    uint16_t pwm = map_to_pwm(angle);
    uint8_t msb = (pwm >> 8) & 0xFF;
    uint8_t lsb = pwm & 0xFF;
    return {msb, lsb};
}

int64_t ServoMapper::get_pwm_as_int64(float angle) const {
    return static_cast<int64_t>(map_to_pwm(angle));
}

}  // namespace servo_utils

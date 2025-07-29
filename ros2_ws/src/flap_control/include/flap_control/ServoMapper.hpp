#ifndef SERVO_MAPPER_HPP
#define SERVO_MAPPER_HPP

#include <cstdint>
#include <utility>

namespace servo_utils {

class ServoMapper {
private:
    float min_angle = -90.0;
    float max_angle = 90.0;
    uint16_t min_pwm = 1000;
    uint16_t max_pwm = 2000;
    float min_valid;
    float max_valid;

public:
    ServoMapper(float minV, float maxV);

    float saturate_angle(float angle) const;
    uint16_t map_to_pwm(float angle) const;
    std::pair<uint8_t, uint8_t> get_pwm_as_bytes(float angle) const;
    int64_t get_pwm_as_int64(float angle) const;
};

}  // namespace servo_utils

#endif  // SERVO_MAPPER_HPP

#pragma once

#include <cstdint>
#include "can.h"

class C620CAN
{
public:
    explicit C620CAN(CAN_HandleTypeDef* hcan);
    void init();
    void setCurrent(uint8_t motor_id, float current_amp);
    void sendCurrents();
    void updateMotorStatus();
    float getAngleDeg(uint8_t motor_id) const;
    int16_t getSpeedRpm(uint8_t motor_id) const;
    float getCurrentAmp(uint8_t motor_id) const;
    int8_t getTempDegC(uint8_t motor_id) const;

private:
    static constexpr float LIMIT_CURRENT_AMP = 6.0f;    // ユーザー定義の電流制限[A]
    static constexpr float MAX_CURRENT_AMP = 20.0f;     // 最大電流[A]
    static constexpr uint16_t MAX_CURRENT_RAW = 16384;   // 最大電流(16bit)
    static constexpr uint16_t MAX_ANGLE_RAW = 8191;     // 最大角度(16bit)

    struct MotorState
    {
        int16_t target_current_raw{};
        float angle_deg{};
        int16_t speed_rpm{};
        float current_amp{};
        int8_t temp_degC{};
    };

    CAN_HandleTypeDef* hcan_;
    MotorState motors_[8]{};
};

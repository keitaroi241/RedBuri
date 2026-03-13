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
    bool getAngleRaw(uint8_t motor_id, uint16_t& angle_raw) const;
    bool getSpeedRpm(uint8_t motor_id, int16_t& speed_rpm) const;
    bool getCurrentAmp(uint8_t motor_id, float& current_amp) const;
    bool getTempDegC(uint8_t motor_id, uint8_t& temp_degC) const;

private:
    static constexpr float LIMIT_CURRENT_AMP = 6.0f;    // ユーザー定義の電流制限[A]
    static constexpr float MAX_CURRENT_AMP = 20.0f;     // 最大電流[A]
    static constexpr uint16_t MAX_CURRENT_RAW = 16384;  // 最大電流(生データ)

    static_assert(LIMIT_CURRENT_AMP <= MAX_CURRENT_AMP);

    struct MotorState
    {
        int16_t target_current_raw{};
        uint16_t angle_raw{};
        int16_t speed_rpm{};
        float current_amp{};
        uint8_t temp_degC{};
    };

    CAN_HandleTypeDef* hcan_;
    MotorState motors_[8]{};
};

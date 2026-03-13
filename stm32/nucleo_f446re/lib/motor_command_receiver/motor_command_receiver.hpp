#pragma once

#include <cstdint>
#include "usart.h"

class MotorCommandReceiver
{
public:
    struct BaseCommand
    {
        int16_t motor_rpm;
        int16_t steer_deg;
    };

    struct ArmCommand
    {
        int16_t motor_rpm[7];
    };

    explicit MotorCommandReceiver(UART_HandleTypeDef& huart);
    void init();
    void callback();
    bool fetchBaseCommand(BaseCommand& cmd);
    bool fetchArmCommand(ArmCommand& cmd);

private:
    static constexpr uint8_t BUF_SIZE{64};
    UART_HandleTypeDef& huart_;
    uint8_t rx_byte_{};
    char buf_[BUF_SIZE]{};
    uint8_t len_{};
    BaseCommand base_cmd_{};
    ArmCommand arm_cmd_{};
    bool base_ready_{false};
    bool arm_ready_{false};

    bool parseBaseCommand(const char* line, BaseCommand& cmd);
    bool parseArmCommand(const char* line, ArmCommand& cmd);
    static bool parseInt(const char*& p, int16_t& value);
};
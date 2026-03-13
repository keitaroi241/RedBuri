#include "c620_can.hpp"
#include "can.h"
#include <cmath>

C620CAN::C620CAN(CAN_HandleTypeDef* hcan) : hcan_(hcan) {}

void C620CAN::init()
{
    // C620のCAN_ID: 0x201~0x208
    CAN_FilterTypeDef filter{};
    filter.FilterIdHigh         = 0x200 << 5;
    filter.FilterIdLow          = 0;
    filter.FilterMaskIdHigh     = 0x7F0 << 5;
    filter.FilterMaskIdLow      = 0;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.SlaveStartFilterBank = 14;
    filter.FilterActivation     = ENABLE;

    HAL_CAN_ConfigFilter(hcan_, &filter);
    HAL_CAN_Start(hcan_);
    HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void C620CAN::setCurrent(uint8_t motor_id, float target_current_amp) 
{
    if(motor_id == 0 || motor_id > 8) return;
    if(std::fabs(target_current_amp) >= LIMIT_CURRENT_AMP)
    {
        target_current_amp = std::copysign(LIMIT_CURRENT_AMP, target_current_amp);
    }

    float current_raw = (target_current_amp / MAX_CURRENT_AMP) * MAX_CURRENT_RAW;
    motors_[motor_id - 1].target_current_raw = static_cast<int16_t>(current_raw);
}

void C620CAN::sendCurrents()
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8];

    if(HAL_CAN_GetTxMailboxesFreeLevel(hcan_) >= 2)
    {
        // モーターID: 1~4
        tx_header.StdId = 0x200;
        tx_header.IDE = CAN_ID_STD;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = 8;
        tx_header.TransmitGlobalTime = DISABLE;
        for (int i = 0; i < 4; i++)
        {
            int16_t current = motors_[i].target_current_raw;
            tx_data[2*i] = (current >> 8) & 0xFF;
            tx_data[2*i + 1] = current & 0xFF;
        }

        HAL_CAN_AddTxMessage(hcan_, &tx_header, tx_data, &tx_mailbox);

        // モーターID: 5~8
        tx_header.StdId = 0x1FF;
        for (int i = 0; i < 4; i++)
        {
            int16_t current = motors_[i + 4].target_current_raw;
            tx_data[2*i] = (current >> 8) & 0xFF;
            tx_data[2*i + 1] = current & 0xFF;
        }

        HAL_CAN_AddTxMessage(hcan_, &tx_header, tx_data, &tx_mailbox);
    }
}

void C620CAN::updateMotorStatus() 
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if(HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
        if(rx_header.StdId < 0x201 || rx_header.StdId > 0x208) return;
        const uint8_t idx = rx_header.StdId - 0x201;

        motors_[idx].angle_raw = static_cast<uint16_t>(rx_data[0] << 8 | rx_data[1]);
        motors_[idx].speed_rpm = static_cast<int16_t>(rx_data[2] << 8 | rx_data[3]);
        float current_raw = static_cast<int16_t>(rx_data[4] << 8 | rx_data[5]);
        motors_[idx].current_amp = MAX_CURRENT_AMP * current_raw / MAX_CURRENT_RAW;
        motors_[idx].temp_degC = rx_data[6];
    }
}

uint16_t C620CAN::getAngleRaw(uint8_t motor_id) const
{
    if(motor_id == 0 || motor_id > 8) return -1.0f;
    return motors_[motor_id - 1].angle_raw;
}

int16_t C620CAN::getSpeedRpm(uint8_t motor_id) const
{
    if(motor_id == 0 || motor_id > 8) return -1;
    return motors_[motor_id - 1].speed_rpm;
}

float C620CAN::getCurrentAmp(uint8_t motor_id) const
{
    if(motor_id == 0 || motor_id > 8) return -1.0f;
    return motors_[motor_id - 1].current_amp;
}

int8_t C620CAN::getTempDegC(uint8_t motor_id) const
{
    if(motor_id == 0 || motor_id > 8) return -1;
    return motors_[motor_id - 1].temp_degC;
}

#include "motor_command_receiver.hpp"

MotorCommandReceiver::MotorCommandReceiver(UART_HandleTypeDef& huart)
    : huart_(huart)
{}

void MotorCommandReceiver::init()
{
    len_ = 0;
    HAL_UART_Receive_IT(&huart_, &rx_byte_, 1);
}

void MotorCommandReceiver::callback()
{
    const char c = static_cast<char>(rx_byte_);

    if(c == '\r')
    {
        HAL_UART_Receive_IT(&huart_, &rx_byte_, 1);
        return;
    }

    if(c == '\n')
    {
        buf_[len_] = '\0';

        if(buf_[0] == 'B')
        {
            BaseCommand cmd{};
            if(parseBaseCommand(buf_, cmd))
            {
                base_cmd_ = cmd;
                base_ready_ = true;
            }
        }
        else if(buf_[0] == 'A')
        {
            ArmCommand cmd{};
            if(parseArmCommand(buf_, cmd))
            {
                arm_cmd_ = cmd;
                arm_ready_ = true;
            }
        }

        len_ = 0;
        HAL_UART_Receive_IT(&huart_, &rx_byte_, 1);
        return;
    }

    if(len_ < BUF_SIZE - 1)
    {
        buf_[len_] = c;
        len_++;
    }
    else
    {
        len_ = 0;
    }

    HAL_UART_Receive_IT(&huart_, &rx_byte_, 1);
}

bool MotorCommandReceiver::fetchBaseCommand(BaseCommand& cmd)
{
    if(!base_ready_)
    {
        return false;
    }

    cmd = base_cmd_;
    base_ready_ = false;
    return true;
}

bool MotorCommandReceiver::fetchArmCommand(ArmCommand& cmd)
{
    if(!arm_ready_)
    {
        return false;
    }

    cmd = arm_cmd_;
    arm_ready_ = false;
    return true;
}

bool MotorCommandReceiver::parseBaseCommand(const char* line, BaseCommand& cmd)
{
    const char* p = line;

    if(*p != 'B')
    {
        return false;
    }
    p++;

    if(*p != ',')
    {
        return false;
    }
    p++;

    if(!parseInt(p, cmd.motor_rpm))
    {
        return false;
    }

    if(*p != ',')
    {
        return false;
    }
    p++;
    
    if(!parseInt(p, cmd.steer_deg))
    {
        return false;
    }

    if(*p != '\0')
    {
        return false;
    }

    return true;
}

bool MotorCommandReceiver::parseArmCommand(const char* line, ArmCommand& cmd)
{
    const char* p = line;

    if(*p != 'A')
    {
        return false;
    }
    p++;

    if(*p != ',')
    {
        return false;
    }
    p++;

    for(int i = 0; i < 7; i++)
    {
        if(!parseInt(p, cmd.motor_rpm[i]))
        {
            return false;
        }

        if(i < 6)
        {
            if(*p != ',')
            {
                return false;
            }
            p++;
        }
    }

    if(*p != '\0')
    {
        return false;
    }

    return true;
}

bool MotorCommandReceiver::parseInt(const char*& p, int16_t& value)
{
    bool negative = false;
    int32_t result = 0;

    if(*p == '-')
    {
        negative = true;
        p++;
    }

    if(*p < '0' || *p > '9')
    {
        return false;
    }

    while(*p >= '0' && *p <= '9')
    {
        result = result * 10 + (*p - '0');
        p++;
    }

    if(negative)
    {
        result = -result;
    }

    value = static_cast<int16_t>(result);
    return true;
}
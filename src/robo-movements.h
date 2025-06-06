#ifndef __ROBO_MOVEMENTS_H
#define __ROBO_MOVEMENTS_H
// Static Libs
#include "pins.h"

// source: https://registry.platformio.org/libraries/aruncs31s/Robo%20Initial%20Postions
#include <initial-positions.h>
#include <Adafruit_PWMServoDriver.h>


class Robo{
    public:
        Robo(uint8_t _servo_id);
       
        ~Robo();
        
        static void set_board(Adafruit_PWMServoDriver& board);
        void initialize();
        
        static uint8_t get_total_num_servos();
        static uint8_t get_current_num_servos();
        uint8_t get_current_angle();
        uint16_t get_current_pulse();

        void set_angle(uint8_t _angle);
        // //@brief set the angle of the servo
        void set_pulse(uint16_t _pulse);

        uint16_t get_pulse(uint8_t _angle);

        
    private:
        static uint8_t _total_num_servos;
        static uint8_t _current_num_servos;
        uint8_t _servo_id;
        uint8_t _initial_angle;
        uint8_t _current_angle;
        uint8_t _previous_angle;
        static Adafruit_PWMServoDriver* _board ;
        
};



#endif // __ROBO_MOVEMENTS_H
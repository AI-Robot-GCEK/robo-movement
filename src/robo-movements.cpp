/*
 * Author: Arun CS
 * Repo: https://github.com/AI-Robot-GCEK/robo-movements
 */

#include "robo-movements.h"
#include "configs.h"
#include <Arduino.h>

uint8_t Robo::_total_num_servos = 0;
uint8_t Robo::_current_num_servos = 0;
Adafruit_PWMServoDriver *Robo::_board = nullptr;
/*!
 * @brief Instantiates new  Robo Part Object. This will automatically allign the
 * initial positions of the servo
 * @param _servo_id 0-15
 * @param board The Adafruit_PWMServoDriver object to control the servo
 */

Robo::Robo(uint8_t _servo_id) : _servo_id(_servo_id) {
  _total_num_servos++;
  _current_num_servos++;
  // Set the initial angle based on the servo ID
  switch (_servo_id) {
  case PIN_LA1:
    _initial_angle = LA1_INITIAL_POSITION;
    break;
  case PIN_LA2:
    _initial_angle = LA2_INITIAL_POSITION;
    break;
  case PIN_LA3:
    _initial_angle = LA3_INITIAL_POSITION;
    break;
  case PIN_RA1:
    _initial_angle = RA1_INITIAL_POSITION;
    break;
  case PIN_RA2:
    _initial_angle = RA2_INITIAL_POSITION;
    break;
  case PIN_RA3:
    _initial_angle = RA3_INITIAL_POSITION;
    break;
#if defined(PIN_LH)
  case PIN_LH:
    _initial_angle = LH_INITIAL_POSITION;
    break;
#endif
#if defined(PIN_RH)
  case PIN_RH:
    _initial_angle = RH_INITIAL_POSITION;
    break;
#endif
#if defined(PIN_B1)
  case PIN_B1:
    _initial_angle = LH_INITIAL_POSITION;
    break;
#endif
#if defined(PIN_B2)
  case PIN_B2:
    _initial_angle = RH_INITIAL_POSITION;
    break;
#endif
  case PIN_LL1:
    _initial_angle = LL1_INITIAL_POSITION;
    break;
  case PIN_LL2:
    _initial_angle = LL2_INITIAL_POSITION;
    break;
  case PIN_LL3:
    _initial_angle = LL3_INITIAL_POSITION;
    break;
  case PIN_RL1:
    _initial_angle = RL1_INITIAL_POSITION;
    break;
  case PIN_RL2:
    _initial_angle = RL2_INITIAL_POSITION;
    break;
  case PIN_RL3:
    _initial_angle = RL3_INITIAL_POSITION;
    break;
  case PIN_LF:
    _initial_angle = LF_INITIAL_POSITION;
    break;
  case PIN_RF:
    _initial_angle = RF_INITIAL_POSITION;
    break;
  default:
    _initial_angle = 0;
    break;
  }
  Serial.println("Initial Angle is set");
  // Change this
  _previous_angle = 0;
  _current_angle = _initial_angle;
  _current_pulse = get_pulse(_initial_angle);
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Servo ID: ");
  Serial.print(_servo_id);
  Serial.print(" Initial Angle: ");
  Serial.println(_initial_angle);
  Serial.print("Current Pulse: ");
  Serial.println(_current_pulse);
#endif
}

Robo::~Robo() {
  _current_num_servos--;
}

void Robo::initialize() {
    // Set the initial angle of the servo

    Serial.println("Executed Initialize");
    // _board.setPWM(_servo_id, 0, 200);
    _current_angle = _initial_angle;
#ifdef ENABLE_DEBUG_OUTPUT
    Serial.print("Servo ID: ");
    Serial.print(_servo_id);
    Serial.print(" Initial Angle: ");
    Serial.println(_initial_angle);
#endif
    Serial.println("Complete Executed Initialize");
  }

  void Robo::set_board(Adafruit_PWMServoDriver & board) {
    _board = &board;
    Serial.println("Executed Begin ");
    _board->begin();
    _board->setPWMFreq(SERVO_FREQ);
#ifdef ENABLE_DEBUG_OUTPUT
    Serial.println("Board Set");
#endif
  }

  /*!
   * @return The total number of servos.
   */
  uint8_t Robo::get_total_num_servos() { return Robo::_total_num_servos; }
  /*!
   * @return The current number of servos.
   */
  uint8_t Robo::get_current_num_servos() { return _current_num_servos; }

  // /*!
  // * @warning used by the PWM Driver
  // * @returns count driver output value  out of 4096
  // */
  uint16_t Robo::get_pulse(uint8_t _angle) {
    // constrain the angle to the min and max values
    // _angle = constrain(_angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
    uint16_t _pulse =
        map(_angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX, SERVO_MIN, SERVO_MAX);
    _current_angle = _angle; // Update current angle
#ifdef ENABLE_DEBUG_OUTPUT
    Serial.print("Angle: ");
    Serial.print(_angle);
    Serial.print("pulse: ");
    Serial.println(_pulse);
#endif
    return _pulse;
  }

  uint8_t Robo::get_current_angle() { return _current_angle; }

  uint16_t Robo::get_current_pulse() { return get_pulse(_current_angle); }



  void Robo::set_angle(uint8_t _angle) {
    _previous_angle = _current_angle;

    _current_angle = _angle;
    
    uint16_t _delta_pulse = abs(get_pulse(_current_angle) - get_pulse(_previous_angle));

    if (_previous_angle <  _current_angle ) {
      for (uint16_t __offset = 0; __offset < _delta_pulse; __offset++) {
        /* Offset will grow from 0 to delta*/
        _board->setPWM(_servo_id, 0, _current_pulse + __offset);
        delay(10);
      }
    } else if (_previous_angle > _current_angle) {
      for (int __offset = _delta_pulse ; __offset > 0; __offset++) {
        /* Offset will reduce from delta to 0 */
        _board->setPWM(_servo_id, 0, _current_pulse - __offset);
        delay(10);
      }
    }
    // If the angle is more than 10 degrees, set the pulse directly
    _current_pulse = get_pulse(_angle);
  }

  void Robo::set_pulse(uint16_t _pulse) {
    _current_angle =
        map(_pulse, SERVO_MIN, SERVO_MAX, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
    _board->setPWM(_servo_id, 0, _pulse);
  }

#if !defined(NUM_SERVOS)
#define NUM_SERVOS 16
#endif 
  void Movement::_move(move_position_t* _positions[], uint8_t _num_positions,Robo* robots[NUM_SERVOS]) {
  if(_num_positions < 0 ){
    return;
  }
  else{

  }

  }



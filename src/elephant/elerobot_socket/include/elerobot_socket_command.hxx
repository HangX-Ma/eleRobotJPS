/**
 * @file elerobot_socket_command.hxx
 * @brief elephant robot socket command
 * @author m-contour
 * @date 2021-2022
 * @copyright Copyright (c) 2021-2022 m-contour. All rights reserved.
 * @par License:\n This project is released under the Berkerley Software Distribution License.
 */
#ifndef ELEROBOT_SOCKET_COMMAND_HXX
#define ELEROBOT_SOCKET_COMMAND_HXX

namespace eleRobot{

enum class command
{
  GET_ANGLES = 0,
  SET_ANGLES,
  SET_ANGLE,
  GET_COORDS,
  SET_COORDS,
  SET_COORD,
  GET_DIGIT_OUT_PIN,
  SET_DIGIT_OUT_PIN,
  GET_DIGIT_IN_PIN,
  SET_DIGIT_IN_PIN,
  SET_ANALOG_OUT_PIN,
  SET_COORD_CONTINUOUSLY,
  SET_ANGLE_CONTINUOUSLY,
  ENABLE_SYS,
  DISABLE_SYS,
  STOP_TASK,
  SET_FEED_RATE,
  SLEEP,
  SET_UPSIDE_DOWN,
  POWER_ON,
  POWER_OFF,
  CHECK_ROBOT_STATE,
  CHECK_RUNNING,
  SET_TORQUE_LIMIT,
  GET_ERROR_INFO,
  SET_PAYLOAD,
  SET_ACC,
  GET_ACC,
  ASSIGN_VAR,
  GET_VAR,
  WAIT_COMMAND_DONE,
  PAUSE,
  RESUME,
};

class base_command{
  public:
    base_command();

    void get_angles();

    void set_angles();

    void set_angle();

    void get_coords();

    void set_coords();
    
    void set_coord();
    
    void get_digital_out();

    void set_digital_out();

    void get_digital_in();

    void set_analog_out();

    void jog_coord();

    void jog_angle();

    void state_on();

    void state_off();

    void task_stop();

    void set_feed_rate();

    void wait();

    void set_upside_down();

    void power_on();

    void power_off();

    void get_speed();

    void check_running();

    void set_torque_limit();

    void read_next_error();

    void set_payload();

    void set_acceleration();

    void get_acceleration();

    void assign_variable();

    void get_variable();

    void wait_command_done();

    void pause_program();

    void resume_program();

  private:
    std::string socket_string_;
};

}
#endif
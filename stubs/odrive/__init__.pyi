# This is faar from complete.
# The ACTUAL solution would be to either:
# 
# copy over the public attributes by hand from:
#   https://github.com/madcowswe/ODrive/tree/devel/Firmware/MotorControl
#   or:
#   https://github.com/madcowswe/ODrive/blob/devel/Firmware/odrive-interface.yaml
#   which goes against all the "automate everything!!" bones in my body,
#   but realistically would be quicker to do and probably not much work
#   to maintain afterwards
#
# or,
#
# use that directory + possibly the fibre thingy to autogenerate .pyi
#   files, which (again) realistically would take more of your time
#   (at least initially) but would be so satisfying once done :)

from typing import Any, Union


def find_any() -> Union[ODrive, None]: ...


class ODrive:
    axis0: Axis
    axis1: Axis
    def save_configuration(self) -> None: ...
    def reboot(self) -> None: ...
    # vbus_voltage = 31.90107536315918 (float)
    # serial_number = 20603595524B (int)
    # hw_version_major = 3 (int)
    # hw_version_minor = 6 (int)
    # hw_version_variant = 56 (int)
    # fw_version_major = 0 (int)
    # fw_version_minor = 4 (int)
    # fw_version_revision = 0 (int)
    # fw_version_unreleased = 1 (int)
    # user_config_loaded = True (bool)
    # brake_resistor_armed = True (bool)
    # system_stats:
    #     uptime = 107458 (int)
    #     min_heap_space = 1368 (int)
    #     min_stack_space_axis0 = 7868 (int)
    #     min_stack_space_axis1 = 7868 (int)
    #     min_stack_space_comms = 9956 (int)
    #     min_stack_space_usb = 3260 (int)
    #     min_stack_space_uart = 3932 (int)
    #     min_stack_space_can = 1864 (int)
    #     min_stack_space_usb_irq = 1796 (int)
    #     min_stack_space_startup = 468 (int)
    #     usb: ...
    #     i2c: ...
    # config:
    #     brake_resistance = 10.0 (float)
    #     enable_uart = True (bool)
    #     enable_i2c_instead_of_can = False (bool)
    #     enable_ascii_protocol_on_usb = True (bool)
    #     dc_bus_undervoltage_trip_level = 8.0 (float)
    #     dc_bus_overvoltage_trip_level = 59.92000198364258 (float)
    #     gpio1_pwm_mapping: ...
    #     gpio2_pwm_mapping: ...
    #     gpio3_pwm_mapping: ...
    #     gpio4_pwm_mapping: ...
    #     gpio3_analog_mapping: ...
    #     gpio4_analog_mapping: ...
    # can:
    #     error = 0x0000 (int)
    #     config: ...
    #     can_protocol = 0 (int)
    #     set_baud_rate(baudRate: int)
    # test_property = 0 (int)
    # test_function(delta: int)
    # get_oscilloscope_val(index: int)
    # get_adc_voltage(gpio: int)
    # save_configuration()
    # erase_configuration()
    # reboot()
    # enter_dfu_mode()

class Axis:
  error: int
  step_dir_active: bool
  current_state: int
  requested_state: int
  loop_counter: int
  lockin_state: int
  controller: Controller
  encoder: Encoder
  sensorless_estimator: SensorlessEstimator
  motor: Motor
  trap_traj: TrapTraj
  config: AxisConfig

class SensorlessEstimator:
  error: int
  phase: float
  pll_pos: float
  vel_estimate: float
  config: ...

class TrapTraj:
  config: ...

class AxisConfig:
  startup_motor_calibration: bool
  startup_encoder_index_search: bool
  startup_encoder_offset_calibration: bool
  startup_closed_loop_control: bool
  startup_sensorless_control: bool
  enable_step_dir: bool
  counts_per_step: float
  watchdog_timeout: float
  step_gpio_pin: int
  dir_gpio_pin: int
  can_node_id: int
  can_heartbeat_rate_ms: int
  calibration_lockin: ...
  sensorless_ramp: ...
  general_lockin: ...

class Motor:
  error: int
  armed_state: int
  is_calibrated: bool
  current_meas_phB: float
  current_meas_phC: float
  DC_calib_phB: float
  DC_calib_phC: float
  phase_current_rev_gain: float
  thermal_current_lim: float
  get_inverter_temp: ...
  current_control: ...
  gate_driver: ...
  timing_log: ...
  config: ...

class Controller:
  pos_setpoint: float
  vel_setpoint: float
  vel_integrator_current: float
  current_setpoint: float
  vel_ramp_target: float
  config: ControllerConfig
  error: int
  def set_pos_setpoint(self, pos_setpoint: float, vel_feed_forward: float, current_feed_forward: float): ...
  def set_vel_setpoint(self, vel_setpoint: float, current_feed_forward: float): ...
  def set_current_setpoint(self, current_setpoint: float): ...
  def move_to_pos(self, pos_setpoint: float): ...
  def move_incremental(self, displacement: float, from_goal_point: bool): ...
  def start_anticogging_calibration(self): ...


class ControllerConfig:
  control_mode: int
  offset: float  # or int ?
  pos_gain: float
  vel_gain: float
  vel_integrator_gain: float

class Encoder:
  error: int
  is_ready: bool
  index_found: bool
  shadow_count: int
  count_in_cpr: int
  interpolation: float
  phase: float
  pos_estimate: float
  pos_cpr: float
  hall_state: int
  vel_estimate: float
  calib_scan_response: float
  config: EncoderConfig
  def set_linear_count(self, count: int): ...

class EncoderConfig:
  mode: int
  use_index: bool
  find_idx_on_lockin_only: bool
  zero_count_on_find_idx: bool
  cpr: int
  offset: int
  pre_calibrated: bool
  offset_float: float
  enable_phase_interpolation: bool
  bandwidth: float
  calib_range: float
  calib_scan_distance: float
  calib_scan_omega: float
  idx_search_unidirectional: bool
  ignore_illegal_hall_state: bool
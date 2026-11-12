# Prusa CORE One chamber: temperature and chamber fan regulation
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# The CORE One has no chamber heater: the bed heats the chamber and the
# chamber fans (on the xBuddy extension) let heat out. This follows the
# stock firmware (Prusa-Firmware-Buddy, feature/chamber and
# feature/xbuddy_extension/cooling): the thermistor reading is corrected for
# its position next to the bed, the fans regulate down to the target with an
# integrating controller, and run at full speed above an overheat limit.
import math
from . import fan

PWM_MAX = 255.
CONTROL_PERIOD = 1.0
WAIT_TOLERANCE = 3.
WAIT_WARN_TIME = 30. * 60.
OFFSET_MIN_TEMP = 20.
MAX_FAN_TIME = 5.0

class PrusaChamber:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.fan = fan.Fan(config, default_shutdown_speed=1.)
        self.min_temp = config.getfloat('min_temp', 0.)
        self.max_temp = config.getfloat('max_temp', 68., above=self.min_temp)
        pheaters = self.printer.load_object(config, 'heaters')
        self.sensor = pheaters.setup_sensor(config)
        self.sensor.setup_minmax(self.min_temp, self.max_temp)
        self.sensor.setup_callback(self.temperature_callback)
        pheaters.register_sensor(config, self)
        self.speed_delay = self.sensor.get_report_time_delta()
        self.max_target = config.getfloat('max_target', 60.,
                                          above=self.min_temp,
                                          maxval=self.max_temp)
        self.overheat_temp = config.getfloat('overheat_temp', 65.,
                                             maxval=self.max_temp)
        self.recovery_temp = config.getfloat('recovery_temp', 60.,
                                             below=self.overheat_temp)
        self.min_speed = config.getfloat('min_speed', 40. / PWM_MAX,
                                         minval=0., maxval=1.)
        self.max_auto_speed = config.getfloat('max_auto_speed', 1.,
                                              above=0., maxval=1.)
        # Integration gain in PWM (0-255) steps per degree per second
        self.integration = config.getfloat('integration_constant', 1.5,
                                           above=0.)
        # Thermistor position correction; the default is Buddy's for the L:
        # 8 / ((bed_max - 20) * sqrt(chamber_max - 20)), bed_max 120,
        # chamber_max 65
        self.bed_heater_name = config.get('bed_heater', None)
        self.offset_gain = config.getfloat(
            'bed_offset_gain', 8. / (100. * math.sqrt(45.)), minval=0.)
        self.bed_heater = None
        # State
        self.raw_temp = self.temp = 0.
        self.target = None
        self.manual_speed = None
        self.regulation = 0.
        self.overheated = False
        self.last_control_time = 0.
        self.last_speed = 0.
        self.next_speed_time = 0.
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('M141', self.cmd_M141,
                               desc=self.cmd_M141_help)
        gcode.register_command('M191', self.cmd_M191,
                               desc=self.cmd_M191_help)
        gcode.register_command('SET_CHAMBER_FAN', self.cmd_SET_CHAMBER_FAN,
                               desc=self.cmd_SET_CHAMBER_FAN_help)
        gcode.register_command('CHAMBER_STATUS', self.cmd_CHAMBER_STATUS,
                               desc=self.cmd_CHAMBER_STATUS_help)
    def _handle_connect(self):
        if self.bed_heater_name is not None:
            pheaters = self.printer.lookup_object('heaters')
            self.bed_heater = pheaters.lookup_heater(self.bed_heater_name)
    # Temperature and fan control
    def _corrected_temp(self, raw, eventtime):
        if self.bed_heater is None:
            return raw
        bed_temp = self.bed_heater.get_temp(eventtime)[0]
        if bed_temp <= raw or raw <= OFFSET_MIN_TEMP:
            return raw
        return raw + (self.offset_gain * (bed_temp - raw)
                      * math.sqrt(raw - OFFSET_MIN_TEMP))
    def temperature_callback(self, read_time, temp):
        reactor = self.printer.get_reactor()
        self.raw_temp = temp
        self.temp = self._corrected_temp(temp, reactor.monotonic())
        if read_time < self.last_control_time + CONTROL_PERIOD:
            return
        self.last_control_time = read_time
        self._set_speed(read_time, self._control_step())
    def _control_step(self):
        temp = self.temp
        if temp >= self.overheat_temp:
            self.overheated = True
        elif temp < self.recovery_temp:
            self.overheated = False
        if self.manual_speed is None and self.target is not None:
            err = temp - self.target
            self.regulation = max(0., min(self.max_auto_speed * PWM_MAX,
                                          self.regulation
                                          + self.integration * err))
            speed = self.regulation / PWM_MAX
        else:
            self.regulation = 0.
            speed = self.manual_speed or 0.
        if self.overheated:
            return 1.
        if speed <= 0.:
            return 0.
        return max(speed, self.min_speed)
    def _set_speed(self, read_time, value):
        if (read_time < self.next_speed_time
                and abs(value - self.last_speed) < .01):
            return
        speed_time = read_time + self.speed_delay
        self.next_speed_time = speed_time + 0.75 * MAX_FAN_TIME
        self.last_speed = value
        self.fan.set_speed(value, speed_time)
    def set_target(self, temp):
        if not temp:
            self.target = None
            return None
        self.target = max(self.min_temp, min(self.max_target, temp))
        return self.target
    # Interface for heaters (M105, TEMPERATURE_WAIT) and status
    def get_temp(self, eventtime):
        return self.temp, self.target or 0.
    def stats(self, eventtime):
        return False, '%s: temp=%.1f raw=%.1f target=%.0f speed=%.3f' % (
            self.name, self.temp, self.raw_temp, self.target or 0.,
            self.last_speed)
    def get_status(self, eventtime):
        status = self.fan.get_status(eventtime)
        status['temperature'] = round(self.temp, 2)
        status['thermistor_temperature'] = round(self.raw_temp, 2)
        status['target'] = self.target or 0.
        status['overheated'] = self.overheated
        status['manual'] = self.manual_speed is not None
        return status
    # G-Code commands
    cmd_M141_help = "Set the chamber target (0 = no target)"
    def cmd_M141(self, gcmd):
        temp = gcmd.get_float('S', 0., minval=0.)
        target = self.set_target(temp)
        if target is not None and target < temp:
            gcmd.respond_info("Chamber target limited to %.0f C" % (target,))
    cmd_M191_help = "Set the chamber target and wait for it"
    def cmd_M191(self, gcmd):
        for key, heating, cooling in (('S', True, False), ('R', True, True),
                                      ('C', False, True)):
            if gcmd.get_float(key, None) is not None:
                break
        else:
            return
        target = self.set_target(gcmd.get_float(key, minval=0.))
        if target is None:
            return
        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback((lambda pt: None))
        reactor = self.printer.get_reactor()
        eventtime = start = reactor.monotonic()
        warned = False
        while not self.printer.is_shutdown():
            temp = self.temp
            if ((not heating or temp >= target - WAIT_TOLERANCE)
                    and (not cooling or temp <= target + WAIT_TOLERANCE)):
                break
            if not warned and eventtime - start > WAIT_WARN_TIME:
                warned = True
                gcmd.respond_info("Chamber still at %.1f C after 30 min"
                                  " (target %.0f C)" % (temp, target))
            gcmd.respond_raw("C:%.1f /%.0f" % (temp, target))
            eventtime = reactor.pause(eventtime + 1.)
    cmd_SET_CHAMBER_FAN_help = (
        "Set the chamber fans: SPEED=0..1 for manual, AUTO=1 to regulate")
    def cmd_SET_CHAMBER_FAN(self, gcmd):
        if gcmd.get_int('AUTO', 0):
            self.manual_speed = None
        else:
            self.manual_speed = gcmd.get_float('SPEED', minval=0., maxval=1.)
        self.next_speed_time = 0.

    cmd_CHAMBER_STATUS_help = "Report chamber temperature and fan state"
    def cmd_CHAMBER_STATUS(self, gcmd):
        eventtime = self.printer.get_reactor().monotonic()
        status = self.get_status(eventtime)
        if self.manual_speed is not None:
            mode = "manual"
        elif self.target is not None:
            mode = "auto"
        else:
            mode = "off"
        rpm = status.get('rpm')
        gcmd.respond_info(
            "Chamber %.1f C (thermistor %.1f C), target %s\n"
            "Fans %.0f%% (%s%s)%s" % (
                self.temp, self.raw_temp,
                "%.0f C" % (self.target,) if self.target else "none",
                self.last_speed * 100., mode,
                ", overheated" if self.overheated else "",
                ", %.0f RPM" % (rpm,) if rpm is not None else ""))

def load_config(config):
    return PrusaChamber(config)

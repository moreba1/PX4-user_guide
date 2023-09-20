# Multicopter Configuration

Multicopter configuration and calibration follows the same high level steps as other frames: selection of firmware, configuration of the frame including actuator/motor geometry and output mappings, sensor configuration and calibration, configuration of safety and other features, and finally tuning.

This topic explains how to configure a multicopter using selected topics from [Standard Configuration](../config/README.md), [Advanced Configuration](../advanced_config/README.md), and [Flight Controller Peripherals](../peripherals/README.md), along with multicopter-specific tuning topics.

:::note
This topic is the recommended entry point when performing first-time configuration and calibration of a new multicopter frame.
:::

## Loading Firmware

The first step is to [load PX4 firmware](../config/firmware.md) onto your [flight controller](../flight_controller/README.md).
This is most easily done using QGroundControl, which will automatically select appropriate firmware for your particular controller hardware.
By default QGC will install the latest stable version of PX4, but you can choose beta or custom versions instead if needed.

Relevant topics:

- [Loading Firmware](../config/firmware.md)

## Frame Selection and Configuration

This section explains how to configure the vehicle type (multicopter), specific motor/flight control geometry, and motor outputs.

First [select a multicopter airframe](../config/airframe.md) (options are listed in [Airframe Reference > Copter](../airframes/airframe_reference.md#copter)).
You should select the frame that matches your vehicle brand and model if one exists, and otherwise select the "Generic" frame type that most closely matches your geometry in terms of number of motors and their relative positions.
For example, for a [Quadrotor X](../airframes/airframe_reference.md#quadrotor-x) frame you would look for the name of your frame in the list, and if it was not present select the [Generic Quadrotor X](../airframes/airframe_reference.md#copter_quadrotor_x_generic_quadcopter) frame.


:::note
Any selected multicopter frame can be modified in the next step (actuator configuration) to add/remove motors and otherwise changed the geometry.
Selecting a frame that matches your vehicle reduces the configuration work required.

:::details How does this work (details)
Selecting an airframe applies a [frame configuration file](../dev_airframes/adding_a_new_frame.md#adding-a-frame-configuration) that contains a predefined set of [parameters](../advanced_config/parameters.md), such as [CA_AIRFRAME=0](../advanced_config/parameter_reference.md#CA_AIRFRAME) for the vehicle type and [CA_ROTOR_COUNT](../advanced_config/parameter_reference.md#CA_ROTOR_COUNT) for the number of rotors.

Frames named with model/brand will define the vehicle type, number of rotors, actual rotor positions, and motor directions.
They might also contain tuning and calibration parameters that are good enough to fly or at least auto-tune the vehicle "out of the box".
They don't include the output mappings, unless the frame is pre-wired and non modifiable (so you would have to set this yourself in the next step).
  
The frames named with "Generic" generally just define the vehicle type, number of rotors, and "placeholder" rotor positions; almost everything needs to be configured.
:::

The next step is to define your vehicle [geometry](../config/actuators.md#motor-geometry-multicopter) (the number of motors and their relative positions) and [assign those motors](../config/actuators.md#actuator-outputs) to the physical outputs that they are wired to on your flight controller (both of these are covered in [Actuator Configuration and Testing](../config/actuators.md)).

If using PWM ESCs and OneShot ESCs (but not DShot and DroneCAN/Cyphal ESC) you should then perform [ESC Calibration](../advanced_config/esc_calibration.md) before proceeding to [Motor Configuration](../config/actuators.md#motor-configuration).
This ensures that all ESC provide exactly the same output for a given input (ideally we'd calibrate ESCs first, but you can't calibrate your ESCs until outputs are mapped).

The final step is [Motor Configuration](../config/actuators.md#motor-configuration), which sets the directions of motors, and their disarmed, armed and high limits.

Relevant topics:

- [Vehicle (Frame) Selection](../config/airframe.md) - Select vehicle type to match your frame.
- [Actuator Configuration and Testing](../config/actuators.md) - Vehicle geometry, output mapping, motor configuration, testing.
- [ESC Calibration](../advanced_config/esc_calibration.md) - Do between output mapping and motor configuration (topic above) for PWM and OneShot ESC.

## Sensor Setup and Calibration

PX4 most commonly relies on a magnetometer (compass) for direction information, a barometer for altitude, a gyroscope for attitude, an accelerometer for velocity calculations, and a GPS/GNSS for global position.
Pixhawk flight controllers (and many others) have inbuilt magnetometer, accelerometer, gyroscope, and barometer.
The inbuilt compass usually isn't particularly reliable, and it is common to also add an external compass (usually with a GPS for position).

We first need to set the [Sensor Orientation](../config/flight_controller_orientation.md), informing PX4 how the autopilot (and its inbuilt sensors) and external compasses are oriented relative to the vehicle.
Generally you'll orient towards the front of the vehicle and not have to set anything.
Once that is done we need to calibrate the compass(es), gyroscope, and accelerometer.

The core sensor setup is covered in these topics.

- [Sensor Orientation](../config/flight_controller_orientation.md)
- [Compass](../config/compass.md)
- [Gyroscope](../config/gyroscope.md)
- [Accelerometer](../config/accelerometer.md)

PX4 can use other peripherals, such as distance sensors, optical flow sensors, traffic avoidance alarms, cameras, and so on.

- [Flight Controller Peripherals](../peripherals/README.md) - Setup specific sensors, optional sensors, actuators, and so on.

:::note
Barometers don't need calibration, and if you've mounted the flight controller level, there isn't much value in a [Level Horizon](../config/level_horizon_calibration.md) calibration either.

Obviously you don't need to calibrate or configure anything that isn't present or used by PX4.
For multicopter, that includes [airspeed sensors](../config/airspeed.md), which are not used for flight control.
:::

## GPS Setup

<!-- TBD

Note that it is plugnplay except for RTK. Worth saying.

-->

## Manual Control Setup

<!-- note that you will have one or the other -->

Radio Control:

- [Radio Controller (RC) Setup](../config/radio.md)
- [Flight Mode Configuration](../config/flight_mode.md)

Joystick/GamePad:

- [Joystick Setup](../config/joystick.md)

## Safety Configuration

- [Battery/Power Module Setup](../config/battery.md)
- [Safety Configuration (Failsafes)](../config/safety.md)

## Tuning

Tuning is the final step, carried out only after most other setup and configuration is complete.

<!--
- Explain what you have to tune on PX4, what you can tune, and what each topic covers
- I expect we should start with an exhaustive list of the tuning you could want to do - such as position tuning, etc. Do we have one?
 -->

- [Autotune](../config/autotune.md) (Recommended on vehicles and frames that support it)
- [MC Filter/Control Latency Tuning](../config_mc/filter_tuning.md)
- [MC PID Tuning (Manual/Basic)](../config_mc/pid_tuning_guide_multicopter_basic.md)
- [MC PID Tuning Guide (Manual/Detailed)](../config_mc/pid_tuning_guide_multicopter.md)
- [MC Setpoint Tuning (Trajectory Generator)](../config_mc/mc_trajectory_tuning.md)
  - [MC Jerk-limited Type Trajectory](../config_mc/mc_jerk_limited_type_trajectory.md)
- [Multicopter Racer Setup](../config_mc/racer_setup.md)

<!-- TBD this is just text for me to mine

AFAIK autotune was tested on various not so custom platforms e.g. X500, racer quad, Loong standard VTOL. I honestly used it only once on a tricopter and it worked for roll and pitch but the resulting yaw tuning was not stable. Since then it was improved but that's not merged yet :eyes: https://github.com/PX4/PX4-Autopilot/pull/21857
Autotune was never tested on a Helicopter.
can you in theory autotune frame with any number of motors?
In theory yes but it needs to be able to have reasonable authority around all axes so I'd expect autotune to not work well for a monocopter without swashplate and so on. Probably also the controllers wouldn't work out of the box. I saw issues before with designs that tilt the rotor e.g. tricopter, bicopter, ... again


will PX4 still understand how to autotune?
Autotune should work for any vehicle that has reasonable authority and dynamics around all the body axes. A tiltable motor e.g. tricopter has at the least dynamics which are less tested with autotune.
My assumption is that the mixing system can cope with whatever geometry you throw at it.
Yes but it must be physically feasible. E.g. if you make a quadrotor where all motors turn the same way it will "deal" with it but that cannot work without very specific controllers. Same for a monocopter or a tricopter without swiveling one motor.
-->

## See Also

- [QGroundControl > Setup](https://docs.qgroundcontrol.com/master/en/SetupView/SetupView.html)
- [Flight Controller Peripherals](../peripherals/README.md) - Setup specific sensors, optional sensors, actuators, and so on.
- [Advanced Configuration](../advanced_config/README.md) - Factory/OEM calibration, configuring advanced features, less-common configuration.
- Vehicle-Centric Config/Tuning:

  - [Multicopter Config/Tuning](../config_mc/README.md)
  - [Helicopter Config/Tuning](../config_heli/README.md)
  - [Fixed Wing Config/Tuning](../config_fw/README.md)
  - [VTOL Config/Tuning](../config_vtol/README.md)

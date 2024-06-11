# roboclaw_driver
ROS driver for the Roboclaw motor controllers

## Usage
### Nodes
- `roboclaw_driver`
  ```
  rosrun roboclaw_driver driver.py
  ```
  Parameters are set on node initialization  
  Parameters:  
  -  `~device`
      - Type: `string`
      - Default:  `/dev/ttyUSB0`

      Port device name
  -  `~baudrate`
      - Type:     `integer`
      - Default:  `115200`
      - Unit:     `bps`
  
      Port baudrate 
  -  `~address`
      - Type:     `integer`
      - Default:  `128`
  
      Roboclaw controller address. Has to be between 128 and 135. 
  -  `~motors`
      - Type:     `list`
      - Default:  `[1,2]`
  
      Motors used by the node. Valid values are: `[]`,`[1]`,`[2]`,`[1,2]`
  -  `~communication_timeout`
      - Type:     `integer`
      - Default:  imported from controller
      - Unit:     `ms`
  
      Time after which motors will stop If communication with the controller is lost.
  -  `~multiunit_mode`
      - Type:     `boolean`
      - Default:  imported from controller
  
      Sets S2 pin to open drain. Allows multiple Roboclaws to be controlled from a single serial port.
  -  `~relay_mode`
      - Type:     `boolean`
      - Default:  imported from controller
  
      Enables RoboClaw to pass data from USB through S1 (RX) and S2 (TX). Allows several RoboClaws to be networked from one USB connection. All connected RoboClaw's baud rates must be set to the same.
  -  `~swap_encoders`
      - Type:     `boolean`
      - Default:  imported from controller
  
      This option will swap encoder channels. Pair encoder 1 to motor channel 2 and encoder 2 to motor channel 1.
  -  `~swap_buttons`
      - Type:     `boolean`
      - Default:  imported from controller
  
      Swaps Mode and LIPO button interface. Only affects hardware V5 and RoboClaw 2x15, 2x30 and 2x45.
  -  `~motor1/command_timeout`   
     (`~motor2/command_timeout`)
      - Type:     `integer`
      - Default:  `500`
      - Unit:     `ms`
  
      Time after which motor will stop If there are no new messages on topic `~/motor1/joint_state` (`~/motor2/joint_state`)
  -  `~motor1/ticks_per_revolution`   
     (`~motor2/ticks_per_revolution`)
      - Type:     `double`
      - Default:  `360`
      - Unit:     `ms`
  
      For quadrature encoders - number of encoder quadrature pulses per revolution.  
      For aboslute encoders - voltage gain per revolution. 0..2V corresponds to values 0..2047
  -  `~motor1/torque_constant`   
     (`~motor2/torque_constant`)
      - Type:     `double`
      - Default:  `1`
      - Unit:     `N·m/A`
  
      Torque-current relationship of the motor.  
      τ = K<sub>T</sub> · I  
      Where:  
      - τ = Motor torque  
      - K<sub>T</sub> = Torque constant  
      - I = motor current
  -  `~motor1/absolute_encoder`   
     (`~motor2/absolute_encoder`)
      - Type:     `boolean`
      - Default:  imported from controller
  
      Set to `True` If motor uses absolute encoder, else set to `False`
  -  `~motor1/reverse_motor`   
     (`~motor2/reverse_motor`)
      - Type:     `boolean`
      - Default:  imported from controller
  
      Reverse the motor relative direction
  -  `~motor1/reverse_encoder`   
     (`~motor2/reverse_encoder`)
      - Type:     `boolean`
      - Default:  imported from controller
  
      Reverse the encoder relative direction
  -  `~motor1/position_pid/P`  
     (`~motor2/position_pid/P`)
      - Type:     `double`
      - Default:  imported from controller
  
      Propotional constant
  -  `~motor1/position_pid/I`  
     (`~motor2/position_pid/I`)
      - Type:     `double`
      - Default:  imported from controller
  
      Integral constant
  -  `~motor1/position_pid/D`  
     (`~motor2/position_pid/D`)
      - Type:     `double`
      - Default:  imported from controller
  
      Derivative constant
  -  `~motor1/position_pid/MaxI`  
     (`~motor2/position_pid/MaxI`)
      - Type:     `double`
      - Default:  imported from controller
  
      Maximum integral windup
  -  `~motor1/position_pid/Deadzone`  
     (`~motor2/position_pid/Deadzone`)
      - Type:     `double`
      - Default:  imported from controller
      - Unit:     `rad`
  
      Area around the specified position the controller will consider to be equal to the position specified
  -  `~motor1/position_pid/MinPos`  
     (`~motor2/position_pid/MinPos`)
      - Type:     `double`
      - Default:  imported from controller
      - Unit:     `rad`
  
      Minimum position
  -  `~motor1/position_pid/MaxPos`  
     (`~motor2/position_pid/MaxPos`)
      - Type:     `double`
      - Default:  imported from controller
      - Unit:     `rad`
  
      Maximum position
  -  `~motor1/velocity_pid/P`  
     (`~motor2/velocity_pid/P`)
      - Type:     `double`
      - Default:  imported from controller
  
      Propotional constant
  -  `~motor1/velocity_pid/I`  
     (`~motor2/velocity_pid/I`)
      - Type:     `double`
      - Default:  imported from controller
  
      Integral constant
  -  `~motor1/velocity_pid/D`  
     (`~motor2/velocity_pid/D`)
      - Type:     `double`
      - Default:  imported from controller
  
      Derivative constant
  -  `~motor1/velocity_pid/QPPS`  
     (`~motor2/velocity_pid/QPPS`)
      - Type:     `double`
      - Default:  imported from controller
      - Unit:     `rad/s`
  
      Speed of the motor when it is at 100% power

## Topics
### Published topics
- `~/motor1/joint_state`  
  `~/motor2/joint_state` [sirius_msgs/JointState]  
  Current motor position [*rad*], velocity [*rad/s*], acceleration [*rad/s<sup>2</sup>*], effort [*Nm*] and duty [*%*]

### Subscribed topics
- `~/motor1/set_joint_state`  
  `~/motor2/set_joint_state` [sirius_msgs/JointState]  
  Sets motor target position [*rad*], velocity [*rad/s*], acceleration [*rad/s<sup>2</sup>*], effort [*Nm*] and duty [*%*].   
  
    | Duty | Position | Velocity | Effort | Acceleration |  |
    |---|---|---|---|---|---|
    | ✔ | ❔ | ❔ | ❔ | ❔ | Drive motor with provided duty.    If the effort is provided, maximum motor currents will be set accordingly. Position, velocity and acceleration is ignored |
    | ❌ | ✔ | ❔ | ❔ | ❔ | The position PID controller is used. Position ramps with speed and acceleration If they are given. If the effort is provided,   maximum motor currents will be set accordingly |
    | ❌ | ❌ | ✔ | ❔ | ❔ | The velocity PID controller. Velocity ramps with acceleration If it is given. If the effort is provided, maximum motor currents will be set accordingly. |
    | ❌ | ❌ | ❌ | ✔ | ❔ | Duty cycle is set to 100% and motor max currents are set accordiingly. Acceleration is ignored. |   

    Legend:  
    ✔ - Value provided  
    ❌ - Value missing (Empty array `[]`)  
    ❔ - Optional field
- `~/motor1/add_encoder_offset`  
  `~/motor2/add_encoder_offset` [sirius_msgs/JointState]  
  Adds distance [*rad*] to the motor current position. It only changes value returned in topic `~/motor1/joint_state` or `~/motor2/joint_state`. You should use this topic i.e to reset odometry. If you want to set a target position for the motor then use `~/motor1/set_joint_state` or `~/motor2/set_joint_state` topic instead.
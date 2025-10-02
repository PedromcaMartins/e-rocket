# Before the first Tagus flight test

- [X] Setpoint receives 3d input (roll, pitch, yaw)
- [X] Bash script to convert ros2 bag into csv / plotting
    - Uses Plot Juggler
- [X] Adding sitl

- [?] Adding mocap package
- [?] Add yaw control to the controller
- [?] Adding motor allocator

- [X] Test abort command
- [X] Use frame conversions library instead of chatgpt code (quaternion_to_euler_radians)
    - Frame conversions library quaterion to euler does not work. Using Wikipedia formula instead. 
- [X] Increase publish message rate on px4

---

# Before the second Tagus flight test

- [X] MavLink forwarder through RPi
- [X] Improve PID Controller Class
- [X] Create Euler angles help functions
- [X] Create Allocation class
- [X] Removing inderect connection controller <-> px4 and controller <-> simulator
    - Transform message mappping to mocap forwarder
- [X] Fix mocap implementation (including test)
- [?] Refactor Mission to interact directly with px4
    - Remove flight mode
    -> Not possible! Flight Mode needs to be a seperate node! 
- [X] Setpoint to attitude degrees
- [X] Sin wave trajectory for attitude
- [X] Add position setpoint
- [X] Add position controller

# Before testing Andre's Controller

- [X] Add matrix library
    - Can Eigen3 be used?

## Mission

- [ ] Use timestamp from csv data and use it to wait for sending the endpoint (make the thread wait until the setpoint timestamp)

## Controller

Controller is generic, but Allocator is specific! receives forces and torques

- [X] Controller input - State

    ```
    struct State {
        Vehicle Attitude
        Rot. Matrix (q.to_rotation_matrix)
        Vel. Ang. Vehicle
        Veh local pos
        Veh odometry
    }
    ```

    - Setpoints exists in c5, and uses position instead of translation

- [?] Controller output - torques + forces

    ```
    struct Pre-Allocated?... {
        Torques
        Forces
    }
    ```

- [X] Allocator receives torques + forces -> Servo + Motor PWM

    ```
    struct Actuators?... {
        motorsPWM
        servosPWM
    }
    ```

- [ ] Integrator class 
- [ ] Reference frame transform class
    - .to_ned()
    - .to_enu()
    - ...

## New Controller-specific

- [X] Create a private fork to implement Andre's controller

## Flight Mode

- [X] Create a no-legs-only-rope-specific flight mode sequence... 
- [X] New leg-specific flight mode sequence
    - Pre-Arm
        - servo test
        - 4s to switch
    - Arm
        - Liga Motores (0% mby) / ramp-up
        - Save z (ground altitude)
        - Comando to switch?...
    - Take Off
        - Function that generates the trajectory using a transition function?... That thing used in animation
        - Uses controller, generates the setpoints
    - Mission
        - Sets position as origin, and uses it for the trajectory
        - Uses the compile-time trajectory generated using the python script
    - Landing
        - Function that uses the current xy position and sets target to arm z altitude
        - Uses another transition function - sends z, dz, ddz

## Hardware

- [X] Testing RC killswitch
- [ ] Build connector PX4 TELEM1 <-> RPI UART header

## Review

- [ ] Review of frame conversions / decoupled controllers
- [X] Removed all atomics from Ros2 nodes

---

# Nice-to-haves

- [ ] Heatsink RPi

- [ ] Implementation of coupled controller (Pedro Santos)
- [ ] Improve simulator to receive servos and motors and calculate the orientation, velocity, position, ... 

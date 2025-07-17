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
- [ ] Refactor Mission to interact directly with px4
    - Remove flight mode
- [ ] Add position setpoint
- [ ] Add position controller

- [ ] Testing RC killswitch
- [ ] Build connector PX4 TELEM1 <-> RPI UART header

- [ ] Review of frame conversions / decoupled controllers

---

# Nice-to-haves

- [ ] Heatsink RPi

- [ ] Implementation of coupled controller (Pedro Santos)
- [ ] Improve simulator to receive servos and motors and calculate the orientation, velocity, position, ... 

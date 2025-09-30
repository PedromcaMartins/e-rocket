float64[3] position      # meters
float64[3] velocity      # meters per second
float64[3] acceleration  # meters per second squared
float64[3] jerk          # meters per second cubed
float64[3] snap          # meters per second to the fourth

float64 yaw              # radians. If NAN, controller uses the current yaw
float64 yaw_rate         # radians per second
float64 yaw_acceleration # radians per second squared
import pandas as pd

import matplotlib.pyplot as plt

df = pd.read_csv('/home/pipes/e-rocket/Reports/Report 1/experiment/data.csv')

# Convert UNIX time to relative seconds for easier plotting
# df["time_s"] = df["__time"] - df["__time"].iloc[0]

# Select relevant columns
quaternion_cols = [
    "/fmu/out/vehicle_attitude/q[0]",
    "/fmu/out/vehicle_attitude/q[1]",
    "/fmu/out/vehicle_attitude/q[2]",
    "/fmu/out/vehicle_attitude/q[3]"
]
motor_cols = [
    "/fmu/in/actuator_motors/control[0]",
    "/fmu/in/actuator_motors/control[1]"
]
servo_cols = [
    "/fmu/in/actuator_servos/control[0]",
    "/fmu/in/actuator_servos/control[1]"
]

# Convert UNIX time to relative seconds for easier plotting
df["time_s"] = df["__time"] - df["__time"].iloc[0]

# Extract and drop rows with missing quaternion data
df_clean = df[["time_s"] + quaternion_cols + motor_cols + servo_cols]
df_quaternion = df_clean[["time_s"] + quaternion_cols].copy().dropna(subset=quaternion_cols)
df_motor = df_clean[["time_s"] + motor_cols].copy().dropna(subset=motor_cols)
df_servo = df_clean[["time_s"] + servo_cols].copy().dropna(subset=servo_cols)

# Plot quaternion components over time
plt.figure(figsize=(12, 6))
for q in quaternion_cols:
    plt.plot(df_quaternion["time_s"], df_quaternion[q], label=q.split("/")[-1])
plt.xlabel("Time (s)")
plt.ylabel("Quaternion Value")
plt.legend()
plt.grid(True)
plt.tight_layout()

# Plot motor[0] and motor[1]
plt.figure(figsize=(12, 6))
for motor in motor_cols:
    plt.plot(df_motor['time_s'], df_motor[motor], label=motor.split("/")[-1])
plt.title('Motor Controls [0] and [1]')
plt.xlabel('Time (s)')
plt.ylabel('Control Value')
plt.legend()
plt.grid(True)
plt.tight_layout()


# Plot servo[0] and servo[1]
plt.figure(figsize=(12, 6))
for i, servo in enumerate(servo_cols):
    label = "Pitch Servo" if i == 0 else "Roll Servo"
    plt.plot(df_servo['time_s'], df_servo[servo], label=label)
plt.xlabel('Time (s)')
plt.ylabel('PWM Value')
plt.legend()
plt.grid(True)
plt.tight_layout()

plt.show()

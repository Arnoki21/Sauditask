import math
class StabDrone:
    def __init__(self, kp, ki, kd):
        #PID regulators for each axes
        self.kp = kp
        self.ki = ki
        self.kd = kd
        #Errors for integral and diffirential parts
        self.prev_error = {'roll': 0, 'pitch': 0}
        self.integral_error = {'roll': 0, 'pitch': 0}
    def stabilizing(self, gyro_data, accel_data, dt):
        #Formulas of identifying deviations along roll and pitch axes
        roll_angle = math.atan2(accel_data['y'], accel_data['z'] * 180 / math.pi)
        pitch_angle = math.atan2(-accel_data['x'], math.sqrt(accel_data['y'] + accel_data['z'] ** 2)) * 180 / math.pi
        #Calculating of errors
        errors = {
            'roll': -roll_angle,
            'pitch': -pitch_angle
        }
        #PID regulator for each axes of drone
        motor_commands = {}
        for axis in ['roll', 'pitch']:
            self.integral_error[axis] += errors[axis] * dt
            der_error = (errors[axis] - self.prev_error[axis]) / dt
            motor_commands[axis] = (
                self.ki * self.integral_error[axis] +
                self.kp * errors[axis] +
                self.kd * der_error
            )
            self.prev_error[axis] = errors[axis]
        #Here we calculating commands for each motor in order to make drone stable
        #I have used 1000 as a basic power of motors, but it can be changed according to real power
        motor_commands['motor_1'] = 1000 + motor_commands['roll'] - motor_commands['pitch']
        motor_commands['motor_2'] = 1000 - motor_commands['roll'] - motor_commands['pitch']
        motor_commands['motor_3'] = 1000 - motor_commands['roll'] + motor_commands['pitch']
        motor_commands['motor_4'] = 1000 + motor_commands['roll'] + motor_commands['pitch']
        return motor_commands

#Simple example of usage
if __name__ == "__main__":
    #First we create ob PID
    stabilizer = StabDrone(kp=1.5, ki=0.1, kd=0.05)

    #Example input data
    gyro_data = {'x': 1.2, 'y': 0.8, 'z': -0.5}  #Gyroscope data in degrees
    accel_data = {'x': 0.05, 'y': -0.2, 'z': 0.95}  #Accelerometer data in 'g'
    dt = 0.01  #Time interval in secs
    #Calculating commands for motor
    motor_commands = stabilizer.stabilizing(gyro_data, accel_data, dt)
    #Result output
    print("Commands for motors:")
    for motor, command in motor_commands.items():
        print(f"{motor}: {command:.2f}")

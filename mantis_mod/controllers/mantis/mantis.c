from controller import Robot, Motor
import math
import sys

def main():
    robot = Robot()
    
    time_step = int(robot.getBasicTimeStep())
    
    # Meaning of the motor characters:
    # - 'R': Right / 'L': Left
    # - 'A': Front / 'M': Middle / 'P': Rear
    # - 'C': Base / 'F': Shoulder / 'T': Knee
    motor_names = [
        "RPC", "RPF", "RPT", "RMC", "RMF", "RMT",
        "RAC", "RAF", "RAT", "LPC", "LPF", "LPT",
        "LMC", "LMF", "LMT", "LAC", "LAF", "LAT"
    ]
    motors = [robot.getDevice(name) for name in motor_names]
    
    # Parameters for a simple walking gait (empirically found)
    f = 0.5  # frequency [Hz]
    
    # amplitude [rad]
    aC = 0.25  # Base motors
    aF = 0.2   # Shoulder motors
    aT = 0.05  # Knee motors
    a = [
        aC, aF, -aT, -aC, -aF, aT,
        aC, aF, -aT, aC, -aF, aT,
        -aC, aF, -aT, aC, -aF, aT
    ]
    
    # phase [s]
    pC = 0.0  # Base motors
    pF = 2.0  # Shoulder motors
    pT = 2.5  # Knee motors
    p = [
        pC, pF, pT, pC, pF, pT,
        pC, pF, pT, pC, pF, pT,
        pC, pF, pT, pC, pF, pT
    ]
    
    # offset [rad]
    dC = 0.6    # Base motors
    dF = 0.8    # Shoulder motors
    dT = -2.4   # Knee motors
    d = [
        -dC, dF, dT, 0.0, dF, dT,
        dC, dF, dT, dC, dF, dT,
        0.0, dF, dT, -dC, dF, dT
    ]
    
    while robot.step(time_step) != -1:
        time = robot.getTime()
        for i in range(18):  # Apply sinusoidal function to each motor
            motors[i].setPosition(a[i] * math.sin(2 * math.pi * f * time + p[i]) + d[i])
    
    sys.exit(0)

if __name__ == "__main__":
    main()
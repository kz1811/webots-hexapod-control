from controller import Robot, Motor, TouchSensor
import math
import numpy as np
import matplotlib.pyplot as plt

GAIT_SWITCH_TIME = 10.0     # Смена походки
END_SIM_TIME = 15.0         # Конец симуляции и рисование графиков
NEW_BASE_PHASES = [
    0.0,
    2 * np.pi / 3,
    4 * np.pi / 3,
    np.pi / 3,
    np.pi,
    5 * np.pi / 3
]
DELTA_FEMUR = 2.0
DELTA_TIBIA = 2.5

def main():
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())
    f = 0.5

    motor_names = [
        "RPC", "RPF", "RPT",
        "RMC", "RMF", "RMT",
        "RAC", "RAF", "RAT",
        "LPC", "LPF", "LPT",
        "LMC", "LMF", "LMT",
        "LAC", "LAF", "LAT"
    ]
    motors = {name: robot.getDevice(name) for name in motor_names}

    a = [
        0.25, 0.2, -0.05,
        -0.25, -0.2, 0.05,
        0.25, 0.2, -0.05,
        0.25, -0.2, 0.05,
        -0.25, 0.2, -0.05,
        0.25, -0.2, 0.05
    ]
    p = [0.0, 2.0, 2.5] * 6
    d = [
        -0.6, 0.8, -2.4,
        0.0, 0.8, -2.4,
        0.6, 0.8, -2.4,
        0.6, 0.8, -2.4,
        0.0, 0.8, -2.4,
        -0.6, 0.8, -2.4
    ]

    sensors = {}
    for name in motor_names:
        sensor = motors[name].getPositionSensor()
        sensor.enable(time_step)
        sensors[name] = sensor

    leg_sensors = {
        "RP": robot.getDevice("RPS"),
        "RM": robot.getDevice("RMS"),
        "RA": robot.getDevice("RAS"),
        "LP": robot.getDevice("LPS"),
        "LM": robot.getDevice("LMS"),
        "LA": robot.getDevice("LAS")
    }
    for s in leg_sensors.values():
        s.enable(time_step)

    legs = {
        "RP": [0, 1, 2], "RM": [3, 4, 5], "RA": [6, 7, 8],
        "LP": [9,10,11], "LM": [12,13,14], "LA": [15,16,17]
    }

    leg_keys = list(legs.keys())
    phase_updated = {leg: False for leg in leg_keys}

    pos_log = {i: [] for i in range(18)}
    signal_log = {i: [] for i in range(18)}
    time_log = []
    contact_log = []

    while robot.step(time_step) != -1:
        t = robot.getTime()
        time_log.append(t)
        contact_snapshot = {}

        for leg, sensor in leg_sensors.items():
            contact = 1 if sensor.getValue() > 0.01 else 0
            contact_snapshot[leg] = contact

        contact_log.append(contact_snapshot.copy())

        if t >= GAIT_SWITCH_TIME:
            for i, leg in enumerate(leg_keys):
                if not phase_updated[leg] and contact_snapshot[leg] == 0:
                    idxs = legs[leg]
                    p[idxs[0]] = NEW_BASE_PHASES[i]
                    p[idxs[1]] = NEW_BASE_PHASES[i] + DELTA_FEMUR
                    p[idxs[2]] = NEW_BASE_PHASES[i] + DELTA_TIBIA
                    phase_updated[leg] = True

        for i in range(18):
            angle = a[i] * math.sin(2 * math.pi * f * t + p[i]) + d[i]
            motors[motor_names[i]].setPosition(angle)
            pos_log[i].append(sensors[motor_names[i]].getValue())
            signal_log[i].append(angle)

        if t > END_SIM_TIME:
            break

    #  Правая средняя нога
    labels = ["RMC", "RMF", "RMT"]
    idxs = [3, 4, 5]
    leg_key = "RM"

    for i, label in zip(idxs, labels):
        plt.figure()
        xs, ys = [], []
        contact_series = [frame[leg_key] for frame in contact_log]
        current_state = contact_series[0]
        label_used = {0: False, 1: False}

        for t_val, y_val, contact in zip(time_log, pos_log[i], contact_series):
            if contact != current_state:
                plt.plot(xs, ys, color='green' if current_state else 'red',
                         label='Опора' if current_state and not label_used[1] else 'Перенос' if not current_state and not label_used[0] else None)
                label_used[current_state] = True
                xs, ys = [], []
                current_state = contact
            xs.append(t_val)
            ys.append(y_val)

        if xs:
            plt.plot(xs, ys, color='green' if current_state else 'red',
                     label='Опора' if current_state and not label_used[1] else 'Перенос' if not current_state and not label_used[0] else None)

        plt.plot(time_log[:len(signal_log[i])], signal_log[i], label="Управляющий сигнал", linestyle='--', color='gray')
        plt.xlabel("Время [с]")
        plt.ylabel("Положение")
        plt.title(f"Смена походки — {label}")
        plt.legend()
        plt.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()


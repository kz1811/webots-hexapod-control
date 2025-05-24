from controller import Robot, Motor, TouchSensor
import math
import numpy as np
import matplotlib.pyplot as plt

# Константы настройки адаптации
CALIBRATION_DURATION = 5.0              # Время сбора данных для калибровки
ADAPTATION_START = CALIBRATION_DURATION # Начало периода адаптации
THRESHOLD = 0.1                         # Порог чувствительности отклонения
RECOVERY_RATE = 0.01                    # Скорость возврата к целевой амплитуде
ADAPTATION_RATE = 0.02                  # Скорость адаптации амплитуды
MAX_AMPLITUDE = 0.4                     # Максимальная амплитуда
MIN_AMPLITUDE = 0.1                     # Минимальная амплитуда
FINAL_TIME = 15.0                       # Время рисования графиков

def plot_adaptive_leg_results(time_series, amp_log, pos_log, target_log, signal_log, contact_log):
    #  Правая передняя нога - 3 мотора
    motors = ['RAC', 'RAF', 'RAT']
    ids = [6, 7, 8]
    for i, label in zip(ids, motors):
        leg_key = label[:2]
        plt.figure()
        xs, ys = [], []
        color_state = contact_log[0][leg_key]
        label_used = {0: False, 1: False}
        for t, y, contact in zip(time_series, pos_log[i], [c[leg_key] for c in contact_log]):
            if contact != color_state:
                plt.plot(xs, ys, color='green' if color_state else 'red',
                         label='Опора' if color_state and not label_used[1] else 'Перенос' if not color_state and not label_used[0] else None)
                label_used[color_state] = True
                xs, ys = [], []
                color_state = contact
            xs.append(t)
            ys.append(y)
        if xs:
            plt.plot(xs, ys, color='green' if color_state else 'red',
                     label='Опора' if color_state and not label_used[1] else 'Перенос' if not color_state and not label_used[0] else None)

        plt.plot(time_series[:len(signal_log[i])], signal_log[i], label="Задаваемый сигнал", linestyle='--', color='gray')
        plt.xlabel("Время [с]")
        plt.ylabel("Положение")
        plt.title(f"Движение - {label}")
        plt.legend()
        plt.grid(True)

        plt.figure()
        plt.plot(time_series[:len(amp_log[i])], amp_log[i], label="Фактическая амплитуда", color='blue')
        plt.plot(time_series[:len(target_log[i])], target_log[i], label="Целевая амплитуда", linestyle='--', color='gray')
        plt.xlabel("Время [с]")
        plt.ylabel("Амплитуда")
        plt.title(f"Адаптация амплитуды - {label}")
        plt.legend()
        plt.grid(True)

    plt.tight_layout()
    plt.show()

def main():
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())
    f = 0.5  # Частота шага

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
    p = [0.0, 2.0, 2.5,
         0.0, 2.0, 2.5,
         0.0, 2.0, 2.5,
         0.0, 2.0, 2.5,
         0.0, 2.0, 2.5,
         0.0, 2.0, 2.5]
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

    adaptive_amplitudes = {i: a[i] for i in range(18)}
    target_amplitudes = {i: a[i] for i in range(18)}

    stance_memory = {leg: [] for leg in legs}
    amp_log = {i: [] for i in range(18)}
    pos_log = {i: [] for i in range(18)}
    target_log = {i: [] for i in range(18)}
    signal_log = {i: [] for i in range(18)}
    time_log = []
    contact_log = []

    phase_time = 1.0 / f
    phase_timer = 0.0
    phase_buffer = {leg: [] for leg in legs}

    while robot.step(time_step) != -1:
        t = robot.getTime()
        time_log.append(t)
        contact_snapshot = {}

        for leg, sensor in leg_sensors.items():
            contact = 1 if sensor.getValue() > 0.01 else 0
            contact_snapshot[leg] = contact
            phase_buffer[leg].append(contact)

        contact_log.append(contact_snapshot.copy())
        phase_timer += time_step / 1000.0

        if phase_timer >= phase_time:
            for leg in legs:
                phase = phase_buffer[leg]
                ratio = sum(phase) / len(phase)
                stance_memory[leg].append(ratio)
                if len(stance_memory[leg]) > 30:
                    stance_memory[leg].pop(0)

                if t >= ADAPTATION_START:
                    avg = np.mean(stance_memory[leg])
                    for idx in legs[leg]:
                        if ratio < avg - THRESHOLD:
                            adaptive_amplitudes[idx] = min(MAX_AMPLITUDE, adaptive_amplitudes[idx] + ADAPTATION_RATE)
                        elif ratio > avg + THRESHOLD:
                            adaptive_amplitudes[idx] = max(MIN_AMPLITUDE, adaptive_amplitudes[idx] - ADAPTATION_RATE)
                        else:
                            ta = target_amplitudes[idx]
                            if adaptive_amplitudes[idx] < ta:
                                adaptive_amplitudes[idx] = min(ta, adaptive_amplitudes[idx] + RECOVERY_RATE)
                            elif adaptive_amplitudes[idx] > ta:
                                adaptive_amplitudes[idx] = max(ta, adaptive_amplitudes[idx] - RECOVERY_RATE)
            phase_buffer = {leg: [] for leg in legs}
            phase_timer = 0.0

        for i in range(18):
            amp = adaptive_amplitudes[i]
            angle = amp * math.sin(2 * math.pi * f * t + p[i]) + d[i]
            motors[motor_names[i]].setPosition(angle)
            amp_log[i].append(amp)
            pos_log[i].append(sensors[motor_names[i]].getValue())
            signal_log[i].append(angle)
            target_log[i].append(target_amplitudes[i])

        if t > FINAL_TIME:
            break


    plot_adaptive_leg_results(time_log, amp_log, pos_log, target_log, signal_log, contact_log)

if __name__ == "__main__":
    main()

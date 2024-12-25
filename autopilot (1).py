import krpc
import time
import csv
from math import sqrt
import numpy as np
import pathlib

# Параметры миссии
turn_start_altitude = 250  # Начало гравитационного поворота
turn_end_altitude = 45000  # Завершение поворота
target_altitude = 265000  # Целевая высота орбиты
min_periapsis = 135000  # Минимальная высота перигея (80 км)

# Контрольные высоты
target_altitude_las = 80660  # Сброс системы аварийного спасения
target_altitude_core_stage = 82000  # Сброс центральной ступени

# Подключение к kRPC
conn = krpc.connect(name='Artemis-1 Launch')
vessel = conn.space_center.active_vessel

# Настройка потоков телеметрии
ut = conn.add_stream(getattr, conn.space_center, 'ut')
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
periapsis = conn.add_stream(getattr, vessel.orbit, 'periapsis_altitude')

# Создаем файл для записи данных
PATH = str(pathlib.Path(__file__).parent.joinpath("ksp_flight_data.csv"))
with open(PATH, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Time", "Altitude", "Vertical Velocity", "Horizontal Velocity",
                     "Total Velocity", "Drag", "Displacement", "Engine Thrust", "Vacuum Thrust", "Air Temperature"])

    # Подготовка к запуску
    vessel.control.throttle = 1.0
    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_pitch_and_heading(90, 90)
    vessel.auto_pilot.target_roll = 0

    # Начальная позиция для расчета смещения
    start_time = conn.space_center.ut
    initial_position = vessel.position(vessel.orbit.body.reference_frame)
    initial_position_vec_length = np.linalg.norm(initial_position)

    print('3...')
    time.sleep(1)
    print('2...')
    time.sleep(1)
    print('1...')
    time.sleep(1)
    print('Пуск!')

    vessel.control.activate_next_stage()

    # Получение начальной тяги двигателя
    initial_thrust = vessel.thrust  # Текущая тяга
    vacuum_thrust = sum(
        [engine.max_vacuum_thrust for engine in vessel.parts.engines])  # Тяга в вакууме
    print(f"Начальная тяга: {initial_thrust} Н, Тяга в вакууме: {vacuum_thrust} Н")


    # Основной цикл подъема
    turn_angle = 0
    srb_dropped = False
    las_dropped = False
    core_stage_dropped = False
    tli_transfer = False

    while True:
        # Настоящее время
        ut = conn.space_center.ut
        elapsed_time = ut - start_time

        # Гравитационный поворот
        if turn_start_altitude < altitude() < turn_end_altitude:
            frac = ((altitude() - turn_start_altitude) /
                    (turn_end_altitude - turn_start_altitude))
            new_turn_angle = frac * 90
            if abs(new_turn_angle - turn_angle) > 0.5:
                turn_angle = new_turn_angle
                vessel.auto_pilot.target_pitch_and_heading(
                    90 - (turn_angle), 90)

        if not srb_dropped and vessel.resources.amount('SolidFuel') < 400:
            srb_dropped = True
            vessel.control.activate_next_stage()
            print("Сброс ускорителей")
            print(vessel.flight().static_air_temperature)

        # Температура воздуха на текущей высоте (если доступно)
        try:
            air_temperature = vessel.flight().static_air_temperature
        except AttributeError:
            air_temperature = None  # Если температура недоступна

        # Отделение ступени аварийного спасения
        if target_altitude_las <= altitude() < target_altitude_las + 1000:
            vessel.control.activate_next_stage()
            time.sleep(1)
            vessel.control.activate_next_stage()
            print("Сброс системы аварийного спасения (LAS)")
            time.sleep(1)

        # Отделение центральной ступени
        if target_altitude_core_stage <= altitude() < (target_altitude_core_stage + 3000):
            vessel.control.throttle = 0
            time.sleep(1)
            vessel.control.activate_next_stage()  # Отделяем ступень
            print("Отделение центральной ступени")

            # Ищем и активируем двигатель
            eisorau_engine_found = False
            for engine in vessel.parts.engines:
                if 'Inon-R-10B2 "Eisorau" Cryogenic Engine' in engine.part.title:
                    engine.active = True  # Включение двигателя
                    eisorau_engine_found = True
                    print(f"{engine.part.title} активирован")
                    break

            if not eisorau_engine_found:
                print("Ошибка: двигатель 'Inon-R-10B2' не найден")

            # Включаем тягу
            vessel.control.throttle = 1
            time.sleep(2)
            vessel.control.throttle = 0
            time.sleep(210)
            

        if vessel.orbit.time_to_apoapsis < 15:
            vessel.control.throttle = 1

        # Запись данных в файл
        altitude_val = altitude()
        speed = vessel.flight(vessel.orbit.body.reference_frame).speed
        drag_x, drag_y, drag_z = vessel.flight().drag
        drag = sqrt(drag_x ** 2 + drag_y ** 2 + drag_z ** 2)
        current_position = vessel.position(vessel.orbit.body.reference_frame)
        current_position = current_position / \
            np.linalg.norm(current_position) * initial_position_vec_length
        horizontal_displacement = np.linalg.norm(
            current_position - initial_position)
        vertical_speed = vessel.flight(
            vessel.orbit.body.reference_frame).vertical_speed
        horizontal_speed = vessel.flight(
            vessel.orbit.body.reference_frame).horizontal_speed

        # Получение текущей тяги двигателя
        current_thrust = vessel.thrust

        writer.writerow([elapsed_time, altitude_val, vertical_speed, horizontal_speed, speed, drag,
                         horizontal_displacement, current_thrust, vacuum_thrust, air_temperature])

        # Достижение целевых орбитальных параметров
        if not tli_transfer and periapsis() > min_periapsis:
            tli_transfer = True
            vessel.control.throttle = 0
            # vessel.control.activate_next_stage()
            # time.sleep(1)
            # vessel.control.activate_next_stage()
            # time.sleep(1)

            print("Орбита достигнута. Завершение работы.")
            break

        time.sleep(0.1)
    
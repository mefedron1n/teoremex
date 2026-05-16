# main.py
import pybullet as p
import pybullet_data
import time
import math
from keyboard import get_keyboard_input, print_controls, FlightState

# Начальная ориентация модели (переворот на 180° по тангажу)
INITIAL_ORIENTATION = [0, math.sin(math.pi / 2), 0, math.cos(math.pi / 2)]

# ============================================================================
# ИНИЦИАЛИЗАЦИЯ СИМУЛЯЦИИ
# ============================================================================

# Создаем окно симуляции PyBullet
# p.GUI открывает графический интерфейс, p.DIRECT работает без окна
physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)  # Гравитация направлена вниз (ось Z)
p.loadURDF("plane.urdf")  # Загружаем статическую плоскость (земля)

# Загружаем модель ракеты (цель для перехвата)вв
# Ракета будет лететь прямо, управляемая клавишами
# Переворачиваем модель на 180° по тангажу (ось Y)
rocket_urdf = p.loadURDF("rocket.urdf", basePosition=[-5, 0, 1], baseOrientation=INITIAL_ORIENTATION, flags=p.URDF_USE_SELF_COLLISION)

# Загружаем модель дрона (перехватчик)
# Дрон будет автоматически наводиться на ракету по формуле наведения
drone_urdf = p.loadURDF("drone.urdf", basePosition=[0, 0, 1], flags=p.URDF_USE_SELF_COLLISION)

# Отключаем встроенное демпфирование PyBullet для обоих объектов
p.changeDynamics(rocket_urdf, -1, linearDamping=0.0, angularDamping=0.0)
p.changeDynamics(drone_urdf, -1, linearDamping=0.0, angularDamping=0.0)

# Назначаем роли:
# plane - управляемая цель (ракета в URDF, но роль "самолета")
# rocket - перехватчик (дрон в URDF, но роль "ракеты")
plane = rocket_urdf
rocket = drone_urdf

rocket_mass = 1  # Масса перехватчика для расчета сил

# ============================================================================
# ПАРАМЕТРЫ ДВИЖЕНИЯ
# ============================================================================

# Сила тяги управляемой ракеты (цели)
plane_thrust = 50.0  # Ньютоны

# Сила тяги перехватчика (дрона)
drone_thrust = 80.0  # Ньютоны

# Масса объектов (кг)
plane_mass = 1.0
rocket_mass = 1.0

# Коэффициент демпфирования скорости (сопротивление воздуха)
damping_coefficient = 2.0

# ============================================================================
# СЧЕТЧИКИ ДЛЯ ОТЛАДКИ
# ============================================================================

step_count = 0          # Номер текущего шага симуляции
simulation_time = 0.0   # Прошедшее время симуляции в секундах

# Вывод информации об управлении в консоль
print_controls()

# ============================================================================
# ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
# ============================================================================

def euler_to_quaternion(roll, pitch, yaw):
    """
    Преобразование углов Эйлера в кватернион.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy
    ]


# ============================================================================
# ОСНОВНОЙ ЦИКЛ СИМУЛЯЦИИ
# ============================================================================

try:
    # Создаем объект для хранения состояния полета (углы тангажа и рыскания)
    flight_state = FlightState()

    while True:
        # ====================================================================
        # ШАГ 1: ПОЛУЧЕНИЕ ТЕКУЩИХ ПОЗИЦИЙ И СКОРОСТЕЙ
        # ====================================================================

        pos_plane, _ = p.getBasePositionAndOrientation(plane)
        pos_rocket, rocket_orientation = p.getBasePositionAndOrientation(rocket)

        # Получаем текущие скорости
        vel_plane, _ = p.getBaseVelocity(plane)
        vel_rocket, _ = p.getBaseVelocity(rocket)

        # ====================================================================
        # ШАГ 2: ОБРАБОТКА ВВОДА И ПРИМЕНЕНИЕ СИЛЫ К САМОЛЕТУ (ЦЕЛИ)
        # ====================================================================

        # Получаем единичный вектор направления на основе углов
        direction = get_keyboard_input(flight_state)

        # Преобразуем углы в кватернион для ориентации
        plane_orientation = euler_to_quaternion(
            roll=flight_state.roll,
            pitch=flight_state.pitch,
            yaw=flight_state.yaw
        )
        p.resetBasePositionAndOrientation(plane, pos_plane, plane_orientation)

        # Сила тяги в направлении полета
        thrust_force = [
            plane_thrust * direction[0],
            plane_thrust * direction[1],
            plane_thrust * direction[2]
        ]

        # Сила сопротивления воздуха (демпфирование)
        # Направлена против скорости, стабилизирует движение
        damping_force = [
            -damping_coefficient * vel_plane[0],
            -damping_coefficient * vel_plane[1],
            -damping_coefficient * vel_plane[2]
        ]

        # Суммарная сила: тяга + демпфирование
        total_force = [
            thrust_force[0] + damping_force[0],
            thrust_force[1] + damping_force[1],
            thrust_force[2] + damping_force[2]
        ]

        p.applyExternalForce(
            objectUniqueId=plane,
            linkIndex=-1,
            forceObj=total_force,
            posObj=[0, 0, 0],
            flags=p.WORLD_FRAME
        )

        # ====================================================================
        # ШАГ 3: НАВЕДЕНИЕ РАКЕТЫ НА ЦЕЛЬ ЧЕРЕЗ СИЛУ
        # ====================================================================

        # Вектор от ракеты к цели
        delta_x = pos_plane[0] - pos_rocket[0]
        delta_y = pos_plane[1] - pos_rocket[1]
        delta_z = pos_plane[2] - pos_rocket[2]
        distance = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

        # Направляющая сила к цели (всегда активна)
        if distance > 0.01:
            guidance_force = [
                drone_thrust * delta_x / distance,
                drone_thrust * delta_y / distance,
                drone_thrust * delta_z / distance
            ]

            # Добавляем демпфирование к ракете
            damping_rocket = [
                -damping_coefficient * vel_rocket[0],
                -damping_coefficient * vel_rocket[1],
                -damping_coefficient * vel_rocket[2]
            ]

            total_rocket_force = [
                guidance_force[0] + damping_rocket[0],
                guidance_force[1] + damping_rocket[1],
                guidance_force[2] + damping_rocket[2]
            ]

            p.applyExternalForce(
                objectUniqueId=rocket,
                linkIndex=-1,
                forceObj=total_rocket_force,
                posObj=[0, 0, 0],
                flags=p.WORLD_FRAME
            )

        # Фиксируем ориентацию ракеты (не даем вращаться)
        # Сохраняем текущую линейную скорость!
        p.resetBasePositionAndOrientation(rocket, pos_rocket, rocket_orientation)
        p.resetBaseVelocity(rocket, linearVelocity=vel_rocket, angularVelocity=[0, 0, 0])

        # ====================================================================
        # ШАГ 4: ПРОВЕРКА СТОЛКНОВЕНИЯ
        # ====================================================================

        # Вычисляем евклидово расстояние между ракетой и целью
        distance = math.sqrt(
            (pos_rocket[0] - pos_plane[0])**2 +
            (pos_rocket[1] - pos_plane[1])**2 +
            (pos_rocket[2] - pos_plane[2])**2
        )

        # Если расстояние меньше порога - попадание
        if distance < 0.3:
            print("\n🎯 ПОПАДАНИЕ!")
            # Небольшая задержка для визуального эффекта
            for _ in range(10):
                p.stepSimulation()
                time.sleep(0.05)
            break

        # ====================================================================
        # ШАГ 5: ВЫВОД ОТЛАДОЧНОЙ ИНФОРМАЦИИ
        # ====================================================================

        # Выводим информацию каждые 50 кадров
        if step_count % 50 == 0:
            # Получаем текущие углы самолета в градусах
            pitch_deg, roll_deg, yaw_deg = flight_state.get_display_angles()

            # Получаем текущие скорости для отображения
            vel_plane, _ = p.getBaseVelocity(plane)
            velocity_magnitude = math.sqrt(
                vel_plane[0]**2 + vel_plane[1]**2 + vel_plane[2]**2
            )

            print(f"Время: {simulation_time:5.1f}c | "
                  f"Дист: {distance:5.1f}м | "
                  f"Скорость: {velocity_magnitude:5.1f} м/с | "
                  f"Тангаж: {pitch_deg:6.1f}° | "
                  f"Рыскание: {yaw_deg:6.1f}°")

        # ====================================================================
        # ШАГ СИМУЛЯЦИИ
        # ====================================================================
        
        # stepSimulation продвигает физику на один шаг
        # По умолчанию шаг = 1/60 секунды
        p.stepSimulation()
        
        # Ограничиваем частоту кадров до 60 FPS
        time.sleep(1/60)
        
        # Обновляем счетчики
        simulation_time += 1/60
        step_count += 1

except KeyboardInterrupt:
    print("\n\n🛑 Остановка симуляции пользователем")

finally:
    # Отключаемся от сервера физики
    p.disconnect()
    print("✅ Симуляция завершена")

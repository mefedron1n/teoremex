# main.py
import pybullet as p
import pybullet_data
import time
import numpy as np
from keyboard import get_keyboard_input, print_controls, FlightState

# Инициализация PyBullet
physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)  # Отключаем гравитацию
p.loadURDF("plane.urdf")

# Загружаем модель ракеты (вместо самолета)
rocket_urdf = p.loadURDF("rocket.urdf", basePosition=[-5, 0, 1])

# Загружаем модель дрона (вместо ракеты)
drone_urdf = p.loadURDF("drone.urdf", basePosition=[0, 0, 1])

plane = rocket_urdf
rocket = drone_urdf

rocket_mass = 1

# Параметры управления ракетой
plane_speed = 1.5
flight_state = FlightState()

# Хранение истории для вычисления производных
history_size = 20  # Количество сохраняемых точек
time_history = []
pos_x_history = []
pos_y_history = []
pos_z_history = []

# Для отображения информации о силе
step_count = 0
simulation_time = 0

# Вывод информации об управлении
print_controls()

def calculate_force_from_trajectory(positions_x, positions_y, positions_z, time_points, mass):
    """
    Вычисление силы на основе второй производной (ускорения)
    
    Args:
        positions_x, positions_y, positions_z: списки координат
        time_points: список моментов времени
        mass: масса объекта
    
    Returns:
        tuple: (force_x, force_y, force_z) - компоненты силы
    """
    if len(time_points) < 3:
        return 0, 0, 0
    
    # Преобразуем в массивы numpy
    t = np.array(time_points)
    x = np.array(positions_x)
    y = np.array(positions_y)
    z = np.array(positions_z)
    
    # Первая производная (скорость)
    vx = np.gradient(x, t)
    vy = np.gradient(y, t)
    vz = np.gradient(z, t)
    
    # Вторая производная (ускорение)
    ax = np.gradient(vx, t)
    ay = np.gradient(vy, t)
    az = np.gradient(vz, t)
    
    # Берем последние значения ускорения
    if len(ax) > 0:
        last_ax = ax[-1]
        last_ay = ay[-1]
        last_az = az[-1]
    else:
        last_ax, last_ay, last_az = 0, 0, 0
    
    # Сила = масса * ускорение (второй закон Ньютона)
    force_x = mass * last_ax
    force_y = mass * last_ay
    force_z = mass * last_az
    
    # Ограничиваем силу, чтобы избежать слишком больших значений
    max_force = 50.0
    force_x = np.clip(force_x, -max_force, max_force)
    force_y = np.clip(force_y, -max_force, max_force)
    force_z = np.clip(force_z, -max_force, max_force)
    
    return force_x, force_y, force_z

def calculate_desired_velocity(rocket_pos, target_pos, base_speed=3.0):
    """
    Расчет желаемой скорости для движения к цели
    
    Args:
        rocket_pos: позиция ракеты
        target_pos: позиция цели
        base_speed: базовая скорость
    
    Returns:
        list: желаемая скорость [vx, vy, vz]
    """
    delta_x = target_pos[0] - rocket_pos[0]
    delta_y = target_pos[1] - rocket_pos[1]
    delta_z = target_pos[2] - rocket_pos[2]
    
    distance = (delta_x**2 + delta_y**2 + delta_z**2)**0.5
    
    if distance > 0:
        velocity = [
            base_speed * delta_x / distance,
            base_speed * delta_y / distance,
            base_speed * delta_z / distance
        ]
    else:
        velocity = [0, 0, 0]

    return velocity

# Основной цикл симуляции
try:
    while True:
        # 1. ЛОГИКА ДВИЖЕНИЯ САМОЛЕТА
        direction = get_keyboard_input(flight_state)

        plane_velocity = [
            direction[0] * plane_speed,
            direction[1] * plane_speed,
            direction[2] * plane_speed
        ]
        p.resetBaseVelocity(plane, linearVelocity=plane_velocity)
        
        # 2. ПОЛУЧЕНИЕ ПОЗИЦИЙ
        pos_rocket, _ = p.getBasePositionAndOrientation(rocket)
        pos_plane, _ = p.getBasePositionAndOrientation(plane)

        # Направление к цели
        direction_to_target = np.array(pos_plane) - np.array(pos_rocket)

        distance = np.linalg.norm(direction_to_target)

        if distance > 0.001:
            direction_to_target = direction_to_target / distance

        # Текущая скорость ракеты
        current_velocity, _ = p.getBaseVelocity(rocket)
        current_velocity = np.array(current_velocity)

        # Желаемая скорость
        target_speed = 3.0
        desired_velocity = direction_to_target * target_speed

        # Плавное наведение (сглаживание)
        steering_strength = 0.08

        new_velocity = (
                current_velocity * (1.0 - steering_strength)
                + desired_velocity * steering_strength
        )

        # Ограничение максимальной скорости
        max_speed = 4.0

        speed = np.linalg.norm(new_velocity)

        if speed > max_speed:
            new_velocity = new_velocity / speed * max_speed

        # Устанавливаем новую скорость
        p.resetBaseVelocity(
            rocket,
            linearVelocity=new_velocity.tolist()
        )
        
        # 5. ДОПОЛНИТЕЛЬНО: устанавливаем желаемую скорость для наведения
        
        # 6. ПРОВЕРКА СТОЛКНОВЕНИЯ
        distance = ((pos_rocket[0] - pos_plane[0])**2 + 
                   (pos_rocket[1] - pos_plane[1])**2 + 
                   (pos_rocket[2] - pos_plane[2])**2)**0.5
        
        if distance < 0.3:
            # Визуальный эффект попадания
            for _ in range(10):
                p.stepSimulation()
                time.sleep(0.05)
            break
        
        # 7. ВЫВОД ИНФОРМАЦИИ (каждые 50 шагов)
        # 7. ВЫВОД ИНФОРМАЦИИ
        if step_count % 50 == 0:
            current_speed = np.linalg.norm(new_velocity)

            print(
                f"Время: {simulation_time:.2f}c | "
                f"Дист: {distance:.2f}м | "
                f"Скорость ракеты: {current_speed:.2f} м/с"
            )
        
        # Шаг симуляции
        p.stepSimulation()
        time.sleep(1/60)
        simulation_time += 1/60
        step_count += 1

except KeyboardInterrupt:
    print("\n\n🛑 Остановка симуляции пользователем")

finally:
    p.disconnect()
    print("✅ Симуляция завершена")
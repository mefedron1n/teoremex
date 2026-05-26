# keyboard.py
import pybullet as p


class FlightState:
    """
    Состояние полета самолета.

    Хранит текущие углы тангажа (pitch), крена (roll) и рыскания (yaw), которые
    изменяются клавишами управления. Эти углы определяют направление
    движения самолета при постоянной скорости.

    Attributes:
        pitch: угол тангажа в радианах (без ограничений)
               положительный = нос вверх, отрицательный = нос вниз
        roll: угол крена в радианах (без ограничений)
              положительный = наклон вправо, отрицательный = наклон влево
        yaw: угол рыскания в радианах (без ограничений)
             положительный = поворот вправо, отрицательный = поворот влево
    """

    # Скорость изменения углов (радиан за кадр)
    PITCH_RATE = 0.03
    ROLL_RATE = 0.04
    YAW_RATE = 0.03

    def __init__(self):
        """Инициализация состояния - самолет летит горизонтально."""
        self.pitch = 0.0
        self.roll = 0.0  # Крен отключен, всегда 0
        self.yaw = 0.0

    def update(self, keys):
        """
        Обновление углов тангажа, крена и рыскания на основе нажатых клавиш.

        Как это работает:
        1. Проверяем, нажата ли клавиша управления тангажем (U/J)
        2. Если да - увеличиваем/уменьшаем угол pitch
        3. То же самое для крена (H/K) и угла roll
        4. То же самое для рыскания (Y/I) и угла yaw
        5. Если клавиши не нажаты - углы не меняются (самолет сохраняет положение)

        Args:
            keys: словарь нажатых клавиш от p.getKeyboardEvents()
        """
        # Управление тангажем (pitch) - нос вверх/вниз
        # U = нос вверх (увеличиваем pitch)
        # J = нос вниз (уменьшаем pitch)
        if ord('u') in keys and keys[ord('u')] & p.KEY_IS_DOWN:
            self.pitch += self.PITCH_RATE
        elif ord('j') in keys and keys[ord('j')] & p.KEY_IS_DOWN:
            self.pitch -= self.PITCH_RATE

        # Управление рысканием (yaw) - поворот влево/вправо
        # H = поворот влево (уменьшаем yaw, отрицательный)
        # K = поворот вправо (увеличиваем yaw, положительный)

        if ord('h') in keys and keys[ord('h')] & p.KEY_IS_DOWN:
            self.yaw += self.YAW_RATE
        elif ord('k') in keys and keys[ord('k')] & p.KEY_IS_DOWN:
            self.yaw -= self.YAW_RATE

    def get_direction_vector(self):
        """
        Преобразование углов в единичный вектор направления.

        Как это работает:
        1. Углы pitch и yaw определяют направление "вперед"
        2. Базовое направление без углов: [-1, 0, 0] (вдоль отрицательной оси X)
        3. Применяем вращения в порядке: yaw (Z) -> pitch (Y)

        Используем кватернион для вращения базового направления [-1, 0, 0].

        Args:
            None

        Returns:
            list: единичный вектор направления [dx, dy, dz]
        """
        import math

        # Половины углов для кватерниона
        sy = math.sin(self.yaw * 0.5)
        cy = math.cos(self.yaw * 0.5)
        sp = math.sin(self.pitch * 0.5)
        cp = math.cos(self.pitch * 0.5)
        sr = math.sin(self.roll * 0.5)
        cr = math.cos(self.roll * 0.5)

        # Компоненты кватерниона (порядок ZYX: yaw -> pitch -> roll)
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        # Вращаем базовое направление [-1, 0, 0] кватернионом
        dx = -(qw * qw + qx * qx - qy * qy - qz * qz)
        dy = 2 * (-(qx * qy + qz * qw))
        dz = 2 * (-(qy * qw - qx * qz))

        return [dx, dy, dz]

    def get_display_angles(self):
        """
        Получение углов в градусах для отображения.

        Returns:
            tuple: (pitch_degrees, roll_degrees, yaw_degrees)
        """
        import math
        return (math.degrees(self.pitch), math.degrees(self.roll), math.degrees(self.yaw))


def get_keyboard_input(flight_state):
    """
    Получение вектора направления для самолета на основе ввода с клавиатуры.

    Как это работает:
    1. Считываем текущие нажатия клавиш
    2. Обновляем углы тангажа и рыскания в flight_state
    3. Преобразуем углы в единичный вектор направления

    Args:
        flight_state: объект FlightState, хранящий текущие углы

    Returns:
        list: единичный вектор направления [dx, dy, dz]
    """
    keys = p.getKeyboardEvents()

    # Обновляем углы на основе нажатых клавиш
    flight_state.update(keys)

    # Преобразуем углы в вектор направления
    return flight_state.get_direction_vector()


def print_controls():
    """Вывод информации об управлении."""
    print("\n" + "=" * 50)
    print("УПРАВЛЕНИЕ САМОЛЕТОМ:")
    print("=" * 50)
    print("U - тангаж вверх (нос вверх, набор высоты)")
    print("J - тангаж вниз (нос вниз, снижение)")
    print("H - рыскание влево (поворот влево)")
    print("K - рыскание вправо (поворот вправо)")
    print("-" * 50)
    print("Самолет летит с ПОСТОЯННОЙ скоростью.")
    print("Клавиши изменяют только направление (углы).")
    print("-" * 50)
    print("Для выхода нажмите Ctrl+C")
    print("=" * 50 + "\n")


def get_simple_input(speed=2.0):
    """
    Альтернативное управление через WASD + Q/E.

    Примечание: эта функция не используется в новой системе,
    но оставлена для совместимости.

    Args:
        speed: скорость перемещения

    Returns:
        list: вектор скорости [vx, vy, vz]
    """
    keys = p.getKeyboardEvents()
    velocity = [0, 0, 0]

    if ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN:
        velocity[0] = -speed
    if ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
        velocity[0] = speed
    if ord('w') in keys and keys[ord('w')] & p.KEY_IS_DOWN:
        velocity[1] = speed
    if ord('s') in keys and keys[ord('s')] & p.KEY_IS_DOWN:
        velocity[1] = -speed
    if ord('q') in keys and keys[ord('q')] & p.KEY_IS_DOWN:
        velocity[2] = speed
    if ord('e') in keys and keys[ord('e')] & p.KEY_IS_DOWN:
        velocity[2] = -speed

    return velocity


def print_simple_controls():
    """Вывод информации об управлении (WASD вариант)."""
    print("\n" + "=" * 40)
    print("УПРАВЛЕНИЕ САМОЛЕТОМ (WASD + Q/E):")
    print("=" * 40)
    print("W/A/S/D - движение по X и Y")
    print("Q/E - движение вверх/вниз")
    print("=" * 40 + "\n")

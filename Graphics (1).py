from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import math
from Flight_data import time_data, height_data_ksp, velocities_data_ksp, horizontal_velocities, altitudes_data, Horizontal_displacement

# Константы
G = 6.67430e-11
M = 5.2915158 * 10**22 
R = 600_000
R_g = 8.310
M_m = 0.029
F0 = 8109133
F1 = 10631500
T = 135
T_k = 186.95
C = 0.5
S = math.pi * 4.2**2
p0 = 1.2255
H = R + 93
phi0 = math.pi / 2  # Угол в радианах (90 градусов)
beta = -0.0131
m0 = 527561
m_fuel = 293621
k = m_fuel / T
g = 9.81
dt = 0.01

# Плотность
def p_h(h):
    return p0 * math.exp(-(g*M_m*h) / R_g*T_k)


def gravity(h):
    return G * (M / (R + h)**2)


# F_Тяги
def thrust(t):
    a = (F1 - F0) / T

    if t <= T:
        return F0 + a * t
    return 0


# Система ОДУ
def rocket_system(t, state):
    x, y, vx, vy, m_t = state
    m_t = m0 - k * t

    h = y
    phi_t = phi0 + beta * t  # Учитываем обновлённое значение phi0
    
    # Скорость и её модуль
    v = math.sqrt(vx**2 + vy**2)

    #Силы
    F_thrust = thrust(t)
    F_с = 0.5 * C * S * p_h(h) * v**2
    F_tyajesti = gravity(h) * m_t

    # Ускорения
    ax = (F_thrust * math.cos(phi_t) - F_с * math.cos(phi_t)) / m_t
    ay = (F_thrust * math.sin(phi_t) - F_с * math.sin(phi_t) - F_tyajesti) / m_t

    dm = -k if m_t > (m0 - m_fuel) else 0
    
    return [vx, vy, ax, ay, dm]

# Начальные условия
initial_state = [0, H, 0, 0, m0]
time = [i * dt for i in range(int(T / dt))]

# Решение с использованием solve_ivp
solution = solve_ivp(rocket_system, [0, T], initial_state, method='RK45', t_eval=time)

x_positions = solution.y[0]
y_positions = solution.y[1]
velocities = [math.sqrt(vx**2 + vy**2) for vx, vy in zip(solution.y[2], solution.y[3])]

# Графики
plt.figure(figsize=(12, 12))

# Зависимость высоты от времени
plt.subplot(3, 1, 1)
plt.plot(time_data, height_data_ksp, marker='o', linestyle='--', label='Высота KSP', color='red')
plt.plot(time, [i - R for i in y_positions], linestyle='-', label='Мат. модель', color='blue')
plt.title("Сравнение графиков высоты от времени")
plt.xlabel("Время (с)")
plt.ylabel("Высота (м)")
plt.legend()
plt.grid()

# Зависимость скорости от времени
plt.subplot(3, 1, 2)
plt.plot(time, velocities, label="Мат. модель", color="brown")
plt.plot(time_data, velocities_data_ksp, marker='o', linestyle='--', label='KSP', color='red')
plt.title("Зависимость скорости от времени")
plt.xlabel("Время (с)")
plt.ylabel("Скорость (м/с)")
plt.grid()
plt.legend()

# Траектория полета
plt.subplot(3, 1, 3)
plt.plot([i for i in x_positions if i < 47_000], [i - R for i in y_positions[:len([i for i in x_positions if i < 47_000])]], label="Мат. модель", color="green")
plt.plot(Horizontal_displacement, altitudes_data, marker='o', linestyle='--', label='KSP данные', color='red')
plt.title("Траектория полета ракеты")
plt.xlabel("Расстояние по горизонтали (м)")
plt.ylabel("Высота (м)")
plt.grid()
plt.legend()
plt.tight_layout()
plt.show()

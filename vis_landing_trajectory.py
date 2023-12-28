import matplotlib.pyplot as plt
import json
import math

COMPUTE_LANDING = False

dx_flight, dy_flight = [], []
dx_func, dy_func = [], []


def compute_landing_trajectory(flight):
    flight_data = []

    x = 0
    y = flight["altitude"]
    v_x = flight["horizontal_speed"]
    v_y = flight["vertical_speed"]
    mass = flight["mass"]
    t = 0
    delta_t = 0.5

    planet_mass = 5.2915158 * 10**22
    planet_radius = 600000
    g = 6.67430 * 10**-11
    g_planet_mass = g * planet_mass
    pod_square = 2.5**2 * math.pi

    while y > 0:
        v = math.sqrt(v_x**2 + v_y**2)
        air_density = 1.21325 * math.exp(-y / 5000)
        drag = 0.5 * air_density * v**2 * pod_square * 0.65
        drag_x = drag / v * v_x
        drag_y = drag / v * v_y

        g = g_planet_mass / ((planet_radius + y) ** 2) * 0.41

        a_x = abs(drag_x) / mass * -v_x / abs(v_x)
        a_y = -g + abs(drag_y) / mass * -v_y / abs(v_y)

        v_x += a_x * delta_t
        v_y += a_y * delta_t
        x += v_x * delta_t
        y += v_y * delta_t
        t += delta_t

        flight_data.append({"altitude": y, "horizontal_speed": v_x})
    return flight_data


def fill_data(flight_data, dx, dy):
    for entry in flight_data:
        dx.append(dx[-1] + entry["horizontal_speed"] / 2 if len(dx) else 0)
        dy.append(entry["altitude"])


with open("data.json", "r") as data:
    flight_data = json.loads(data.read())
    fill_data(flight_data, dx_flight, dy_flight)
    fill_data(compute_landing_trajectory(flight_data[0]), dx_func, dy_func)

fig, ax = plt.subplots()

ax.plot(dx_flight, dy_flight, label="траектория корабля")
ax.plot(dx_func, dy_func, label="высчитанная траектория")
ax.set_xlabel("пройденное расстояние")
ax.set_ylabel("высота")
ax.set_title("Траектория посадки корабля")
ax.legend()

plt.show()

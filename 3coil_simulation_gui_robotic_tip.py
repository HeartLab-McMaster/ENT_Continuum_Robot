import numpy as np
import magpylib as magpy
import pyvista as pv
import os

os.system('cls')

# Constants
mu_0 = 4 * np.pi * 1e-7  # Permeability of free space (T·m/A)
offset = 0.3  # Offset of 30cm for each coil to be away from center

def create_solenoid(solenoid_current, solenoid_radius=0.1, solenoid_length=0.1, solenoid_turns=145, rotation_angle=0):
    """Creates a solenoid using Magpylib, with optional rotation around the y-axis."""
    solenoid = magpy.Collection()
    for z in np.linspace(-solenoid_length / 2, solenoid_length / 2, solenoid_turns):
        winding = magpy.current.Circle(
            current=solenoid_current,
            diameter=2 * solenoid_radius,
            position=(0, 0, z+offset),
        )
        if rotation_angle != 0:
            winding.rotate_from_angax(rotation_angle, [0, 1, 0], anchor=(0, 0, 0))
        solenoid.add(winding)
    return solenoid

def solenoid_field_on_axis(z, N, I, L, R):
    """Computes the on-axis magnetic field inside and outside the solenoid."""
    n = N / L  # Turns per unit length
    if abs(z) <= L / 2:
        return mu_0 * n * I
    else:
        term1 = (L / 2) / np.sqrt((L / 2) ** 2 + R ** 2)
        term2 = (L / 2) / np.sqrt((L / 2) ** 2 + R ** 2)
        return (mu_0 * I * N / (2 * L)) * (term1 + term2)

def biot_savart_field(position, N, I, L, R):
    """Computes the off-axis magnetic field using Biot-Savart Law."""
    x, y, z = position
    B_total = np.array([0.0, 0.0, 0.0])

    for z_turn in np.linspace(-L / 2, L / 2, N):
        loop_points = 100
        for theta in np.linspace(0, 2 * np.pi, loop_points):
            x_turn, y_turn = R * np.cos(theta), R * np.sin(theta)
            dl = np.array([-np.sin(theta), np.cos(theta), 0]) * (2 * np.pi * R / loop_points)
            r_vec = np.array([x - x_turn, y - y_turn, z - (z_turn+offset)])
            r_mag = np.linalg.norm(r_vec)

            if r_mag == 0:
                continue  

            dB = mu_0 * I * np.cross(dl, r_vec) / (4 * np.pi * r_mag**3)
            B_total += dB

    return B_total

def simulate_coil(solenoid_current=15):
    """Runs the solenoid simulation and visualization."""
    solenoid_radius = 0.1
    solenoid_length = 0.1
    solenoid_turns = 145
    solenoid_rotation = 45

    # Create three solenoids
    solenoid_z = create_solenoid(solenoid_current, solenoid_radius, solenoid_length, solenoid_turns)
    solenoid_pos = create_solenoid(solenoid_current, solenoid_radius, solenoid_length, solenoid_turns, solenoid_rotation)
    solenoid_neg = create_solenoid(solenoid_current, solenoid_radius, solenoid_length, solenoid_turns, -1*solenoid_rotation)

    # Visualization Setup
    pl = pv.Plotter()

    # Create PyVista representation of solenoids
    for solenoid, color, rotation in zip([solenoid_z, solenoid_pos, solenoid_neg], ["blue", "red", "green"], [0, solenoid_rotation, -1*solenoid_rotation]):
        solenoid_points = []
        theta = np.linspace(0, 2 * np.pi, 100)
        x = solenoid_radius * np.cos(theta)
        y = solenoid_radius * np.sin(theta)

        for z in np.linspace(-solenoid_length / 2, solenoid_length / 2, solenoid_turns):
            points = np.column_stack((x, y, np.full_like(theta, z+offset)))
            if rotation != 0:
                rotation_matrix = np.array([
                    [np.cos(np.radians(rotation)), 0, np.sin(np.radians(rotation))],
                    [0, 1, 0],
                    [-np.sin(np.radians(rotation)), 0, np.cos(np.radians(rotation))]
                ])
                points = points @ rotation_matrix.T
            solenoid_points.append(points)

        solenoid_lines = pv.PolyData(np.vstack(solenoid_points))
        pl.add_mesh(solenoid_lines, color=color, line_width=3, label=f"Solenoid {color}")

    # Create an arrow at the center pointing in the Z-direction
    arrow_start = (0, 0, 0)
    arrow_direction = (0, 0, 1)  # Initial direction (Z-axis)
    arrow_scale = 0.05  # Scale for visibility

    arrow = pv.Arrow(start=arrow_start, direction=arrow_direction, scale=arrow_scale)
    arrow_actor = pl.add_mesh(arrow, color="cyan", name="Arrow")

    # Slider Callback to Update Arrow Direction
    def update_arrow(value):
        """Updates the arrow orientation from positive X to negative X."""
        new_direction = (value, 0, 1)  # Modify X-component based on slider
        new_arrow = pv.Arrow(start=arrow_start, direction=new_direction, scale=arrow_scale)
        
        # Remove old arrow and add new one
        pl.remove_actor(arrow_actor)
        pl.add_mesh(new_arrow, color="cyan", name="Arrow")

    # Add slider widget to control arrow orientation
    pl.add_slider_widget(update_arrow, [-1, 1], value=0, title="Arrow X-Orientation")

    # Compute and print magnetic fields
    sensing_positions = [(0, 0, 0), (0, 0, solenoid_length / 2), (0, 0, solenoid_length / 2 + 0.20)]
    sensors = [magpy.Sensor(position=pos) for pos in sensing_positions]

    for sensor, position in zip(sensors, sensing_positions):
        z_pos = position[2] - offset
        if position[0] == 0 and position[1] == 0:
            B_theoretical = np.array([0, 0, solenoid_field_on_axis(z_pos, solenoid_turns, solenoid_current, solenoid_length, solenoid_radius)])
        else:
            B_theoretical = biot_savart_field(position, solenoid_turns, solenoid_current, solenoid_length, solenoid_radius)
        
        # Using sensor from library to obtain field at the specified position for each solenoid
        B_sensor_zaxis_blue = sensor.getB(solenoid_z)
        B_sensor_pos_red = sensor.getB(solenoid_pos)
        B_sensor_neg_green = sensor.getB(solenoid_neg)
        B_sensor_total = B_sensor_zaxis_blue + B_sensor_pos_red + B_sensor_neg_green


        print(f"\nMagnetic Field at {position}:")
        print("--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------")
        print(f"Theoretical Calculation: B_x: {B_theoretical[0] * 1000:.5f} mT, B_y: {B_theoretical[1] * 1000:.5f} mT, B_z: {B_theoretical[2] * 1000:.5f} mT")
        print(f"Sensor Measurement of blue (centered): B_x: {B_sensor_zaxis_blue[0] * 1000:.5f} mT, B_y: {B_sensor_zaxis_blue[1] * 1000:.5f} mT, B_z: {B_sensor_zaxis_blue[2] * 1000:.5f} mT")
        print(f"Sensor Measurement of red (+45): B_x: {B_sensor_pos_red[0] * 1000:.5f} mT, B_y: {B_sensor_pos_red[1] * 1000:.5f} mT, B_z: {B_sensor_pos_red[2] * 1000:.5f} mT")
        print(f"Sensor Measurement of green (-45): B_x: {B_sensor_neg_green[0] * 1000:.5f} mT, B_y: {B_sensor_neg_green[1] * 1000:.5f} mT, B_z: {B_sensor_neg_green[2] * 1000:.5f} mT")
        print(f"\nTotal theoretical calculation = {B_theoretical * 1000} mT")
        print(f"Total sensor measurement = {B_sensor_total*1000} mT")
        print("--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------")

    # Add a small sphere at the center to identify sensor
    center_sphere = pv.Sphere(radius=0.002, center=(0, 0, 0))  # Define the sphere
    pl.add_mesh(center_sphere, color="yellow", specular=0.5, name="Center Sphere")

    # Add a small sphere at the edge to identify sensor
    edge_sphere = pv.Sphere(radius=0.002, center=(0, 0, solenoid_length / 2))  # Define the sphere
    pl.add_mesh(edge_sphere, color="pink", specular=0.5, name="Edge Sphere")

    # Add a small sphere 20cm away to identify sensor
    distance_20_sphere = pv.Sphere(radius=0.002, center=(0, 0, solenoid_length / 2+0.20))  # Define the sphere
    pl.add_mesh(distance_20_sphere, color="orange", specular=0.5, name="20cm Sphere")

    # Add XYZ Axes
    axis_length = 0.2
    pl.add_mesh(pv.Line((-axis_length, 0, 0), (axis_length, 0, 0)), color="red", line_width=4)
    pl.add_mesh(pv.Line((0, -axis_length, 0), (0, axis_length, 0)), color="green", line_width=4)
    pl.add_mesh(pv.Line((0, 0, -axis_length), (0, 0, axis_length)), color="blue", line_width=4)

    pl.add_axes()
    pl.camera.position = (0.2, 0.2, 0.2)
    pl.show()

if __name__ == "__main__":
    simulate_coil()

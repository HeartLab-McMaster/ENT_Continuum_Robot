import numpy as np
import magpylib as magpy
import pyvista as pv
import os

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

def simulate_coil(solenoid_current=15):
    """Runs the solenoid simulation and visualization."""
    solenoid_radius = 0.1
    solenoid_length = 0.1
    solenoid_turns = 145

    # Create three solenoids
    solenoid_z = create_solenoid(solenoid_current, solenoid_radius, solenoid_length, solenoid_turns)  # Original along z-axis
    solenoid_p45 = create_solenoid(solenoid_current, solenoid_radius, solenoid_length, solenoid_turns, 45)  # Rotated +45 degrees
    solenoid_n45 = create_solenoid(solenoid_current, solenoid_radius, solenoid_length, solenoid_turns, -45)  # Rotated -45 degrees

    # Visualization Setup
    pl = pv.Plotter()

    # Create PyVista representation of solenoids
    for solenoid, color, rotation in zip([solenoid_z, solenoid_p45, solenoid_n45], ["blue", "red", "green"], [0, 45, -45]):
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

    # Add XYZ Axes for visualization
    axis_length = 0.2
    x_axis = pv.Line((-axis_length, 0, 0), (axis_length, 0, 0))
    y_axis = pv.Line((0, -axis_length, 0), (0, axis_length, 0))
    z_axis = pv.Line((0, 0, -axis_length), (0, 0, axis_length))

    pl.add_mesh(x_axis, color="red", line_width=4, label="X-Axis")
    pl.add_mesh(y_axis, color="green", line_width=4, label="Y-Axis")
    pl.add_mesh(z_axis, color="blue", line_width=4, label="Z-Axis")

    # Camera setup and display
    pl.add_axes()
    pl.camera.position = (0.2, 0.2, 0.2)
    pl.show()

if __name__ == "__main__":
    simulate_coil()

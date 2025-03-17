import numpy as np
import magpylib as magpy
import pyvista as pv
import os
import serial  # Import serial for ESP32 communication
import time    

# os.system('cls')
# Setup Serial Communication
SERIAL_PORT = "COM9" 
BAUD_RATE = 115200

# Open Serial connection
try:
    esp32 = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)  
    time.sleep(2)  
    print(f"Connected to ESP32 on {SERIAL_PORT}")
except serial.SerialException:
    print("Failed to connect to ESP32. Check the port and try again.")
    esp32 = None 
    exit 

################################### #Start of GUI

# Constants
mu_0 = 4 * np.pi * 1e-7  # Permeability of free space
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
    solenoid_rotation = 45

    # Create three solenoids
    solenoid_z = create_solenoid(solenoid_current, solenoid_radius, solenoid_length, solenoid_turns)
    solenoid_pos = create_solenoid(solenoid_current, solenoid_radius, solenoid_length, solenoid_turns, solenoid_rotation)
    solenoid_neg = create_solenoid(solenoid_current, solenoid_radius, solenoid_length, solenoid_turns, -1*solenoid_rotation)

    # Visualization Setup
    pl = pv.Plotter()

    # **Ensure solenoids are visible initially by explicitly adding them**
    for solenoid, color, rotation in zip(
        [solenoid_z, solenoid_pos, solenoid_neg], ["grey", "grey", "grey"], [0, solenoid_rotation, -1 * solenoid_rotation]
    ):
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

    # Joystick background panel (to visualize the control area)
    joystick_panel = pv.Plane(center=(0, 0, -0.3), direction=(0, 1, 0), i_size=0.2, j_size=0.2)
    pl.add_mesh(joystick_panel, color="lightgray", opacity=0.5, name="Joystick Panel")

    # Horizontal Line (X-axis guide)
    horizontal_line = pv.Line((-0.1, 0, -0.3), (0.1, 0, -0.3))

    # Vertical Line (Z-axis guide)
    vertical_line = pv.Line((0, 0, -0.2), (0, 0, -0.3))  # Adjusted to be along Z-axis

    # Diagonal Line 1 - NEED TO ADJUST
    diagonal_line1 = pv.Line((0, 0, -0.3), (0.1, 0, -0.2))  

    # Diagonal Line 2 
    diagonal_line2 = pv.Line((-0.1, 0, -0.2), (0, 0, -0.3)) 

    for line in [horizontal_line, vertical_line, diagonal_line1, diagonal_line2]:
        pl.add_mesh(line, color="white", line_width=3)

    # Create an arrow at the center pointing in the Z-direction
    arrow_start = (0, 0, 0)
    arrow_direction = (0, 0, 1)  # Initial direction (Z-axis)
    arrow_scale = 0.05  # Scale for visibility
    arrow = pv.Arrow(start=arrow_start, direction=arrow_direction, scale=arrow_scale)
    arrow_actor = pl.add_mesh(arrow, color="cyan", name="Arrow")

    # **Define `update_arrow` AFTER joystick widget is created**
    def update_arrow(center):
        value = center[0] * 15
        z_value = center[2]  # Extract Z component to detect up/down movement

        # Reset joystick position after movement
        joystick_widget.SetCenter((0, 0, -0.3))

        # **Check if joystick is moved downward**
        if z_value < -0.3:  # Ensure significant downward movement
            print("Moving tip downwards.")
            color = ["grey", "grey", "grey"]  # *Reset all coils to grey*
            command = "DOWN\n"
            new_arrow_direction = (0, 0, 1)  # **Reset arrow to default Z-axis*

        else:
            if value > -0.7 and value < -0.3:
                color = ["green", "grey", "green"]
                print("Left and Middle Coil ON")
                command = "TOP LEFT\n"
            elif value < 0.7 and value > 0.3:
                color = ["green", "green", "grey"]
                print("Middle and Right Coil ON")
                command = "TOP RIGHT\n"
            elif value < -0.5:
                color = ["grey", "grey", "green"]
                print("Left Coil ON")
                command = "LEFT\n"
            elif value > -0.5 and value < 0.5:
                color = ["green", "grey", "grey"]
                print("Middle Coil ON")
                command = "MIDDLE\n"
            elif value > 0.5:
                color = ["grey", "green", "grey"]
                print("Right Coil ON")
                command = "RIGHT\n"
            else:
                command = "OFF\n"

            # **Update arrow direction based on joystick movement**
            new_arrow_direction = (value, 0, 1)  # Small X-axis movement

        # **Update the arrow orientation**
        new_arrow = pv.Arrow(start=arrow_start, direction=new_arrow_direction, scale=arrow_scale)
        pl.remove_actor(arrow_actor)
        pl.add_mesh(new_arrow, color="cyan", name="Arrow")

        # **Apply the color update to solenoids (same as existing logic)**
        for solenoid, coil_color, rotation in zip(
            [solenoid_z, solenoid_pos, solenoid_neg], color, [0, solenoid_rotation, -1 * solenoid_rotation]
        ):
            solenoid_points = []
            theta = np.linspace(0, 2 * np.pi, 100)
            x = solenoid_radius * np.cos(theta)
            y = solenoid_radius * np.sin(theta)

            for z in np.linspace(-solenoid_length / 2, solenoid_length / 2, solenoid_turns):
                points = np.column_stack((x, y, np.full_like(theta, z + offset)))
                if rotation != 0:
                    rotation_matrix = np.array([
                        [np.cos(np.radians(rotation)), 0, np.sin(np.radians(rotation))],
                        [0, 1, 0],
                        [-np.sin(np.radians(rotation)), 0, np.cos(np.radians(rotation))]
                    ])
                    points = points @ rotation_matrix.T
                solenoid_points.append(points)

            solenoid_lines = pv.PolyData(np.vstack(solenoid_points))
            pl.add_mesh(solenoid_lines, color=coil_color, line_width=3, label=f"Solenoid {coil_color}")

        if esp32:
            print("After esp32")
            try:
                print("TRY")
                esp32.write(command.encode())
                print(f"Sent to ESP32: {command.strip()}")
            except serial.SerialException:
                print("EXCEPT")
                print("Serial write failed. Check ESP32 connection.")

    # **Define `joystick_widget` BEFORE calling `update_arrow`**
    joystick_widget = pl.add_sphere_widget(update_arrow, center=(0, 0, -0.3), radius=0.01, color="red")

    # Add XYZ Axes
    axis_length = 0.2
    pl.add_mesh(pv.Line((-axis_length, 0, 0), (axis_length, 0, 0)), color="red", line_width=4)
    pl.add_mesh(pv.Line((0, -axis_length, 0), (0, axis_length, 0)), color="green", line_width=4)
    pl.add_mesh(pv.Line((0, 0, -axis_length), (0, 0, axis_length)), color="blue", line_width=4)

    pl.add_axes()
    pl.camera.position = (0, -1.5, 0)
    pl.show()

if __name__ == "__main__":
    simulate_coil()
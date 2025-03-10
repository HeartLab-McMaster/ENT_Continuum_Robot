import numpy as np
import magpylib as magpy
import pyvista as pv
import os

#os.system('cls')

# Constants
mu_0 = 4 * np.pi * 1e-7  # Permeability of free space (T·m/A)
offset = 0.3 # Offset of 30cm for each coil to be away from center

def create_solenoid(solenoid_current, solenoid_radius=0.1, solenoid_length=0.1, solenoid_turns=145):
    """Creates a solenoid using Magpylib."""
    solenoid = magpy.Collection()
    for z in np.linspace(-solenoid_length / 2, solenoid_length / 2, solenoid_turns):
        winding = magpy.current.Circle(
            current=solenoid_current,
            diameter=2 * solenoid_radius,
            position=(0, 0, z+offset),
        )
        solenoid.add(winding)
    return solenoid

def solenoid_field_on_axis(z, N, I, L, R):
    """Computes the on-axis magnetic field inside and outside the solenoid."""
    n = N / L  # Turns per unit length
    if abs(z) <= L / 2:
        return mu_0 * n * I
    else:
        #this code is changed; check backend calc gui code!
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

def simulate_coil(solenoid_current = 15):
    """Runs the solenoid simulation and visualization."""
    solenoid_radius = 0.1
    solenoid_length = 0.1
    solenoid_turns = 145

    print(f"\nCurrent of solenoids set to {solenoid_current} A\n")
    
    solenoid = create_solenoid(solenoid_current, solenoid_radius, solenoid_length, solenoid_turns)

    # May need to change with more coils
    sensing_positions = [
        (0, 0, 0),  # Center of solenoid
        (0, 0, solenoid_length / 2),  # Edge of solenoid
        (0, 0, solenoid_length / 2 + 0.20),  # 20 cm away from edge
        #(0.02, 0, solenoid_length / 2 + 0.10),  # Off-axis, outside solenoid
    ]

    print("\nCalculating Magnetic Fields at Sensor Positions...\n")
    for position in sensing_positions:
        z_pos = position[2] - offset

        # On-axis: Use solenoid formula
        if position[0] == 0 and position[1] == 0:
            B_theoretical = np.array([0, 0, solenoid_field_on_axis(z_pos, solenoid_turns, solenoid_current, solenoid_length, solenoid_radius)])
        else:
            B_theoretical = biot_savart_field(position, solenoid_turns, solenoid_current, solenoid_length, solenoid_radius)

        # Compute Magpylib field
        B_magpylib = solenoid.getB(position)

        # Print comparison
        print(f"\nMagnetic Field at position {position}:")
        print(f"Theoretical Calculation: B_x: {B_theoretical[0] * 1000:.5f} mT, B_y: {B_theoretical[1] * 1000:.5f} mT, B_z: {B_theoretical[2] * 1000:.5f} mT")
        print(f"Magpylib Sensor:    B_x: {B_magpylib[0] * 1000:.5f} mT, B_y: {B_magpylib[1] * 1000:.5f} mT, B_z: {B_magpylib[2] * 1000:.5f} mT")

    # Visualization Setup
    pl = pv.Plotter()

    print("After plotter setup")

    # Create PyVista representation of solenoid
    solenoid_points = []
    theta = np.linspace(0, 2 * np.pi, 100)
    x = solenoid_radius * np.cos(theta)
    y = solenoid_radius * np.sin(theta)

    for z in np.linspace(-solenoid_length / 2, solenoid_length / 2, solenoid_turns):
        solenoid_points.append(np.column_stack((x, y, np.full_like(theta, z+offset))))

    solenoid_lines = pv.PolyData(np.vstack(solenoid_points))
    pl.add_mesh(solenoid_lines, color="blue", line_width=3, label="Solenoid")

    # Add XYZ Axes for visualization
    axis_length = 0.9
    x_axis = pv.Line((-axis_length, 0, 0), (axis_length, 0, 0))
    y_axis = pv.Line((0, -axis_length, 0), (0, axis_length, 0))
    z_axis = pv.Line((0, 0, -axis_length), (0, 0, axis_length))

    pl.add_mesh(x_axis, color="red", line_width=4, label="X-Axis")
    pl.add_mesh(y_axis, color="green", line_width=4, label="Y-Axis")
    pl.add_mesh(z_axis, color="blue", line_width=4, label="Z-Axis")

    print("After solenoid setup")

    #can be ignored for now, may need to remove bc this doesnt bring up visualization
    """  
    theta = np.linspace(0, 2 * np.pi, 100)
    x, y = solenoid_radius * np.cos(theta), solenoid_radius * np.sin(theta)
    solenoid_points = [np.column_stack((x, y, np.full_like(theta, z))) for z in np.linspace(-solenoid_length / 2, solenoid_length / 2, solenoid_turns)]
    
    solenoid_lines = pv.PolyData(np.vstack(solenoid_points))
    pl.add_mesh(solenoid_lines, color="blue", line_width=3, label="Solenoid")
"""
    # Add sensor positions
    sensor_colors = ['yellow', 'green', 'red', 'cyan']
    for idx, position in enumerate(sensing_positions):
        sensor_sphere = pv.Sphere(radius=0.002, center=position)
        pl.add_mesh(sensor_sphere, color=sensor_colors[idx], specular=0.5, name=f"Sensor {idx+1}")

    print("After sensor setup")

    # Magnetic field visualization grid
    grid_points = np.array(np.meshgrid(np.linspace(-0.1, 0.1, 5),
                                       np.linspace(-0.1, 0.1, 5),
                                       np.linspace(-0.1, 0.1, 5))).T.reshape(-1, 3)

    B_field = np.array([biot_savart_field(pos, solenoid_turns, solenoid_current, solenoid_length, solenoid_radius) for pos in grid_points]) * 1000

    print("After field grid setup")
    grid = pv.PolyData(grid_points)
    grid["B_magnitude"] = np.linalg.norm(B_field, axis=1)
    grid["B_vectors"] = B_field

    print(f"B field value = {B_field}")

    glyphs = grid.glyph(
        orient="B_vectors",
        scale=False,
        factor=0.0000000001,
    )
    pl.add_mesh(
        glyphs,
        color="blue",
        show_scalar_bar=True,
        label="Magnetic Field Vectors",
    )
    print("After adding vectors")

    pl.add_axes()
    pl.camera.position = (0.2, 0.2, 0.2)
    pl.show()


if __name__ == "__main__":
    simulate_coil()

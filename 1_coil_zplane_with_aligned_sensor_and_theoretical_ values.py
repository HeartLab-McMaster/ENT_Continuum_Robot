import numpy as np
import magpylib as magpy
import pyvista as pv

# Create the coil
coil1 = magpy.Collection()
for z in np.linspace(-0.08, 0.08, 16):
    winding = magpy.current.Circle(
        current=100,
        diameter=0.1,
        position=(0, 0, z),
    )
    coil1.add(winding)

# Create PyVista representation of the coil
coil_points = []
for z in np.linspace(-0.08, 0.08, 16):
    theta = np.linspace(0, 2 * np.pi, 100)
    x = 0.05 * np.cos(theta)
    y = 0.05 * np.sin(theta)
    z_points = np.full_like(theta, z)
    coil_points.append(np.column_stack((x, y, z_points)))

coil_lines = pv.PolyData(np.vstack(coil_points))

# Create a PyVista plotting scene
pl = pv.Plotter()

# Add the coil mesh to the scene
pl.add_mesh(coil_lines, color="blue", line_width=3, label="Coil")

# Add a small sphere at the center (0, 0, 0)
sphere = pv.Sphere(radius=0.002, center=(0, 0, 0))
pl.add_mesh(sphere, color="yellow", specular=0.5, name="Center Sphere")

# Add a sensor at the center (0, 0, 0)
sensor_position = (0, 0, 0)
sensor_field = coil1.getB(sensor_position) * 1000  # Get B-field at the sensor position and convert to mT

# Print the sensor fields
print("Magnetic Field at the Center from Sensor:")
print(f"B_x: {sensor_field[0]:.5f} mT")
print(f"B_y: {sensor_field[1]:.5f} mT")
print(f"B_z: {sensor_field[2]:.5f} mT")
print(f"Total B (|B|): {np.linalg.norm(sensor_field):.5f} mT")

# Theoretical math to calculate magnetic field at center !!!!!!!!!!!!
# constants
mu = 4 * np.pi * 1e-7  # Permeability of free space (T·m/A)
I = 100  # Current in amperes
R = 0.05  # Radius of the loop in meters
z_positions = np.linspace(-0.08, 0.08, 16)  # Positions of the loops along the z-axis

# Function to calculate B_z at the center from a single loop
def magnetic_field_center(I, R, z, mu):
    return (mu * I * R**2) / (2 * (R**2 + z**2)**(3/2))

# Sum the contributions from all loops
B_total_mT = sum(magnetic_field_center(I, R, z, mu) for z in z_positions)

# Convert to mT (milliTesla)
B_total_mT = B_total_mT * 1000
print(f"Theoretical Magnetic Field at the Center (0, 0, 0): {B_total_mT:.5f} mT")

# Add axes and adjust the camera position
pl.add_axes()
pl.camera.position = (0.2, 0.2, 0.2)

# Show the scene
pl.show()

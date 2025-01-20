import numpy as np
import magpylib as magpy
import pyvista as pv

# Create the coil using Magpylib
coil1 = magpy.Collection()  # Create a collection to hold multiple loops
for z in np.linspace(-0.08, 0.08, 16):  # Position loops along the z-axis
    winding = magpy.current.Circle(  # Define a single circular loop
        current=100,  # Current in the loop (in amperes)
        diameter=0.1,  # Diameter of the loop (in meters)
        position=(0, 0, z),  # Position of the loop in 3D space
    )
    coil1.add(winding)  # Add the loop to the collection

# Create PyVista representation of the coil for visualization
coil_points = []  # List to hold the points of all loops
for z in np.linspace(-0.08, 0.08, 16):  # Loop positions along the z-axis
    theta = np.linspace(0, 2 * np.pi, 100)  # Define angles for the circular loop
    x = 0.05 * np.cos(theta)  # x-coordinates of the loop
    y = 0.05 * np.sin(theta)  # y-coordinates of the loop
    z_points = np.full_like(theta, z)  # z-coordinates for the loop
    coil_points.append(np.column_stack((x, y, z_points)))  # Combine points into 3D coordinates

coil_lines = pv.PolyData(np.vstack(coil_points))  # Create a PolyData object for the coil

# Create a 3D grid of points for magnetic field visualization
x = np.linspace(-0.1, 0.1, 10)  # Grid along the x-axis
y = np.linspace(-0.1, 0.1, 10)  # Grid along the y-axis
z = np.linspace(-0.1, 0.1, 10)  # Grid along the z-axis
grid_points = np.array(np.meshgrid(x, y, z)).T.reshape(-1, 3)  # Combine into a 3D grid

# Compute magnetic field vectors at each grid point
B_field = np.array([coil1.getB(pos) for pos in grid_points]) * 1000  # Convert to milliTesla (mT)

# Add magnetic field data to the grid for visualization
grid = pv.PolyData(grid_points)  # Create a PolyData object for the grid points
grid["B_magnitude"] = np.linalg.norm(B_field, axis=1)  # Add field magnitudes for coloring
grid["B_vectors"] = B_field  # Add field vectors for arrow orientation

# Create a PyVista plotting scene
pl = pv.Plotter()

# Add the coil mesh to the scene
pl.add_mesh(coil_lines, color="blue", line_width=3, label="Coil")

# Add a small sphere at the center (0, 0, 0)
sphere = pv.Sphere(radius=0.002, center=(0, 0, 0))  # Define the sphere
pl.add_mesh(sphere, color="yellow", specular=0.5, name="Center Sphere")  # Add it to the scene

# Add magnetic field vectors as arrows
glyphs = grid.glyph(
    orient="B_vectors",  # Use magnetic field vectors for arrow orientation
    scale=True,          # Scale arrows by magnitude
    factor=0.001,        # Scale factor for arrow size
)
pl.add_mesh(
    glyphs,
    color="blue",
    show_scalar_bar=False,
    label="Magnetic Field Vectors",
)

# Add axes and set the camera position
pl.add_axes()  # Add axes to the scene
pl.camera.position = (0.2, 0.2, 0.2)  # Adjust the camera view

# Compute and print the magnetic field at the center
sensor_position = (0, 0, 0)  # Center position
sensor_field = coil1.getB(sensor_position) * 1000  # Compute field and convert to mT
print("Magnetic Field at the Center from Sensor:")
print(f"B_x: {sensor_field[0]:.5f} mT")  # x-component
print(f"B_y: {sensor_field[1]:.5f} mT")  # y-component
print(f"B_z: {sensor_field[2]:.5f} mT")  # z-component
print(f"Total B (|B|): {np.linalg.norm(sensor_field):.5f} mT")  # Total magnitude

# Compute the theoretical magnetic field at the center
mu = 4 * np.pi * 1e-7  # Permeability of free space
I = 100  # Current in amperes
R = 0.05  # Radius of the loop
z_positions = np.linspace(-0.08, 0.08, 16)  # Positions of the loops
def magnetic_field_center(I, R, z, mu):
    return (mu * I * R**2) / (2 * (R**2 + z**2)**(3/2))  # Magnetic field formula
B_total_mT = sum(magnetic_field_center(I, R, z, mu) for z in z_positions) * 1000  # Total field in mT
print(f"Theoretical Magnetic Field at the Center (0, 0, 0): {B_total_mT:.5f} mT")

# Show the 3D scene
pl.show()

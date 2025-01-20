import numpy as np
import magpylib as magpy
import pyvista as pv

coil1_current = 100
coil1_diameter = 0.1

coil2_current = -100
coil2_diameter = 0.1
# Create the coil(1) using Magpylib
coil1 = magpy.Collection()  # Create a collection to hold multiple loops
for z in np.linspace(-0.16, -0.08, 16):  # Position loops along the z-axis
    winding = magpy.current.Circle(  # Define a single circular loop
        current=coil1_current,  # Current in the loop (in amperes)
        diameter=coil1_diameter,  # Diameter of the loop (in meters)
        position=(0, 0, z),  # Position of the loop in 3D space
    )
    coil1.add(winding)  # Add the loop to the collection

# Create the coil(2) using Magpylib
coil2 = magpy.Collection()  # Create a collection to hold multiple loops
for z in np.linspace(0.16, 0.08, 16):  # Position loops along the z-axis
    winding = magpy.current.Circle(  # Define a single circular loop
        current=coil2_current,  # Current in the loop (in amperes)
        diameter=coil2_diameter,  # Diameter of the loop (in meters)
        position=(0, 0, z),  # Position of the loop in 3D space
    )
    coil2.add(winding)  # Add the loop to the collection



# Create PyVista representation of the coil(1) for visualization
coil_points1 = []  # List to hold the points of all loops
for z in np.linspace(-0.16, -0.08, 16):  # Loop positions along the z-axis
    theta = np.linspace(0, 2 * np.pi, 100)  # Define angles for the circular loop
    x = 0.05 * np.cos(theta)  # x-coordinates of the loop
    y = 0.05 * np.sin(theta)  # y-coordinates of the loop
    z_points = np.full_like(theta, z)  # z-coordinates for the loop
    coil_points1.append(np.column_stack((x, y, z_points)))  # Combine points into 3D coordinates


# Create PyVista representation of the coil(2) for visualization
coil_points2 = []  # List to hold the points of all loops
for z in np.linspace(0.16, 0.08, 16):  # Loop positions along the z-axis
    theta = np.linspace(0, 2 * np.pi, 100)  # Define angles for the circular loop
    x = 0.05 * np.cos(theta)  # x-coordinates of the loop
    y = 0.05 * np.sin(theta)  # y-coordinates of the loop
    z_points = np.full_like(theta, z)  # z-coordinates for the loop
    coil_points2.append(np.column_stack((x, y, z_points)))  # Combine points into 3D coordinates

coil_lines1 = pv.PolyData(np.vstack(coil_points1))  # Create a PolyData object for the coil
coil_lines2 = pv.PolyData(np.vstack(coil_points2))  # Create a PolyData object for the coil


# Create a 3D grid of points for magnetic field visualization
x = np.linspace(-0.2, 0.2, 10)  # Grid along the x-axis
y = np.linspace(-0.2, 0.2, 10)  # Grid along the y-axis
z = np.linspace(-0.2, 0.2, 10)  # Grid along the z-axis
grid_points = np.array(np.meshgrid(x, y, z)).T.reshape(-1, 3)  # Combine into a 3D grid

# Compute magnetic field vectors at each grid point
B_field1 = np.array([coil1.getB(pos) for pos in grid_points]) * 1000  # Convert to milliTesla (mT)
B_field2 = np.array([coil2.getB(pos) for pos in grid_points]) * 1000  # Convert to milliTesla (mT)

B_field_combined = B_field1 + B_field2

# Add magnetic field data to the grid for visualization
grid = pv.PolyData(grid_points)  # Create a PolyData object for the grid points
grid["B_magnitude"] = np.linalg.norm(B_field_combined, axis=1)  # Add field magnitudes for coloring
grid["B_vectors"] = B_field_combined  # Add field vectors for arrow orientation



# Create a PyVista plotting scene
pl = pv.Plotter()

# Add the coil mesh to the scene
pl.add_mesh(coil_lines1, color="blue", line_width=3, label="Coil 1")
pl.add_mesh(coil_lines2, color="red", line_width=3, label="Coil 2")

# Add a small sphere at the center to identify sensor
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
    show_scalar_bar=True,
    label="Magnetic Field Vectors",
)

# Add axes and set the camera position
pl.add_axes()  # Add axes to the scene
pl.camera.position = (0.3, 0.3, 0.3)  # Adjust the camera view

# Compute magnetic field using sensor
sensor_position = (0, 0, 0)  # Center position
sensor_field1 = coil1.getB(sensor_position) * 1000  # Compute field and convert to mT
sensor_field2 = coil2.getB(sensor_position) * 1000  # Convert to mT
total_sensor_field = sensor_field1 + sensor_field2

print("Magnetic Field at the Center from Sensor:")
print(f"B_x: {total_sensor_field[0]:.5f} mT")  # x-component
print(f"B_y: {total_sensor_field[1]:.5f} mT")  # y-component
print(f"B_z: {total_sensor_field[2]:.5f} mT")  # z-component
print(f"Total B (|B|): {np.linalg.norm(total_sensor_field):.5f} mT")  # Total magnitude

# Compute the theoretical magnetic field at the center for coil 1
mu = 4 * np.pi * 1e-7  # Permeability of free space
I = coil1_current  # Current in amperes
R = coil1_diameter/2  # Radius of the loop
z_positions = np.linspace(-0.16, -0.08, 16)  # Positions of the loops
def magnetic_field_center(I, R, z, mu):
    return (mu * I * R**2) / (2 * (R**2 + z**2)**(3/2))  # Magnetic field formula - Bio-savert Law of field of loop away from point
B_total_coil1 = sum(magnetic_field_center(I, R, z, mu) for z in z_positions) * 1000  # Total field in mT


# Compute the theoretical magnetic field at the center for coil 2
I = coil2_current  # Current in amperes
R = coil2_diameter/2  # Radius of the loop
z_positions = np.linspace(0.16, 0.08, 16)  # Positions of the loops
B_total_coil2 = sum(magnetic_field_center(I, R, z, mu) for z in z_positions) * 1000  # Total field in mT

B_total_mT = B_total_coil1 + B_total_coil2
print(f"Theoretical Magnetic Field at the Center (0, 0, 0): {B_total_mT:.5f} mT")

# Show the 3D scene
pl.show()

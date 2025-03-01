import numpy as np
import magpylib as magpy
import pyvista as pv

# Define parameters for the coil
coil_current = 15  # Current in amperes
coil_radius = 0.1  # Radius in meters
coil_length = 0.1  # Length of the coil in meters
coil_turns = 100  # Number of turns
theoretical_val = 12.57

# Create the coil using Magpylib
coil1 = magpy.Collection()  # Create a collection to hold multiple loops

# Calculate spacing between loops based on the length and turns
loop_spacing = coil_length / coil_turns

# Add loops to the collection
for z in np.linspace(-coil_length / 2, coil_length / 2, coil_turns):  # Symmetrically position loops
    winding = magpy.current.Circle(
        current=coil_current,  # Current in the loop
        diameter=2 * coil_radius,  # Diameter (twice the radius)
        position=(0, 0, z),  # Position of the loop along the z-axis
    )
    coil1.add(winding)  # Add the loop to the collection

# Create PyVista representation of the coil for visualization
coil_points = []  # List to hold the points of all loops
for z in np.linspace(-coil_length / 2, coil_length / 2, coil_turns):  # Loop positions along the z-axis
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

# Add a small sphere at the center to identify sensor
center_sphere = pv.Sphere(radius=0.002, center=(0, 0, 0))  # Define the sphere
pl.add_mesh(center_sphere, color="yellow", specular=0.5, name="Center Sphere")

# Add a small sphere at the edge to identify sensor
edge_sphere = pv.Sphere(radius=0.002, center=(0, 0, coil_length / 2))  # Define the sphere
pl.add_mesh(edge_sphere, color="green", specular=0.5, name="Edge Sphere")

# Add a small sphere 20cm away to identify sensor
distance_20_sphere = pv.Sphere(radius=0.002, center=(0, 0, coil_length / 2+0.20))  # Define the sphere
pl.add_mesh(distance_20_sphere, color="red", specular=0.5, name="20cm Sphere")

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
pl.camera.position = (0.2, 0.2, 0.2)  # Adjust the camera view

# Compute magnetic field using sensor at center
sensor_position_center = (0, 0, 0)  # Center position
sensor_field_center = coil1.getB(sensor_position_center) * 1000  # Compute field and convert to mT
print("Magnetic Field at the Center from Sensor:")
print(f"B_x: {sensor_field_center[0]:.5f} mT")  # x-component
print(f"B_y: {sensor_field_center[1]:.5f} mT")  # y-component
print(f"B_z: {sensor_field_center[2]:.5f} mT")  # z-component
print(f"Total B (|B|): {np.linalg.norm(sensor_field_center):.5f} mT")  # Total magnitude

# Compute magnetic field using sensor at edge
sensor_position_edge = (0, 0, coil_length / 2)  # Center position
sensor_field_edge = coil1.getB(sensor_position_edge) * 1000  # Compute field and convert to mT
print("\nMagnetic Field at the Edge from Sensor:")
print(f"B_x: {sensor_field_edge[0]:.5f} mT")  # x-component
print(f"B_y: {sensor_field_edge[1]:.5f} mT")  # y-component
print(f"B_z: {sensor_field_edge[2]:.5f} mT")  # z-component
print(f"Total B (|B|): {np.linalg.norm(sensor_field_edge):.5f} mT")  # Total magnitude

# Compute magnetic field using sensor at away
sensor_position_20 = (0, 0, coil_length / 2+0.20)  # Center position
sensor_field_20 = coil1.getB(sensor_position_20) * 1000  # Compute field and convert to mT
print("\nMagnetic Field away from the Sensor:")
print(f"B_x: {sensor_field_20[0]:.5f} mT")  # x-component
print(f"B_y: {sensor_field_20[1]:.5f} mT")  # y-component
print(f"B_z: {sensor_field_20[2]:.5f} mT")  # z-component
print(f"Total B (|B|): {np.linalg.norm(sensor_field_20):.5f} mT")  # Total magnitude

# Compute the theoretical magnetic field at the center
mu = 4 * np.pi * 1e-7  # Permeability of free space
I = coil_current  # Current in amperes
R = coil_radius  # Radius of the loop
z_positions = np.linspace(-coil_length / 2, coil_length / 2, coil_turns)  # Positions of the loops

def magnetic_field_center(I, R, z, mu):
    return (mu * I * R**2) / (2 * (R**2 + z**2)**(3/2))  # Magnetic field formula - Bio-savert Law of field of loop away from point
B_total_mT = sum(magnetic_field_center(I, R, z, mu) for z in z_positions) * 1000  # Total field in mT
print(f"\nTheoretical Magnetic Field at the Center (0, 0, 0): {B_total_mT:.5f} mT")

# Show the 3D scene
pl.show()

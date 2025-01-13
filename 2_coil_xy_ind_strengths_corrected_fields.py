import numpy as np
import magpylib as magpy
import pyvista as pv

# Define the coil offsets
coil_offset = 0.025  # 25 mm offset to separate the coils
coil_radius = 0.005  # 5 mm radius of the coils

# Calculate the symmetric tilt angle
tilt_angle = np.arctan2(coil_offset, coil_radius)  # Tilt angle in radians

# Define two coils (Coil 2 and Coil 3 only) as collections of circular current loops
coil2 = magpy.Collection()
coil3 = magpy.Collection()

# Coil 2: Along the x-axis (red), shifted and rotated
for x in np.linspace(-0.008, 0.008, 16):
    winding = magpy.current.Circle(
        current=100,  # Current in amperes
        diameter=0.01,  # Diameter of the winding in meters
        position=(coil_offset + x, 0, 0),  # Shifted along x-axis
    )
    # Rotate the coil to align symmetrically toward the center
    winding.rotate_from_angax(tilt_angle, [0, 1, 0], anchor=(coil_offset, 0, 0))
    coil2.add(winding)

# Coil 3: Along the y-axis (green), shifted and rotated
for y in np.linspace(-0.008, 0.008, 16):
    winding = magpy.current.Circle(
        current=100,  # Current in amperes
        diameter=0.01,  # Diameter of the winding in meters
        position=(0, coil_offset + y, 0),  # Shifted along y-axis
    )
    # Rotate the coil to align symmetrically toward the center
    winding.rotate_from_angax(-tilt_angle, [1, 0, 0], anchor=(0, coil_offset, 0))
    coil3.add(winding)

# Combine Coil 2 and Coil 3 into a single collection
combined_coil = magpy.Collection(coil2, coil3)

# Add a sensor at the center (0, 0, 0)
sensor_position = (0, 0, 0)
sensor_field = combined_coil.getB(sensor_position) * 1000  # Get B-field at the sensor position and convert to mT

# Print the sensor fields
print("Magnetic Field at the Center (0, 0, 0):")
print(f"B_x: {sensor_field[0]:.5f} mT")
print(f"B_y: {sensor_field[1]:.5f} mT")
print(f"B_z: {sensor_field[2]:.5f} mT")
print(f"Total B (|B|): {np.linalg.norm(sensor_field):.5f} mT")

# Create a 3D grid with Pyvista
grid = pv.ImageData(
    dimensions=(21, 21, 21),  # Adjust resolution for two coils
    spacing=(0.004, 0.004, 0.004),  # Spacing between grid points in meters
    origin=(-0.04, -0.04, -0.02),  # Origin of the grid
)

# Compute the cumulative B-field and add as data to grid
B_field = combined_coil.getB(grid.points) * 1000  # Compute B-field in mT
grid["B_magnitude"] = np.linalg.norm(B_field, axis=1)  # Add magnitude as scalar data
grid["B_vector"] = B_field  # Add vector data for glyphs

# Compute the field lines
seed = pv.Disc(inner=0.002, outer=0.005, r_res=1, c_res=9)  # Disc for streamline seeds
strl = grid.streamlines_from_source(
    seed,
    vectors="B_vector",
    max_step_length=0.1,
    max_time=0.02,
    integration_direction="both",
)

# Create a Pyvista plotting scene
pl = pv.Plotter()

# Add XYZ planes
# X-Y Plane
xy_plane = pv.Plane(
    center=(0, 0, 0),
    direction=(0, 0, 1),  # Normal vector of the plane
    i_size=0.05,
    j_size=0.05,
)
pl.add_mesh(xy_plane, color="lightblue", opacity=0.4, name="X-Y Plane")

# Y-Z Plane
yz_plane = pv.Plane(
    center=(0, 0, 0),
    direction=(1, 0, 0),  # Normal vector of the plane
    i_size=0.05,
    j_size=0.05,
)
pl.add_mesh(yz_plane, color="lightgreen", opacity=0.4, name="Y-Z Plane")

# X-Z Plane
xz_plane = pv.Plane(
    center=(0, 0, 0),
    direction=(0, 1, 0),  # Normal vector of the plane
    i_size=0.05,
    j_size=0.05,
)
pl.add_mesh(xz_plane, color="pink", opacity=0.4, name="X-Z Plane")

# Manually visualize the two coils as circular loops
for z in np.linspace(-0.008, 0.008, 16):
    # Coil 2 (red)
    points2 = np.array([
        [coil_offset + z, 0.005 * np.cos(theta), 0.005 * np.sin(theta)]
        for theta in np.linspace(0, 2 * np.pi, 100)
    ])
    pl.add_lines(points2, color="red", width=8)

    # Coil 3 (green)
    points3 = np.array([
        [0.005 * np.cos(theta), coil_offset + z, 0.005 * np.sin(theta)]
        for theta in np.linspace(0, 2 * np.pi, 100)
    ])
    pl.add_lines(points3, color="green", width=8)

# Add a small sphere at the center (0, 0, 0)
sphere = pv.Sphere(radius=0.002, center=(0, 0, 0))
pl.add_mesh(sphere, color="yellow", specular=0.5, name="Center Sphere")

# Add glyphs (arrows) for field direction with uniform size
glyphs = grid.glyph(
    orient="B_vector",  # Use the vector field for orientation
    scale= True,  # Disable scaling by magnitude
    factor=0.00002,  # Set a constant size for the arrows
)
pl.add_mesh(
    glyphs,
    color="black",
    show_scalar_bar=False,  # Hide the glyph scalar bar
)

# Add streamlines and scalar bar for magnitude
legend_args = {
    "title": "B (mT)",
    "title_font_size": 20,
    "color": "black",
    "position_y": 0.25,
    "vertical": True,
}
pl.add_mesh(
    strl.tube(radius=0.0001),  # Make streamlines thinner
    cmap="hot",  # Blue-white-red colormap
    scalar_bar_args=legend_args,  # Legend arguments
)

# Set the camera position for better view and show the scene
pl.camera.position = (0.06, 0.06, 0.06)
pl.show()

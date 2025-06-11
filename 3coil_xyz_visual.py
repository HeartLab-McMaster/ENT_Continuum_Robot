import numpy as np
import magpylib as magpy
import pyvista as pv
from scipy.spatial.transform import Rotation as R
import os
import time
from functools import partial
import serial  # Import serial for ESP32 communication
import threading

import pygame
pygame.init()
pygame.joystick.init()
joy = pygame.joystick.Joystick(0)



# DONE: Need to add buttons for arrows, need to update coil colours/msgs based on arrow clicks, need to add update arrow function to move arrow based on button click, need to add states for button clickst to enable start and stop og vontinuous movement,  add serial comm port and commands (send as function)
# TODO: , , work on arduino code, maybe add sensor vals 

os.system("cls")

###################### Setup
skull = pv.read("Scull_geant_fix02.stl")
logo = pv.read("HeartLabLogo.png")

arrow_label_global = ["X pos","X neg", "Y pos", "Y neg", "Z pos", "Z neg", "XY pos", "XY neg", "XZ neg", "XYZ Z neg", "XYZ YZ neg"]
global tip_actor

###################### Setup Serial Communication
SERIAL_PORT = "COM3" 
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

###################### Send commands to ESP32
def esp_command(message):
    if esp32:
        try:
            esp32.write(message.encode())
            print(f"Command sent: {message}")
        except serial.SerialException:
            print("Serial write failed. Check ESP32 connection.")

###################### Stepper status update
def update_stepper_status(message):
    """Updates the stepper status text """
    status_text = f"Actuator Status: {message}"

    # Remove the old label if it exists
    if "stepper_text" in pl.actors:
        pl.remove_actor("stepper_text")

    # Add updated text
    pl.add_text(
        status_text,
        position=(10,560),
        font_size=12,
        color="black",
        name="stepper_text"
    )

###################### Update arrow positioning
def update_arrow (direction):
    global tip_actor

    # Remove old arrow
    if "tip" in pl.actors:
        pl.remove_actor("tip")
    
    # Create new arrow in the desired direction
    new_tip = pv.Arrow(start=(0,0,0), direction=direction, scale=0.02)
    tip_actor = pl.add_mesh(new_tip, color="black", name="tip")

####################### Solenoid visualization
def create_solenoid(plane, solenoid_current, solenoid_radius=0.1, solenoid_length=0.1, solenoid_turns=145, rotation_angle=0):
    # Create coil 1
    offset = 1  # Offset of 30cm for each coil to be away from center

    if plane == "z":
        coil1 = magpy.Collection()  # Create a collection to hold multiple loops
        for z in np.linspace(-solenoid_length / 2, solenoid_length / 2, solenoid_turns):  # Position loops along the z-axis
            winding = magpy.current.Circle(  # Define a single circular loop
                current=solenoid_current,  # Current in the loop (in amperes)
                diameter=2 * solenoid_radius,  # Diameter of the loop (in meters)
                position=(0, 0, z+offset),  # Position of the loop in 3D space
            )
            coil1.add(winding)  # Add the loop to the collection

        # Create PyVista representation of the coil(1) for visualization
        coil_points_Z = []  # List to hold the points of all loops
        for z in np.linspace(-0.16, -0.08, 16):  # Loop positions along the z-axis
            theta = np.linspace(0, 2 * np.pi, 100)  # Define angles for the circular loop
            x = 0.05 * np.cos(theta)  # x-coordinates of the loop
            y = 0.05 * np.sin(theta)  # y-coordinates of the loop
            z_points = np.full_like(theta, z)  # z-coordinates for the loop
            coil_points_Z.append(np.column_stack((x, y, z_points)))  # Combine points into 3D coordinates
        return coil_points_Z
    
    # Create Coil 2
    elif plane == "y":
        coil2 = magpy.Collection()
        for y in np.linspace(-solenoid_length / 2, solenoid_length / 2, solenoid_turns):  # Position loops along the z-axis
            winding = magpy.current.Circle(  # Define a single circular loop
                current=solenoid_current,  # Current in the loop (in amperes)
                diameter=2 * solenoid_radius,  # Diameter of the loop (in meters)
                position=(0, y+offset, 0),  # Position of the loop in 3D space
            )
            coil2.add(winding)  # Add the loop to the collection

        # Create PyVista representation of the coil(2) for visualization
        coil_points_Y = []  # List to hold the points of all loops
        for y in np.linspace(0.16, 0.08, 16):  # Loop positions along the z-axis
            theta = np.linspace(0, 2 * np.pi, 100)  # Define angles for the circular loop
            x = 0.05 * np.cos(theta)  # x-coordinates of the loop
            z = 0.05 * np.sin(theta)  # y-coordinates of the loop
            y_points = np.full_like(theta, y)  # z-coordinates for the loop
            coil_points_Y.append(np.column_stack((x, y_points, z)))  # Combine points into 3D coordinates
        return coil_points_Y

    elif plane == "x":
        coil3 = magpy.Collection()
        for x in np.linspace(-solenoid_length / 2, solenoid_length / 2, solenoid_turns):  # Position loops along the z-axis
            winding = magpy.current.Circle(  # Define a single circular loop
                current=solenoid_current,  # Current in the loop (in amperes)
                diameter=2 * solenoid_radius,  # Diameter of the loop (in meters)
                position=(x+offset, 0, 0),  # Position of the loop in 3D space
            )
            coil3.add(winding)  # Add the loop to the collection

        # Create PyVista representation of the coil(3) for visualization
        coil_points_X = []  # List to hold the points of all loops
        for x in np.linspace(0.16, 0.08, 16):  # Loop positions along the z-axis
            theta = np.linspace(0, 2 * np.pi, 100)  # Define angles for the circular loop
            x_points = np.full_like(theta, x)  # z-coordinates for the loop
            y = 0.05 * np.cos(theta)  # x-coordinates of the loop
            z = 0.05 * np.sin(theta)  # y-coordinates of the loop
            coil_points_X.append(np.column_stack((x_points, y, z)))  # Combine points into 3D coordinates
        return coil_points_X

# Coil Parameters
solenoid_radius = 0.1
solenoid_length = 0.1
solenoid_turns = 145
solenoid_current = 20

solenoid_x = create_solenoid("x",solenoid_current, solenoid_radius, solenoid_length, solenoid_turns)
solenoid_y = create_solenoid("y", solenoid_current, solenoid_radius, solenoid_length, solenoid_turns)
solenoid_z = create_solenoid("z",solenoid_current, solenoid_radius, solenoid_length, solenoid_turns)

# Create a 3D grid of points for magnetic field visualization
x = np.linspace(-0.2, 0.2, 10)  # Grid along the x-axis
y = np.linspace(-0.2, 0.2, 10)  # Grid along the y-axis
z = np.linspace(-0.2, 0.2, 10)  # Grid along the z-axis
grid_points = np.array(np.meshgrid(x, y, z)).T.reshape(-1, 3)  # Combine into a 3D grid

coil_linesX = pv.PolyData(np.vstack(solenoid_x))  # Create a PolyData object for the coil
coil_linesY = pv.PolyData(np.vstack(solenoid_y))  # Create a PolyData object for the coil
coil_linesZ = pv.PolyData(np.vstack(solenoid_z))



############################### Extend / Retract Functionality
def extend(state):
    if state:
        update_stepper_status("Extending")
        esp_command("EXT\n")
    else:
        update_stepper_status("Idle")
        esp_command("OFF\n")


def retract(state):
    if state:
        pl.add_mesh(coil_linesX, color="grey", line_width=3, label="Coil X")
        pl.add_mesh(coil_linesY, color="grey", line_width=3, label="Coil Y")
        pl.add_mesh(coil_linesZ, color="grey", line_width=3, label="Coil Z")
        update_stepper_status("Retracting")
        esp_command("RTR\n")
    else:
        update_stepper_status("Idle")
        esp_command("OFF\n")

def turn_Complete(state):
    if state:
        pl.add_mesh(coil_linesX, color="grey", line_width=3, label="Coil X")
        pl.add_mesh(coil_linesY, color="grey", line_width=3, label="Coil Y")
        pl.add_mesh(coil_linesZ, color="grey", line_width=3, label="Coil Z")
        esp_command("Coil OFF\n")
        
##################### 3D Joystick arrows / coil activation
def coil_activation(_, arrow_label):
    # green is going into to coil (attract towards coil) - DIR PIN
    # red is going out from coil (repel away from coil) - DIR PIN
    if arrow_label == arrow_label_global[0]:
        print(f"Activating coils for {arrow_label}")
        pl.add_mesh(coil_linesX, color="green", line_width=3, label="Coil X")
        pl.add_mesh(coil_linesY, color="grey", line_width=3, label="Coil Y")
        pl.add_mesh(coil_linesZ, color="grey", line_width=3, label="Coil Z")
        update_arrow([1,0,0])
        esp_command(f"{arrow_label}\n")

    if arrow_label == arrow_label_global[1]:
        print(f"Activating coils for {arrow_label}")
        pl.add_mesh(coil_linesX, color="red", line_width=3, label="Coil X")
        pl.add_mesh(coil_linesY, color="grey", line_width=3, label="Coil Y")
        pl.add_mesh(coil_linesZ, color="grey", line_width=3, label="Coil Z")
        update_arrow([-1,0,0])
        esp_command(f"{arrow_label}\n")
        
    if arrow_label == arrow_label_global[2]:
        print(f"Activating coils for {arrow_label}")
        pl.add_mesh(coil_linesX, color="grey", line_width=3, label="Coil X")
        pl.add_mesh(coil_linesY, color="green", line_width=3, label="Coil Y")
        pl.add_mesh(coil_linesZ, color="grey", line_width=3, label="Coil Z")
        update_arrow([0,1,0])
        esp_command(f"{arrow_label}\n")
        
    if arrow_label == arrow_label_global[3]:
        print(f"Activating coils for {arrow_label}")
        pl.add_mesh(coil_linesX, color="grey", line_width=3, label="Coil X")
        pl.add_mesh(coil_linesY, color="red", line_width=3, label="Coil Y")
        pl.add_mesh(coil_linesZ, color="grey", line_width=3, label="Coil Z")
        update_arrow([0,-1,0])
        esp_command(f"{arrow_label}\n")
        
    if arrow_label == arrow_label_global[4]:
        print(f"Activating coils for {arrow_label}")
        pl.add_mesh(coil_linesX, color="grey", line_width=3, label="Coil X")
        pl.add_mesh(coil_linesY, color="grey", line_width=3, label="Coil Y")
        pl.add_mesh(coil_linesZ, color="red", line_width=3, label="Coil Z")
        update_arrow([0,0,1])
        esp_command(f"{arrow_label}\n")
        
    if arrow_label == arrow_label_global[5]:
        print(f"Activating coils for {arrow_label}")
        pl.add_mesh(coil_linesX, color="grey", line_width=3, label="Coil X")
        pl.add_mesh(coil_linesY, color="grey", line_width=3, label="Coil Y")
        pl.add_mesh(coil_linesZ, color="green", line_width=3, label="Coil Z")
        update_arrow([0,0,-1])
        esp_command(f"{arrow_label}\n")
        
    if arrow_label == arrow_label_global[6]:
        print(f"Activating coils for {arrow_label}")
        pl.add_mesh(coil_linesX, color="green", line_width=3, label="Coil X")
        pl.add_mesh(coil_linesY, color="green", line_width=3, label="Coil Y")
        pl.add_mesh(coil_linesZ, color="grey", line_width=3, label="Coil Z")
        update_arrow([1,1,0])
        esp_command(f"{arrow_label}\n")
        
    if arrow_label == arrow_label_global[7]:
        print(f"Activating coils for {arrow_label}")
        pl.add_mesh(coil_linesX, color="green", line_width=3, label="Coil X")
        pl.add_mesh(coil_linesY, color="red", line_width=3, label="Coil Y")
        pl.add_mesh(coil_linesZ, color="grey", line_width=3, label="Coil Z")
        update_arrow([1,-1,0])
        esp_command(f"{arrow_label}\n")
        
    if arrow_label == arrow_label_global[8]:
        print(f"Activating coils for {arrow_label}")
        pl.add_mesh(coil_linesX, color="green", line_width=3, label="Coil X")
        pl.add_mesh(coil_linesY, color="grey", line_width=3, label="Coil Y")
        pl.add_mesh(coil_linesZ, color="green", line_width=3, label="Coil Z")
        update_arrow([1,0,-1])
        esp_command(f"{arrow_label}\n")

    if arrow_label == arrow_label_global[9]:
        print(f"Activating coils for {arrow_label}")
        pl.add_mesh(coil_linesX, color="green", line_width=3, label="Coil X")
        pl.add_mesh(coil_linesY, color="green", line_width=3, label="Coil Y")
        pl.add_mesh(coil_linesZ, color="green", line_width=3, label="Coil Z")
        update_arrow([1,1,-1])
        esp_command(f"{arrow_label}\n")
        
    if arrow_label == arrow_label_global[10]:
        print(f"Activating coils for {arrow_label}")
        pl.add_mesh(coil_linesX, color="green", line_width=3, label="Coil X")
        pl.add_mesh(coil_linesY, color="red", line_width=3, label="Coil Y")
        pl.add_mesh(coil_linesZ, color="green", line_width=3, label="Coil Z")
        update_arrow([1,-1,-1])
        esp_command(f"{arrow_label}\n")


###################### Coil simulator
def simulate_coil(solenoid_current=15):
    """Runs the solenoid simulation and visualization."""


    # Add the grey coil mesh to the scene
    # pl.add_mesh(coil_linesX, color="blue", line_width=3, label="Coil 1")
    # pl.add_mesh(coil_linesY, color="red", line_width=3, label="Coil 2")
    # pl.add_mesh(coil_linesZ, color="green", line_width=3, label="Coil 3")
    pl.add_mesh(coil_linesX, color="grey", line_width=3, label="Coil X")
    pl.add_mesh(coil_linesY, color="grey", line_width=3, label="Coil Y")
    pl.add_mesh(coil_linesZ, color="grey", line_width=3, label="Coil Z")
    
    # Heart Lab logo not working - TODO
    # pl.add_logo_widget(logo, (1,1), (1,1), 1)

    ###################### SKULL
    # Adding skull model 
    skull_copy = skull.copy()
    # Scale down STL
    skull_copy.points *= 0.00072  
    # Rotate skull Z-axis to X-axis
    r = R.from_euler('y', 90, degrees=True)  
    # Rotate -90° around Y
    skull_copy.points = r.apply(skull_copy.points)
    # Centering skull to coils
    skull_copy.points[:, 0] -= 0.035  
    pl.add_mesh(skull_copy, color="tan", opacity=0.15)


    #################### Robot tip
    # Create an tip at the center pointing in the Z-direction
    tip_start = (0, 0, 0)
    tip_direction = (0, 0, -1)  # Initial direction
    tip_scale = 0.02  # Scale for visibility
    tip = pv.Arrow(start=tip_start, direction=tip_direction, scale=tip_scale)
    tip_actor = pl.add_mesh(tip, color="black", name="tip")


    
    # Coil X
    joystick_size = 0.02
    X_pos =  pv.Arrow(start=(0.02,0,0), direction=(1,0,0), scale=joystick_size)
    X_pos_actor = pl.add_mesh(X_pos, color="cyan", name=arrow_label_global[0])
    coil_activation_callback = partial(coil_activation, arrow_label=arrow_label_global[0])
    pl.add_sphere_widget(callback=coil_activation_callback, center=(0.02 + joystick_size, 0, 0), radius=0.002, test_callback=False)

    X_neg = pv.Arrow(start=(-0.02,0,0), direction=(-1,0,0), scale=joystick_size)
    X_neg_actor = pl.add_mesh(X_neg, color="cyan", name=arrow_label_global[1])
    coil_activation_callback = partial(coil_activation, arrow_label=arrow_label_global[1])
    pl.add_sphere_widget(callback=coil_activation_callback, center=(-0.02 - joystick_size, 0, 0), radius=0.002, test_callback=False)

    # Coil Y
    Y_pos =  pv.Arrow(start=(0,0.02,0), direction=(0,1,0), scale=joystick_size)
    Y_pos_actor = pl.add_mesh(Y_pos, color="cyan", name=arrow_label_global[2])
    coil_activation_callback = partial(coil_activation, arrow_label=arrow_label_global[2])
    pl.add_sphere_widget(callback=coil_activation_callback, center=(0, 0.02 + joystick_size, 0), radius=0.002, test_callback=False)

    Y_neg =  pv.Arrow(start=(0,-0.02,0), direction=(0,-1,0), scale=joystick_size)
    Y_neg_actor = pl.add_mesh(Y_neg, color="cyan", name=arrow_label_global[3])
    coil_activation_callback = partial(coil_activation, arrow_label=arrow_label_global[3])
    pl.add_sphere_widget(callback=coil_activation_callback, center=(0, -0.02 - joystick_size, 0), radius=0.002, test_callback=False)

    # Coil Z
    Z_pos =  pv.Arrow(start=(0,0,0.02), direction=(0,0,1), scale=joystick_size)
    Z_pos_actor = pl.add_mesh(Z_pos, color="cyan", name=arrow_label_global[4])
    coil_activation_callback = partial(coil_activation, arrow_label=arrow_label_global[4])
    pl.add_sphere_widget(callback=coil_activation_callback, center=(0,0,0.02 + joystick_size), radius=0.002, test_callback=False)

    Z_neg = pv.Arrow(start=(0,0,-0.02), direction=(0,0,-1), scale=joystick_size)
    Z_pos_actor = pl.add_mesh(Z_neg, color="cyan", name=arrow_label_global[5])
    coil_activation_callback = partial(coil_activation, arrow_label=arrow_label_global[5])
    pl.add_sphere_widget(callback=coil_activation_callback, center=(0,0,-0.02 - joystick_size), radius=0.002, test_callback=False)


    # Extra Arrows
    XY_pos = pv.Arrow(start=(0.02,0.02,0), direction=(1,1,0), scale=joystick_size)
    XY_pos_actor = pl.add_mesh(XY_pos, color="cyan", name=arrow_label_global[6])
    coil_activation_callback = partial(coil_activation, arrow_label=arrow_label_global[6])
    pl.add_sphere_widget(callback=coil_activation_callback, center=(0.03414, 0.03414, 0), radius=0.002, test_callback=False)


    XY_neg = pv.Arrow(start=(0.02,-0.02,0), direction=(1,-1,0), scale=joystick_size)
    XY_neg_actor = pl.add_mesh(XY_neg, color="cyan", name=arrow_label_global[7])
    coil_activation_callback = partial(coil_activation, arrow_label=arrow_label_global[7])
    pl.add_sphere_widget(callback=coil_activation_callback, center=(0.03414,-0.03414, 0), radius=0.002, test_callback=False)

    #### CHANGE CENTERS FOR FOLLOWING CODE - TODO

    #XZ_pos = pv.Arrow(start=(0.02,0,0.02), direction=(1,0,1), scale=joystick_size)
    #XZ_pos_actor = pl.add_mesh(XZ_pos, color="cyan", name="XZ pos")
    XZ_neg = pv.Arrow(start=(0.02, 0, -0.02), direction=(1,0,-1), scale=joystick_size)
    XZ_neg_actor = pl.add_mesh(XZ_neg, color="cyan", name=arrow_label_global[8])
    coil_activation_callback = partial(coil_activation, arrow_label=arrow_label_global[8])
    pl.add_sphere_widget(callback=coil_activation_callback, center=(0.0341, 0, -0.0341), radius=0.002, test_callback=False)


    # XYZ_Xneg = pv.Arrow(start=(-0.02,0.02,0.02), direction=(-1,1,1), scale=joystick_size)
    # XYZ_Xneg_actor = pl.add_mesh(XYZ_Xneg, color="cyan", name="XYZ X neg")
    # XYZ_Yneg = pv.Arrow(start=(0.02,-0.02,0.02), direction=(1,-1,1), scale=joystick_size)
    # XYZ_Yneg_actor = pl.add_mesh(XYZ_Yneg, color="cyan", name="XYZ Y neg")
    # Need to fix - TODO
    XYZ_Zneg = pv.Arrow(start=(0.02,0.02,-0.02), direction=(1,1,-1), scale=joystick_size)
    XYZ_Zneg_actor = pl.add_mesh(XYZ_Zneg, color="cyan", name=arrow_label_global[9])
    coil_activation_callback = partial(coil_activation, arrow_label=arrow_label_global[9])
    pl.add_sphere_widget(callback=coil_activation_callback, center=(0.0341, 0.0341, -0.0341), radius=0.002, test_callback=False)


    # XYZ_XYneg = pv.Arrow(start=(-0.02,-0.02,0.02), direction=(-1,-1,1), scale=joystick_size)
    # XYZ_XYneg_actor = pl.add_mesh(XYZ_XYneg, color="cyan", name="XYZ XY neg")
    # Need to fix - TODO
    XYZ_YZneg = pv.Arrow(start=(0.02,-0.02,-0.02), direction=(1,-1,-1), scale=joystick_size)
    XYZ_YZneg_actor = pl.add_mesh(XYZ_YZneg, color="cyan", name=arrow_label_global[10])
    coil_activation_callback = partial(coil_activation, arrow_label=arrow_label_global[10])
    pl.add_sphere_widget(callback=coil_activation_callback, center=(0.0341, -0.0341, -0.0341), radius=0.002, test_callback=False)

    # XYZ_XZneg = pv.Arrow(start=(-0.02,0.02,-0.02), direction=(-1,1,-1), scale=joystick_size)
    # XYZ_XZneg_actor = pl.add_mesh(XYZ_XZneg, color="cyan", name="XYZ XZ neg")


    # Extend / Retract Buttons
    pl.add_checkbox_button_widget(extend, value = False, position=(1600, 10), size=30, border_size=2, color_on="blue") 
    pl.add_text("Extend", position=(1565, 40), font_size=14, color="black")
    
    pl.add_checkbox_button_widget(retract, value = False, position=(1400, 10), size=30, border_size=2, color_on="blue") 
    pl.add_text("Retract", position=(1365, 40), font_size=14, color="black")

    pl.add_checkbox_button_widget(turn_Complete, value = False, position=(1150, 10), size=30, border_size=2, color_on="red") 
    pl.add_text("Completed turn", position=(1075, 40), font_size=14, color="black")
   
    ######################### Emergency Stop Mechanism
    def stop_callback(state):
        if state:  # If button is clicked (ON)
            print("EMERGENCY STOP ACTIVATED! Turning off all coils.")
            esp_command("SHUT OFF\n")
            pl.add_mesh(coil_linesX, color="grey", line_width=3, label="Coil X")
            pl.add_mesh(coil_linesY, color="grey", line_width=3, label="Coil Y")
            pl.add_mesh(coil_linesZ, color="grey", line_width=3, label="Coil Z")
            update_arrow([0, 0, -1])
            msg = pl.add_text("EMERGENCY STOPPED!", position="upper_left", font_size=20, color="red")
            update_stepper_status("OFF")
            pl.render()
            time.sleep(5)  # Show the message briefly
            pl.remove_actor(msg)  # Remove the text
            pl.render()
            

    button_position = (10, 10)
    pl.add_checkbox_button_widget(stop_callback, value = False, position=button_position, size=30, border_size=2, color_on="red") 
    pl.add_text("EMERGENCY STOP", position=(button_position[0] + 40, button_position[1] - 5), font_size=14, color="red")
    
    ############################ Camera settings

    # Add axes and set the camera position
    pl.add_axes()  # Add axes to the scene
    pl.camera_position = [
        (0, 0, 1),  # camera location: in front of the origin along -Z
        (0, 0, 0),   # look at the origin
        (1, 0, 0),   # X-axis is up
    ]

    # Show the 3D scene
    pl.show(interactive_update=True)


def joy_thread():
    """Thread to handle joystick events."""
    while True:
        for event in pygame.event.get():
            print(event)
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0:  # Assuming button 0 is the emergency stop
                    print("Emergency Stop Activated")
                    # stop_callback(True)  # Call the emergency stop function
            elif event.type == pygame.JOYAXISMOTION:
                # Handle joystick axis motion if needed
                pass

if __name__ == "__main__":
    # Create a PyVista plotting scene
    pl = pv.Plotter()
    update_stepper_status("Ready")
    # Start the joystick event thread
    # joystick_thread = threading.Thread(target=joy_thread, daemon=True)
    # joystick_thread.start()

    simulate_coil()
    while True:
        for event in pygame.event.get():
            # print(event)
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0:  # Assuming button 0 is the emergency stop
                    print("Stop coils")
                    turn_Complete(True)
                elif event.button == 2:
                    print("Retract coils")
                    retract(True)
                elif event.button == 1:
                    print("Extend coils")
                    extend(True)
                elif event.button == 3:
                    print("Stopping catheter movement")
                    extend(False)
            elif event.type == pygame.JOYHATMOTION:
                if event.value == (1, 0):
                    print("Joystick moved right")
                    coil_activation(None, "Y neg")
                elif event.value == (-1, 0):
                    print("Joystick moved left")
                    coil_activation(None, "Y pos")
                elif event.value == (0, 1):
                    print("Joystick moved up")
                    coil_activation(None, "X pos")
                elif event.value == (0, -1):
                    print("Joystick moved down")
                    coil_activation(None, "X neg")
            elif event.type == pygame.JOYAXISMOTION:
                # Handle joystick axis motion if needed
                pass
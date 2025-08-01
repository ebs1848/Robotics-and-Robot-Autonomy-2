import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.patches import Rectangle
import matplotlib.patches as patches

import numpy as np

# Function to plot the robot body
def plot_robot(ax, robot, prev_pos):

    if prev_pos[0] is not None:
        prev_pos[0].remove()  # Remove the previous robot plot
    if prev_pos[1] is not None:
        prev_pos[1].remove()  # Remove the previous wheel plot
    if prev_pos[2] is not None:
        prev_pos[2].remove()  # Remove the previous wheel plot

    # Plot the robot at current position
    robot_pos = Circle((robot.position.x, robot.position.y), radius=10,
                          fill=True, alpha=0.5, edgecolor='black', linewidth=2)
    ax.add_patch(robot_pos)

    # Call the plot_wheel function
    wheel_posr, wheel_posl = plot_wheel(ax, robot)

    return [robot_pos, wheel_posr, wheel_posl]  # Return the new rectangle

# Function to plot the green filled rectangle with a midpoint on the right side of the red rectangle
def plot_wheel(ax, robot):

    # Coordinates of ??? in ??? frame
    x_wheelr_R = -4.5 
    y_wheelr_R = -8
    x_wheell_R = -4.5 
    y_wheell_R =  6


    # Coordinates of ??? in ??? frame
    theta=np.radians(robot.position.theta)
    x_wheelr_W = robot.position.x + x_wheelr_R * np.cos(theta) - y_wheelr_R * np.sin(theta)
    y_wheelr_W = robot.position.y + x_wheelr_R * np.sin(theta) + y_wheelr_R * np.cos(theta)
    x_wheell_W = robot.position.x + x_wheell_R * np.cos(theta) - y_wheell_R * np.sin(theta)
    y_wheell_W = robot.position.y + x_wheell_R * np.sin(theta) + y_wheell_R * np.cos(theta)

    # Plot the wheel at correct position and steering angle
    wheel_posr = Rectangle((x_wheelr_W, y_wheelr_W), 9, 2, angle=np.degrees(theta),
                      facecolor='green', edgecolor='none')
    ax.add_patch(wheel_posr)
    wheel_posl = Rectangle((x_wheell_W, y_wheell_W), 9, 2, angle=np.degrees(theta),
                      facecolor='green', edgecolor='none')
    ax.add_patch(wheel_posl)

    return wheel_posr,wheel_posl


def simulation(fig, ax, robot, prev_pos):

     # Call the plot_robot function to update the robot position
    prev_pos = plot_robot(ax, robot, prev_pos)
    #prev_lidar = plot_lidar(ax,lidar,prev_lidar)
    # Plot the traveled path
    line = ax.plot([robot.prev_position.prev_x, robot.position.x], [robot.prev_position.prev_y, robot.position.y], 'r.', lw=1, ls='dotted')
    return prev_pos


#####################################PLOT SIM ENVIRONMENT######################################
def plot_environment(fig,ax,obstacles,target_position):
    # Set up the empty plot with specified boundaries and aspect ratio
    ax.set_xlim(-30, 200)  # Set x-axis boundaries
    ax.set_ylim(-30, 200)  # Set y-axis boundaries
    ax.set_aspect('equal', adjustable='box')  # Preserve aspect ratio

    # Add gridlines
    ax.set_xticks(np.arange(-30, 200, 10))
    ax.set_xticks(np.arange(-30, 200, 2), minor=True)
    ax.set_yticks(np.arange(-30, 200, 10))
    ax.set_yticks(np.arange(-30, 200, 2), minor=True)

    ax.grid(which='minor', alpha=0.2)
    ax.grid(which='major', alpha=0.5)


    # Add labels
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')

    for obstacle in obstacles:
        ax.add_patch(patches.PathPatch(obstacle, edgecolor='black', facecolor='gray'))
    
    ax.add_patch(Circle(target_position, radius=1, fill=True, color='red', linewidth=2))

    return fig, ax
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from simulations import Robot, Scenario

def main(create_gif=False):
    plt.style.use('seaborn-whitegrid')
    fig, ax = plt.subplots()
    ax.set_title('Robot Brownian Motion')
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.grid(False)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_xticks(np.arange(-2, 2.5, 0.5))  
    ax.set_yticks(np.arange(-2, 2.5, 0.5))
    ax.set_xticklabels(np.arange(-2, 2.5, 0.5), fontsize=8)
    ax.set_yticklabels(np.arange(-2, 2.5, 0.5), fontsize=8)
    ax.tick_params(axis='both', which='major', labelsize=8)


    bounds = (-1.5, 1.5, -1.5, 1.5)
    scenario = Scenario(ax, bounds)
    robot = Robot(ax, scenario, position=(0, 0))

    def update(frame):
        robot.update()
        return robot.circle, robot.arrow_obj

    ani = FuncAnimation(fig, update, frames=np.arange(0, 1000), interval=50, blit=True)

    if create_gif:
        ani.save("robot_motion.gif", writer='pillow', fps=20)
    else:
        plt.show()

if __name__ == "__main__":
    main()
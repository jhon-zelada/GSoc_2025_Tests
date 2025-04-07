import random
import numpy as np
from matplotlib.patches import Circle, FancyArrow, Rectangle



class Scenario:
    def __init__(self, ax, bounds):
        self.ax = ax
        self.left, self.right, self.bottom, self.top = bounds
        self.draw_bounds()

    def draw_bounds(self):
        rect = Rectangle((self.left, self.bottom), self.right - self.left, self.top - self.bottom,
                         fill=None, edgecolor='black', linewidth=2)
        self.ax.add_patch(rect)

    def is_collision(self, x, y, radius):
        #Checks for collision with the bounds of the scenario
        return (
            x - radius < self.left or x + radius > self.right or
            y - radius < self.bottom or y + radius > self.top
        )

class Robot:
    def __init__(self, ax, scenario, position, heading=0, speed=0.05, radius=0.1):
        self.ax = ax
        self.scenario = scenario
        self.position = np.array(position, dtype=float)
        self.heading = heading
        self.speed = speed
        self.radius = radius

        self.rotating = False
        self.bouncing = False
        self.rotate_time_remaining = 0
        self.bounce_time_remaining = 5 
        self.bounce_angle = 0
        self.bump_vector = np.zeros(2)
        self.rotate_direction = 1  # 1 or -1

        self.circle = Circle(self.position, self.radius, color='blue')
        self.arrow = FancyArrow(*self.position, 0.1, 0.1, width=0.03, color='red')
        self.ax.add_patch(self.circle)
        self.arrow_obj = self.ax.add_patch(self.arrow)

    def start_bounce(self):
        bump_strength = 0.005
        self.bump_vector = -bump_strength * np.array([np.cos(self.heading), np.sin(self.heading)])*np.cos(self.bounce_angle)
        self.position += self.bump_vector

    def update(self):
        if self.bouncing:
            self.position += self.bump_vector
            self.bounce_time_remaining -= 1
            self.bounce_angle += np.pi / 5
            self.start_bounce()
            
            if self.bounce_time_remaining <= 0:
                self.bouncing = False
                self.rotating = True
                self.rotate_time_remaining = random.randint(10, 40)
                self.rotate_direction = random.choice([-1, 1])

        elif self.rotating:
            self.heading += self.rotate_direction * np.pi / 20
            self.rotate_time_remaining -= 1
            if self.rotate_time_remaining <= 0:
                self.rotating = False
        else:
            dx = self.speed * np.cos(self.heading)
            dy = self.speed * np.sin(self.heading)
            new_pos = self.position + np.array([dx, dy])

            if self.scenario.is_collision(new_pos[0], new_pos[1], self.radius):
                self.bouncing = True
                self.bounce_angle = 0
                self.bounce_time_remaining = 5 
            else:
                self.position = new_pos

        self.update_draw()

    def update_draw(self):
        self.circle.center = self.position
        self.arrow_obj.remove()
        arrow_length = 0.3
        dx = arrow_length * np.cos(self.heading)
        dy = arrow_length * np.sin(self.heading)
        self.arrow_obj = FancyArrow(self.position[0], self.position[1], dx, dy, width=0.03, color='red')
        self.ax.add_patch(self.arrow_obj)


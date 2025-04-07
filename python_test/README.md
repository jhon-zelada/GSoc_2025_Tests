# Robot Brownian Motion Simulation

This project simulates the Brownian motion of a robot within a bounded 2D scenario using Python and Matplotlib. The robot moves randomly, bounces off boundaries, and rotates to change direction.

## Project Structure

```
./python_test/
├── app.py          # Main script to run the simulation
├── simulations.py  # Contains the Robot and Scenario classes
├── __init__.py     # Empty file to mark the directory as a package
```

### Files Overview

1. **`app.py`**  
    - Entry point of the simulation.  
    - Sets up the Matplotlib figure and animation.  
    - Runs the simulation and optionally saves it as a GIF.

2. **`simulations.py`**  
    - Contains the `Scenario` class, which defines the simulation bounds.  
    - Contains the `Robot` class, which handles the robot's movement, collision, and bouncing behavior.

3. **`__init__.py`**  
    - Marks the directory as a Python package.

## How to Run

1. Install the required dependencies:
    ```bash
    pip install numpy matplotlib
    ```

2. Run the simulation:
    ```bash
    python app.py
    ```

3. To save the simulation as a GIF, modify the `main` function in `app.py`:
    ```python
    main(create_gif=True)
    ```

## Features

- **Random Movement**: The robot moves in random directions.
- **Collision Detection**: The robot detects and reacts to collisions with the scenario boundaries.
- **Bouncing Behavior**: The robot bounces off boundaries and rotates to change direction.
- **Visualization**: The simulation is visualized using Matplotlib.

## Example Output

- **Live Simulation**: Displays the robot's motion in real-time.
- **GIF Output**: Saves the simulation as a GIF file (`robot_motion.gif`).

## Customization

- **Bounds**: Modify the `bounds` in `app.py` to change the simulation area.
- **Robot Speed**: Adjust the `speed` parameter in the `Robot` class.
- **Frames**: Change the number of frames in the `FuncAnimation` call in `app.py`.

## License

This project is licensed under the MIT License.  

## Robot motion DEMO

![Robot Motion](python_test/robot_motion.gif)
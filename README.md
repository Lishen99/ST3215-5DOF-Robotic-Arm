# STServo Python Robot Arm Control

This project provides a Python-based control system for a 5-DOF robot arm utilizing ST3215 motors. It features both analytical and Jacobian-based inverse kinematics (IK) implementations, a graphical user interface (GUI) for real-time control and visualization, and advanced motion planning capabilities.

## Features

*   **Real-time Motor Control**: Direct communication with ST3215 motors for position, velocity, voltage, and temperature feedback.
*   **Graphical User Interface (GUI)**: A Tkinter-based interface for:
    *   Connecting to the robot arm via a COM port.
    *   Scanning and initializing connected motors.
    *   Manual joint control (setting individual motor positions).
    *   Real-time 3D visualization of the robot arm's current posture and end-effector coordinates.
    *   Cartesian control (moving the end-effector to a specified X, Y, Z position).
    *   Axis Sweep functionality (moving the end-effector back and forth along X, Y, or Z axes).
*   **Jacobian-based Velocity Control**: Implements a velocity-based inverse kinematics approach for smooth, continuous path following in Cartesian space.
*   **Singularity Handling**: Utilizes Damped Least-Squares (DLS) to maintain stability and prevent erratic behavior when the arm approaches kinematic singularities.
*   **Posture Optimization**: Employs null-space projection to gently guide the arm towards a neutral, "stretched-out" posture, optimizing reach and avoiding awkward configurations.
*   **Joint Limit Protection**: Actively monitors and prevents joints from exceeding their physical limits during motion.

## Project Structure

*   `jacob_ik/`:
    *   `arm_controller.py`: The core control logic, handling motor communication, motion planning, and IK/FK integration.
    *   `kinematics.py`: Defines the robot's kinematic model (forward and inverse kinematics, Jacobian calculation) and advanced control algorithms (DLS, manipulability gradient - *currently disabled for stability*).
    *   `motor_gui.py`: The Tkinter GUI application for user interaction and visualization.
    *   `calibrate_servos.py`: (Assumed to be for initial motor calibration).
    *   `servo_limits.json`: Configuration file defining the physical limits and calibration parameters for each servo motor.
*   `analytical_ik/`:
    *   Contains an alternative implementation of the robot arm control, likely using an analytical inverse kinematics approach. This folder serves as a reference or an alternative control method.
*   `stservo-env/`: The Python virtual environment for the project.
*   `STServo_Python/`: Contains the low-level STServo SDK files and communication protocols.
*   `requirements.txt`: Lists all Python dependencies required for the project.
*   `backups/`: Contains older versions or backups of project files.
*   `arm_design_files/`: Contains SolidWorks part files and STL models for the robot arm's mechanical components.

## Setup Instructions

1.  **Clone the Repository**:
    ```bash
    git clone <your-repository-url>
    cd STServo_Python
    ```

2.  **Create and Activate Virtual Environment**:
    It is highly recommended to use a virtual environment to manage dependencies.
    ```bash
    python -m venv stservo-env
    # On Windows:
    .\stservo-env\Scripts\activate
    # On macOS/Linux:
    source stservo-env/bin/activate
    ```

3.  **Install Dependencies**:
    ```bash
    pip install -r requirements.txt
    ```

4.  **Connect the Robot Arm**:
    Ensure your ST3215 robot arm is physically connected to your computer via a USB-to-serial adapter. Note the COM port it is connected to (e.g., `COM3` on Windows, `/dev/ttyUSB0` on Linux).

5.  **Run the GUI**:
    Navigate to the `jacob_ik` directory and run the main GUI application:
    ```bash
    cd jacob_ik
    python motor_gui.py
    ```
    Inside the GUI, select your COM port and click "Open & Scan" to connect to the motors.

## Known Issues & Future Work (Jacobian Controller)

The `jacob_ik` controller, while advanced, still has some areas for improvement:

*   **Path Straightness**: Despite using DLS and null-space control, the arm's end-effector path during Cartesian movements (especially sweeps) may not be perfectly straight, exhibiting minor deviations or "dips."
*   **Damping Factor Tuning**: The current damping factor (`0.4`) in `kinematics.py` is a general value. Optimal performance may require fine-tuning this parameter based on the arm's physical characteristics and desired responsiveness.
*   **Joint Velocity Distribution**: There's an ongoing observation that joint velocities might not be optimally distributed, potentially contributing to non-straight paths or inefficient motion. Further investigation into the Jacobian pseudoinverse calculation and its interaction with joint limits is needed.
*   **"Quivering" in Certain Configurations**: In some specific target positions or during sweeps through certain regions of the workspace, the arm may exhibit a "pecking" or "quivering" motion, indicating the controller is struggling to find a stable solution.

## Credits

*   This project extensively utilizes the STServo SDK files and environment setup provided by **Waveshare**.
*   Special thanks to Waveshare for their hardware and software support, which forms the foundation for motor communication.

## License

This project is licensed under the [Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0) License](https://creativecommons.org/licenses/by-nc-sa/4.0/).
You are free to share and adapt the material for non-commercial purposes, provided appropriate credit is given and any adaptations are shared under the same license.

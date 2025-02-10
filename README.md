# Quadcopter Altitude Control with PID

## Introduction
This repository contains a Python implementation of a PID controller for stabilizing the altitude of a quadcopter. The controller manages real-world challenges such as sensor noise, actuator constraints, and external disturbances like wind gusts.

## Project Structure
- `quadcopter_pid.py`: Main script implementing the quadcopter simulation with PID control.
- `README.md`: This file, providing an overview and instructions.

## Dependencies
- `numpy`
- `matplotlib`
- `control`

Install dependencies using pip:
```bash
pip install numpy matplotlib control
```

## Usage
1. Clone the repository:
   ```bash
   git clone https://github.com/SAI-SRIVARDHAN-REDDY-LINGALA/Quadcopter-Altitude-Control-with-PID.git
   ```
   
2. Navigate to the project directory:
   ```bash
   cd Quadcopter-Altitude-Control-with-PID
   ```
   
3. Run the simulation:
   ```bash
   python PID_QUADRACOPTER.py
   ```

## Results
- The PID_QUADRACOPTER.py` script simulates the quadcopter's altitude control over a specified time period.
- Two plots are generated:
  - **Altitude Plot**: Shows the quadcopter's altitude over time compared to the desired altitude.
  - **Control Effort Plot**: Displays the thrust input (control effort) applied by the PID controller.

## Challenges and Enhancements
- **Complex Dynamics**: Addressing nonlinearities and coupled system behaviors.
- **Environmental Factors**: Handling disturbances like wind gusts and sensor noise.
- **Future Enhancements**: Adaptive control strategies, advanced state estimation techniques (LQR, Kalman Filters), and AI-driven controllers for autonomy.

## Contributing
Contributions are welcome! Please fork the repository and create a pull request for any improvements or bug fixes.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

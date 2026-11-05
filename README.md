# TurtleBot3 Visual Waypoint Navigation and Docking

**Project:** Track A 

**Platform:** TurtleBot3 Waffle Pi (MuJoCo Simulation)

## 1. Project Overview

This project develops an autonomous navigation and visual docking system for a TurtleBot3 robot using MuJoCo simulation.

The robot uses wheel encoders and a camera for EKF based localisation, a state feedback controller for waypoint navigation and a 2D LiDAR for obstacle avoidance. It then uses ArUco marker detection to approach a docking station.

## 2. Project Structure

 File Description 

 `main.py` : Runs and integrates the complete simulation.

 `config.py` : Robot parameters, sensor noise and controller settings. 

 `model_verification.py` : Controllability and observability analysis. 

 `src/model/` : Robot motion and state space models.

 `src/sensors/` : Wheel encoder and LiDAR simulation.

 `src/perception/` : ArUco marker detection.

 `src/estimation/` : Wheel odometry and EKF localisation.

 `src/control/` : Waypoint navigation, obstacle avoidance and visual docking. 

 `src/utils/` : Coordinate transformations, visualisation and evaluation. 

 `models/` : MuJoCo robot and environment models. 

 `tools/` : Scripts for summarising experimental results. 

 `results/` : Saved simulation data and evaluation results. 

## 3. Installation

**Requirements:**
 Python 3
 Windows with PowerShell
 Dependencies listed in `requirements.txt`

Clone or download the repository and open a terminal in the project directory.

Create and activate a virtual environment:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
```

Install the required dependencies:

```powershell
pip install -r requirements.txt
```

## 4. Running the Simulation

Run the complete simulation:

```powershell
python main.py
```

The program opens the MuJoCo simulation, camera view and LiDAR display. The robot performs waypoint navigation, obstacle avoidance and visual docking.

After the simulation finishes, close the MuJoCo viewer normally. Evaluation results are automatically saved as CSV and JSON files in the `results/` folder.

To verify the state space model:

```powershell
python model_verification.py
```

### Reproducibility

The simulation supports fixed random seeds. To reproduce a run with a specific seed:

```powershell
$env:RUN_SEED="2026"
python main.py
```

The experiments use seeds 2026–2030. Sensor noise and obstacle configurations can be adjusted through `config.py`, `main.py` and the MuJoCo scene files. Experimental summaries can be generated using the scripts in `tools/`.
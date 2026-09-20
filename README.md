# Autonomous Mobile Robot (AMR) Navigation Engine

A Python-based navigation engine for differential-drive robots featuring 8-connected A* path planning, unicycle kinematics, simulated 32-ray LiDAR, and motor PWM telemetry generation.

## Project Structure
- `core/kinematics.py`: Robot motion models and LiDAR raycasting.
- `planning/astar.py`: A* path planning with obstacle clearance.
- `telemetry/motor_pwm.py`: Telemetry serialization and checksum generation.
- `visualizer.py`: Real-time Matplotlib animation dashboard.
- `main.py`: Main execution script.
- `tests/`: Automated unit tests.

## Setup & Installation

Clone repository:
```bash
git clone [https://github.com/Masud24bec10113/amr-navigation-engine.git](https://github.com/Masud24bec10113/amr-navigation-engine.git)
cd amr-navigation-engine
```

Install dependencies:
```bash
pip install -r requirements.txt
```

## How to Run

- Run unit tests:
```bash
python -m pytest
```

- Run main controller:
```bash
python main.py
```

- Run main with custom coordinates:
```bash
python main.py --start 1.0 1.0 --goal 25.0 25.0
```

- Run standalone visualizer:
```bash
python visualizer.py
```

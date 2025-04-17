# CSS-CAV
Repo for further implementation of CAD outlined in USENIX security 2024 paper "On Data Fabrication in Collaborative Vehicular Perception: Attacks and Countermeasures." 

## Requirements

- Something about how much space this will take up 
- Free space > []GB (at least 20GB for CARLA)
- Python 3.7.9 

## References

- [On Data Fabrication in Collaborative Vehicular Perception: Attacks and Countermeasures](https://paperswithcode.com/paper/on-data-fabrication-in-collaborative)
- [Stealthy Data Fabrication in Collaborative Vehicular Perception](https://dl.acm.org/doi/10.1145/3690134.3694822)
- [Building a V2X Simulation Framework for Future Autonomous Driving](https://ieeexplore.ieee.org/document/8892860)

## Get started

```bash
# Get the codebase.
git clone --recursive https://github.com/angelgeck98/CSS-CAV.git
cd CSS-CAV

# Set up the Python environment.
pyenv install 3.7.9
pyenv local 3.7.9

# Create and activate environment.
python3.7 -m venv venv
source venv/bin/activate
python --version # should be 3.7.9

# Install dependencies.
pip install --upgrade pip
pip install -r requirements.txt

# Install CARLA.
# Quickstart information: https://carla.readthedocs.io/en/latest/start_quickstart/
# CARLA Repository: https://github.com/carla-simulator/carla/tree/master?tab=readme-ov-file

# Download appropriate CARLA release from this link: 
# https://github.com/carla-simulator/carla/releases/tag/0.9.15/
# Extract the release file to the current working directory.

# Install CARLA client library.
pip install carla

# Run simulator and output evaluation logs.
python simulator.py
```
This will create the Python 3.7.9 virtual environment, install dependencies, and execute the CARLA simulation. 

## Functionality
In the final version of the application, a simulator spawns in Attacker and Defender vehicles in a CARLA environment. 
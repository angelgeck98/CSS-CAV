# CSS-CAV
Repo for further implementation of CAD outlined in USENIX security 2024 paper "On Data Fabrication in Collaborative Vehicular Perception: Attacks and Countermeasures." 

## Requirements

- Free space > 40GB 
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
This will create the Python 3.7.9 virtual environment, install dependencies, and execute the CARLA simulation. The simulation will output evaluation results. 

## Functionality
In the final version of the application, a simulator spawns in Attacker and Defender vehicles in a CARLA environment. The simulation runs through twice - once without using the firewall, and a second time while using the firewall - and outputs evaluation results. Ideally, the Cars will send each other LiDAR sensor data that is filtered through the firewall; if a Car is determined to be "good" then their data will be accepted, and a "bad" Car's data will be rejected. Evaluation functions properly to show that a firewall increases the amount of attack detections and true positives. Components that need improvement include properly incorporating scores into the Cars' communication and updating send_v2x_message method to manipulate LiDAR sensor data and behavior of the simulated Cars. Further implementations could include further testing with larger numbers of vehicles, on different maps, and with different types of attacks being implemented (such as early, intermediate, and advanced spoofing or removal). 

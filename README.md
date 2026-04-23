# Aerial Robotics Search & Rescue (SAR) Flight System

A comprehensive system for autonomous UAV search and rescue operations, featuring path planning, waypoint generation, state machine control, and a web dashboard. Supports multiple mission scenarios and operational flight versions.

## 📁 Project Structure

### Core Modules

| Directory | Description |
|-----------|-------------|
| **sarFlightDay4** | **PRODUCTION VERSION - Final flight system used for actual operations** |
| **sarFlightDay1V2** | SAR Flight System v2, includes state machine, GUI, and MAVProxy integration |
| **path_planner** | Path planning engine with lawnmower, spiral, and PLB search algorithms |
| **herish_code_v1** | Waypoint generation tools with KML parsing and GPS coordinate conversion |
| **stateMachine-with pp codes** | Complete state machine with integrated path planning (pre-flight version) |

### Key Features

#### State Machine System (`sarFlightDay4/states`)
The **sarFlightDay4** directory contains the production state machine with:
- **idle** - Idle state
- **pre_auto_check** - Pre-flight checks
- **guided_takeoff** - Guided takeoff
- **generate_pattern** - Generate search patterns
- **search** - Execute search mission
- **upload_mission** - Upload flight plan
- **upload_fence** - Upload geofence
- **change_mode** - Change flight mode
- **deliver** - Payload delivery
- **focus** - Focus search on target
- **replan** - Dynamic mission replanning
- **safe_rtl** - Safe return to launch

#### Path Planning Algorithms (`path_planner`)
- **lawnmower_planner** - Lawnmower scan pattern
- **perimeter_planner** - Perimeter sweep pattern
- **PLB_searching** - Probability of Last Known Position algorithm
- **simulate.py** - Spiral search simulation
- **geo_utils** - Geospatial utilities and coordinate transformations

#### Waypoint Generation (`herish_code_v1/herish_code`)
- **waypoint_generator.py** - Generate waypoints from polygon boundaries
- **kml_to_gps.py** - Convert KML files to GPS coordinates
- **mission_upload.py** - Upload missions to UAV
- **kml_parser.py** - Parse KML files

## 🚀 Getting Started

### Requirements

- **Python 3.8+**
- **MAVProxy** - UAV communication middleware
- **Flask** - Web dashboard framework
- **DroneKit** - UAV control library
- **PyYAML** - Configuration file parsing

### Installation

```bash
# Core dependencies
pip install flask dronekit pymavlink pyyaml

# MAVProxy (may require elevated permissions)
pip install MAVProxy --break-system-packages

# Optional: Path planning dependencies
pip install numpy scipy shapely
```

### Quick Start

#### 1. Launch Production SAR System (sarFlightDay4)

```bash
cd sarFlightDay4/
python3 main.py
```

Supported startup options:
```bash
python3 main.py                    # Full startup (MAVProxy + state machine + GUI)
python3 main.py --no-proxy         # Skip MAVProxy
python3 main.py --no-gui           # Skip web dashboard
python3 main.py --no-proxy --no-gui # State machine only
```

Access web dashboard at: `http://<raspberry-pi-ip>:5000`

#### 2. Generate Waypoints

```bash
# Generate waypoints from KML file
cd herish_code_v1/herish_code/
python3 waypoint_generator.py

# Example code
from waypoint_generator import generate_waypoints

polygon_coords = [
    (-2.6021, 51.4545),
    (-2.6018, 51.4545),
    (-2.6018, 51.4548),
    (-2.6021, 51.4548)
]

waypoints = generate_waypoints(polygon_coords)
print(waypoints)
```

#### 3. Path Planning

```bash
cd path_planner/

# Generate lawnmower pattern
python3 lawnmower_planner.py

# Generate spiral search
python3 simulate.py

# Generate PLB probability search
python3 PLB_searching.py
```

## 📋 Configuration

All modules are configured through `config.py`:

```python
# UAV connection parameters
CONNECTION_STRING = "/dev/ttyUSB0"  # or serial port address
BAUD_RATE = 921600

# MAVProxy configuration
MAVPROXY_MASTER = "udp:127.0.0.1:14550"
MAVPROXY_OUTPUTS = ["udp:127.0.0.1:14551"]

# GUI configuration
GUI_HOST = "0.0.0.0"
GUI_PORT = 5000
```

Modify parameters according to your hardware and network setup.

## 📊 Output Files

Path planner outputs are saved in `path_planner/outputs/`:

- `waypoints_lawnmower.txt` - Lawnmower pattern waypoints
- `waypoints_spiral.txt` - Spiral search waypoints
- `comparison.txt` - Performance comparison of algorithms

## 🛠️ Development Guide

### Adding New States

Create new state files in the `states/` directory, inheriting from `base.State`:

```python
from base import State

class MyState(State):
    def __init__(self, drone, shared_status):
        super().__init__(drone, shared_status)
    
    def on_enter(self):
        """Called when entering state"""
        pass
    
    def update(self):
        """State update logic"""
        return next_state_name
    
    def on_exit(self):
        """Called when exiting state"""
        pass
```

### Extending Path Planning Algorithms

Create new algorithm files in `path_planner/` and use `geo_utils` for coordinate transformations and distance calculations.

## 🔗 Dependencies

| Package | Purpose |
|---------|---------|
| `dronekit` | UAV communication and control |
| `flask` | Web dashboard framework |
| `pymavlink` | MAVLink protocol implementation |
| `shapely` | Geometric operations and path planning |
| `numpy` | Numerical computations |

## ❓ FAQ

**Q: MAVProxy fails to start?**
A: Ensure MAVProxy is installed with the `--break-system-packages` option.

**Q: Cannot access web dashboard?**
A: Check the Raspberry Pi IP address and firewall settings. Ensure port 5000 is not in use.

**Q: KML file parsing fails?**
A: Verify the KML file format is valid and contains proper coordinate information.

## 📄 License

This project is licensed under the MIT License. See [LICENSE](LICENSE) file for details.

## 👥 Contributing

Issues and pull requests are welcome!

## 📞 Support

For questions or suggestions, please open an issue on the project repository.

---

**Last Updated:** April 2026  
**Python Version:** 3.8+  
**Status:** Active Development  
**Production Version:** sarFlightDay4

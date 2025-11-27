# COLREGs Core 🚢

[![Python](https://img.shields.io/badge/python-3.10%2B-blue)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Code Style: Black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)

**A Python library for maritime collision risk assessment and DRL reward shaping based on COLREGs.**

`colregs-core` provides robust, modular components for evaluating maritime encounter situations (Head-on, Crossing, Overtaking) according to the **International Regulations for Preventing Collisions at Sea (COLREGs)**. It is specifically designed to facilitate **Deep Reinforcement Learning (DRL)** research for autonomous surface vehicles (ASVs/USVs).

---

## 🌟 Key Features

*   **COLREGs Encounter Classification:** accurately classifies encounter types (Head-on, Crossing Give-way/Stand-on, Overtaking) using relative bearing and course.
*   **Collision Risk Assessment (CRI):** Implements advanced collision risk models, including **CPA/TCPA** calculations and **Dynamic Ship Domains**.
*   **DRL Reward Shaping:** Provides pre-built, research-validated reward calculators (`JeonReward`, `ChunReward`) for training DRL navigation agents.
*   **Coordinate System Support:** Handles conversions between **NED (North-East-Down)** maritime coordinates and standard mathematical coordinates.
*   **Modular Design:** Easy to integrate into any simulation environment (e.g., `ir-sim`, `Gazebo`, `Unity`).

---

## 📦 Installation

### Using Poetry (Recommended)

```bash
git clone https://github.com/your-username/colregs-core.git
cd colregs-core
poetry install
```

### Using Pip

```bash
pip install .
```

---

## 🚀 Quick Start

### 1. Encounter Classification

```python
from colregs_core import EncounterClassifier

classifier = EncounterClassifier()

# Own Ship (OS) and Target Ship (TS) states
# Position: (North, East) in meters, Heading: Degrees (0=North, Clockwise)
situation = classifier.classify(
    os_position=(0, 0), os_heading=0, os_speed=5.0,
    ts_position=(500, 500), ts_heading=270, ts_speed=5.0
)

print(f"Encounter Type: {situation.encounter_type.name}")  # e.g., CROSSING_GIVE_WAY
print(f"Relative Bearing: {situation.relative_bearing:.1f}°")
print(f"Distance: {situation.distance:.1f} m")
```

### 2. Collision Risk Assessment

```python
from colregs_core import JeonCollisionRisk, ShipDomainParams

# Define Ship Domain (Asymmetric)
domain = ShipDomainParams(r_bow=30, r_stern=10, r_starboard=20, r_port=10)

risk_model = JeonCollisionRisk(ship_domain=domain)

risk = risk_model.calculate_collision_risk(
    os_speed=5.0, os_position=(0, 0), os_velocity=(5, 0), os_heading=0,
    ts_speed=5.0, ts_position=(500, 500), ts_velocity=(-5, 0), ts_heading=270
)

print(f"Collision Risk (0-1): {risk['cr']:.4f}")
print(f"DCPA: {risk['dcpa']:.1f} m, TCPA: {risk['tcpa']:.1f} s")
```

### 3. DRL Reward Calculation

```python
from colregs_core.reward import JeonRewardCalculator

# Initialize calculator
reward_calc = JeonRewardCalculator(w_efficiency=1.0, w_safety=1.0)

# Calculate reward at each step
rewards = reward_calc.calculate_total_reward(
    current_distance=100.0,
    previous_distance=102.0,  # Approaching goal
    cross_track_error=0.5,
    os_speed=5.0,
    # ... (other state parameters) ...
    CR_max=0.2
)

print(f"Total Reward: {rewards['r_total']:.4f}")
```

---

## 📚 Modules

| Module | Description |
| :--- | :--- |
| `colregs_core.encounter` | Classifies COLREGs situations (Rule 13, 14, 15). |
| `colregs_core.risk` | Calculates Collision Risk Index (CRI), CPA, TCPA, and Ship Domains. |
| `colregs_core.reward` | Provides reward functions for Reinforcement Learning (Jeon et al., Chun et al.). |
| `colregs_core.geometry` | Handles coordinate transforms (NED ↔ Math) and vector calculations. |

---

## 📖 Documentation

For detailed usage guides and API references, please check the [docs/](docs/) directory:

*   [**Coordinate System Guide**](docs/COORDINATE_SYSTEM_GUIDE.md): Important notes on degree/radian and NED/Math conventions.
*   [**COLREGs Rules**](docs/colregs_rules.md): Explanation of implemented maritime rules.
*   [**Usage Guide**](docs/usage_guide.md): Detailed examples.

---

## 🤝 Contributing

Contributions are welcome! Please follow these steps:

1.  Fork the repository.
2.  Create a new feature branch (`git checkout -b feature/YourFeature`).
3.  Commit your changes (`git commit -m 'Add some feature'`).
4.  Push to the branch (`git push origin feature/YourFeature`).
5.  Open a Pull Request.

---

## 📄 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

---

## 🔗 Citation

If you use this code for your research, please cite the following papers:

*   **Chun et al. (2021).** "Deep reinforcement learning-based collision avoidance for an autonomous ship."
*   **전도현. (2024).** "A Method for Collision Avoidance of a Ship Based on Reinforcement Learning in Complex Maritime Situations."
*   **Chun et al. (2024).** "Method for collision avoidance based on deep reinforcement learning with path-speed control for an autonomous ship."
*   **Woo and Kim (2020).** "Collision avoidance for an unmanned surface vehicle using deep reinforcement learning."

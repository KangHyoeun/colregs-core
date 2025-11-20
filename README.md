# COLREGs Core

A Python library for calculating maritime collision risk and shaping rewards for DRL-based navigation, based on the International Regulations for Preventing Collisions at Sea (COLREGs).

This library provides modular components for assessing encounter situations and calculating rewards, designed for easy integration into DRL training environments like `DRL-otter-navigation`.

## Key Features

-   **Multiple Reward Strategies:** Implements several reward calculation schemes from maritime robotics research, including:
    -   `ChunRewardCalculator`: A multi-component reward function balancing efficiency and safety.
    -   `JeonRewardCalculator`: A reward function based on a dynamic ship domain and collision risk.
-   **Collision Risk Modeling:** Includes the `JeonCollisionRisk` model, which uses concepts like TCPA, DCPA, and a dynamic ship domain to quantify collision risk.
-   **Easy Integration:** Designed to be easily instantiated and called within a DRL agent or environment's step function.

---

## Installation

This project is managed with `poetry`.

```bash
# Navigate to the project directory
cd /home/hyo/colregs-core

# Activate your conda environment
conda activate DRL-otter-nav

# Install dependencies
poetry install
```

---

## Core Components & Usage

The primary components you will use for DRL are the **Reward Calculators**. They encapsulate the complexity of risk assessment and COLREGs compliance into a single, easy-to-use-interface.

### 1. `JeonRewardCalculator`

This calculator uses the `JeonCollisionRisk` model to shape rewards. It heavily penalizes actions that lead to a high collision risk.

**Usage:**

```python
from colregs_core.reward import JeonRewardCalculator

# Initialize the calculator
# Speed parameters are for calibrating the risk model
jeon_calculator = JeonRewardCalculator(
    os_speed_for_cr=2.0,
    ts_speed_for_cr=2.0,
    w_dist=0.3,
    w_coll=0.7
)

# In your environment's step function, calculate the reward
reward_dict = jeon_calculator.calculate_total_reward(
    os_position=(0, 0),
    os_velocity=(2.0, 0.0),
    ts_position=(200, 0),
    ts_velocity=(-2.0, 0.0),
    dist_to_goal=50.0,
    is_static_obstacle=False
)

total_reward = reward_dict['r_total']
```

### 2. `ChunRewardCalculator`

This calculator provides a more granular reward, composed of distinct "efficiency" and "safety" components.

**Usage:**

```python
from colregs_core.reward import ChunRewardCalculator

# Initialize the calculator
chun_calculator = ChunRewardCalculator(
    d_max=20.0,
    v_ref=2.0,
    w_goal=0.4,
    w_cross=0.2,
    w_speed=0.2,
    w_risk=0.2
)

# In your environment's step function, calculate the reward
reward_dict = chun_calculator.calculate_total_reward(
    goal_position=(1000, 0),
    os_position=(0, 0),
    os_velocity=(2.0, 0.0),
    previous_distance=1000.0,
    encounter_type=0, # Replace with actual encounter type
    CR_max=0.1,
    is_static_obstacle=False,
    ts_position=(200, 0),
    ts_velocity=(-2.0, 0.0),
    os_heading=0.0
)

total_reward = reward_dict['r_total']
```
---

## Project Structure

```
colregs-core/
└── src/
    └── colregs_core/
        ├── reward/
        │   ├── jeon_reward.py    # Implements JeonRewardCalculator
        │   └── chun_reward.py    # Implements ChunRewardCalculator
        └── risk/
            └── ship_domain.py    # Implements JeonCollisionRisk
```
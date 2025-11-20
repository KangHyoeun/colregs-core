# Coordinate System and Angle Unit Guide

**⚠️ IMPORTANT: This codebase mixes degrees and radians. Always check function docstrings before use!**

This document provides a quick reference for which functions use **degrees** vs **radians** to help you avoid bugs.

---

## Quick Reference Table

| Module | Function | Input Units | Output Units | Notes |
|--------|----------|-------------|--------------|-------|
| **encounter/classifier.py** | | | | |
| | `classify()` | `os_heading`: **degrees**<br>`ts_heading`: **degrees** | All angles: **degrees** | ✅ Now consistent! All functions use degrees. |
| **geometry/bearings.py** | | | | |
| | `calculate_relative_bearing()` | `os_heading`: **degrees** | **degrees** [0, 360) | ✅ Now uses degrees! |
| | `calculate_aspect_angle()` | `ts_heading`: **degrees** | **degrees** [0, 360) | ✅ Now uses degrees! |
| | `heading_speed_to_velocity()` | `heading`: **degrees** | velocity tuple | ✅ Now uses degrees! |
| | `velocity_to_heading_speed()` | velocity tuple | `heading`: **degrees** [0, 360) | ✅ Now returns degrees! |
| | `calculate_bearing_rate()` | - | **radians/s** | Still returns radians/s |
| **geometry/coordinate_transform.py** | | | | |
| | `ned_to_math_heading()` | **degrees** | **degrees** | |
| | `math_to_ned_heading()` | **degrees** | **degrees** | |
| | `maritime_to_math_velocity()` | `maritime_deg`: **degrees** | velocity tuple | |
| | `math_to_maritime_velocity()` | `math_deg`: **degrees** | velocity tuple | |
| **reward/jeon_reward.py** | | | | |
| | `calculate_heading_reward()` | `os_heading`: **degrees**<br>`previous_heading`: **degrees** | reward value | ✅ Now uses degrees! (phi_max must also be in degrees) |
| | `calculate_risk_reward()` | `os_heading`: **degrees** | reward value | ✅ Now uses degrees! |
| | `calculate_colregs_reward()` | `os_heading`: **degrees** | reward value | ✅ Now uses degrees! |
| **reward/colregs_compliant.py** | | | | |
| | `is_compliant_headon()` | `os_heading`: **degrees** | bool | ✅ Uses degrees! |
| | `is_compliant_standon()` | `os_heading`: **degrees** | bool | ✅ Uses degrees! |
| | `is_compliant_static()` | `os_heading`: **degrees** | bool | ✅ Uses degrees! |
| **utils/utils.py** | | | | |
| | `WrapToPi()` | **radians** | **radians** | |
| | `WrapTo180()` | **degrees** | **degrees** | |
| | `WrapTo360()` | **degrees** | **degrees** | |
| | `ref_course_angle()` | positions | **degrees** | ✅ Now returns degrees! |

---

## Common Patterns & Gotchas

### ✅ Pattern 1: EncounterClassifier Now Consistent!
The main `EncounterClassifier.classify()` method expects **degrees** for headings, and all internal functions now also use degrees:
```python
classifier = EncounterClassifier()
situation = classifier.classify(
    os_position=(0, 0),
    os_heading=90.0,      # ✅ DEGREES
    os_speed=5.0,
    ts_position=(100, 100),
    ts_heading=180.0,     # ✅ DEGREES
    ts_speed=3.0
)
# All returned angles are in degrees
```

**✅ FIXED**: The `bearings.py` functions now use degrees, so the classifier is consistent!

### ✅ Pattern 2: Geometry Functions Now Use Degrees
All functions in `bearings.py` now consistently use **degrees**:
```python
# All inputs and outputs in degrees
relative_bearing_deg = calculate_relative_bearing(
    os_position, os_heading_deg, ts_position
)  # Returns degrees [0, 360)

aspect_angle_deg = calculate_aspect_angle(
    ts_heading_deg, os_position, ts_position
)  # Returns degrees [0, 360)

vx, vy = heading_speed_to_velocity(heading_deg, speed)
heading_deg, speed = velocity_to_heading_speed((vx, vy))
```

### ✅ Pattern 3: Reward Functions Now Use Degrees
All reward calculation functions in `jeon_reward.py` now expect headings in **degrees**:
```python
reward_calc = JeonRewardCalculator(
    phi_max=45.0  # ⚠️ Must be in degrees to match os_heading units!
)
rewards = reward_calc.calculate_total_reward(
    # ... other params ...
    os_heading=90.0,  # ✅ Degrees
    previous_heading=85.0,  # ✅ Degrees
    # ...
)
```

**Important**: Make sure `phi_max` is in the same units as your headings (degrees)!

### ⚠️ Pattern 4: Coordinate Transform Uses Degrees
All functions in `coordinate_transform.py` use **degrees**:
```python
# All in degrees
math_heading_deg = ned_to_math_heading(ned_heading_deg)
maritime_heading_deg = math_to_ned_heading(math_heading_deg)
```

---

## Conversion Helpers

When you need to convert between radians and degrees:

```python
import numpy as np

# Degrees to radians
rad = np.radians(deg)

# Radians to degrees
deg = np.degrees(rad)
```

---

## Typical Workflow Examples

### Example 1: Using EncounterClassifier
```python
# Your simulator gives headings in radians
os_heading_rad = 1.57  # ~90 degrees
ts_heading_rad = 3.14  # ~180 degrees

# Convert to degrees for EncounterClassifier (now all functions use degrees!)
classifier = EncounterClassifier()
situation = classifier.classify(
    os_position=(0, 0),
    os_heading=np.degrees(os_heading_rad),  # Convert to degrees
    os_speed=5.0,
    ts_position=(100, 100),
    ts_heading=np.degrees(ts_heading_rad),  # Convert to degrees
    ts_speed=3.0
)
# All returned angles are in degrees
# situation.relative_bearing is in degrees
# situation.aspect_angle is in degrees
```

### Example 2: Using Reward Calculator
```python
# Your simulator gives headings in radians
os_heading_rad = 1.57
previous_heading_rad = 1.48

# Convert to degrees for reward calculator
reward_calc = JeonRewardCalculator(
    phi_max=45.0  # Must be in degrees!
)
rewards = reward_calc.calculate_total_reward(
    # ...
    os_heading=np.degrees(os_heading_rad),  # Convert to degrees
    previous_heading=np.degrees(previous_heading_rad),  # Convert to degrees
    # ...
)
```

### Example 3: Using Geometry Functions Directly
```python
# If you have headings in radians, convert to degrees first
os_heading_rad = 1.57
ts_heading_rad = 3.14

# All geometry functions now expect degrees
relative_bearing_deg = calculate_relative_bearing(
    os_position, np.degrees(os_heading_rad), ts_position
)  # Returns degrees

# calculate_aspect_angle also expects degrees
aspect_angle_deg = calculate_aspect_angle(
    np.degrees(ts_heading_rad), os_position, ts_position
)  # Returns degrees
```

---

## Summary Rules

1. **EncounterClassifier**: Input headings in **degrees**, outputs in **degrees** ✅
2. **Reward functions** (`jeon_reward.py`): Input headings in **degrees**, outputs are reward values ✅ (FIXED!)
3. **COLREGs compliance** (`colregs_compliant.py`): Input headings in **degrees** ✅
4. **Geometry functions** (`bearings.py`): 
   - Input headings: **degrees** ✅ (FIXED!)
   - Output angles: **degrees** ✅
   - Exception: `calculate_bearing_rate()` returns **radians/s**
5. **Coordinate transform functions**: All use **degrees** ✅
6. **Utility functions**: 
   - `WrapToPi()`: **radians**
   - `WrapTo180()`, `WrapTo360()`: **degrees**
   - `ref_course_angle()`: Returns **degrees** ✅ (FIXED!)

---

## Debugging Tips

If you get unexpected angle values:

1. **Check the function docstring** - it should specify radians or degrees
2. **Check the return type** - if it's wrapped with `WrapTo360()` or `WrapTo180()`, it's likely degrees
3. **Check if `np.degrees()` or `np.radians()` is called** - this indicates a conversion
4. **Common bug**: Passing degrees where radians are expected (or vice versa) will give wrong results but won't crash

---

## Future Improvement (Not Implemented)

Ideally, the codebase should be refactored to use a consistent unit (preferably radians internally, degrees for API). However, this would require:
- Changing all function signatures
- Updating all call sites
- Extensive testing

For now, this guide should help you use the code correctly as-is.


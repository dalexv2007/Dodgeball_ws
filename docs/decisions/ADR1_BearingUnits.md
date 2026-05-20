# Document Heading

## ADR-001: Bearing Units from Sensors

**Status:** Accepted

**Context:**
Previously, the project inconsistently used radians, degrees, and pixels to calculate bearing data. Measurements would return pixel values which made PID controller gains position-dependent, unusable data.  

**Decision:**
I've decided to use strictly radians. Calculating directly from BallFinder.py, updating BallLocation.msg.bearing to double(float64) to hold radian values and using the found flag instead of setting bearing = -1, which was a valid value in radians.

**Reasoning:**
Using the found bool clean, simple code. Bearing value has one job: hold offset value. Found has one job: deciding if the ball is within acceptable bearing error ("targeted"). Radians provide clean, dimensionless values for the PID controller and tuning.  

**Alternatives considered:**
Considered using float type -> NaN value for invalid case, but that adds complexity and responsibility to bearing field.
Also considered using out-of-physical-bounds bearing value, but again that adds complexity and responsibility to bearing.

**Consequences:**
This decisions constrains "jobs" of fields and ensures correct units in downstream dependencies, receiving interpretable dimensions for PID and camera resolution changes. Fields constrained to their roles with accurate, descriptive names.

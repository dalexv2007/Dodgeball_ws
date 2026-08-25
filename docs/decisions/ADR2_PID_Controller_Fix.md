# Document Heading

## ADR-002: PID Controller Fix:

**Status:** Troubleshooting/ Diagnosing

**Context**
The original PID controller file did not work as intended, and the reason was unknown. This solution is about finding the problem, fixing pid_controller.hpp, and the source of the issues from other parts of the project. As of 8-26-2026, it appears that BallFinder.py is not assigning a bearing value, pid_controller.hpp may not be receiving signed errors, and some formulas may be incorrect.

Troubleshooting notes included under pid_controller.hpp comments, will be resolved in order of dependency.

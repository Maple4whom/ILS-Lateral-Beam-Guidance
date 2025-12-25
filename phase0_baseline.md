# ILS Lateral Beam Guidance System 
# Author: Joshua Yandoc

## Project Overview
Simulation-based implementation of an Instrument Landing System (ILS) lateral beam
guidance and control system, developed from a university assignment and rebuilt
as a portfolio-quality GNC project.

The project follows a phased approach:
- Phase 0: Baseline implementation and validation (frozen)
- Phase 1+: Deliberate realism and performance improvements

## Phase 0 — Baseline Validation (Frozen)
Phase 0 establishes a faithful, traceable implementation of the assignment system.
The MATLAB script-based simulation and Simulink block-diagram model implement
identical equations and produce matching responses.

No structural design changes are made in Phase 0.

If we are strictly following the assignment guide, phase 0 direcltly deals with questions 1-8

## Phase 0 summary - What Phase 0 proves 
Phase 0 was all about creating a baseline simulation model that is script-based, which was subsequently validated by a SIMULINK model. 

## Initial Baseline model (untuned --> tuned)
- See 'baselineSim_state_var_plots'.png and 'retunedSim_state_var_plots'.png 
This initial simulation comes from the model before any tuning was done on the coupler gain, G_c, and before an integral term was introduced. 

The system in this state is notable underdamped, and takes an unreasonably unacceptable amount of time to converge. This system was created this way for the purposes of forming a foundational system upon which we can experiment with tuning and introducing other terms, like the integral term (at least for the purposes of the assignment). For this repurposed version, the code utilizing the retuned system will be the default, though the entire code is still left in the file

## Gains used (tuned)
K_I = 0.9
G_c = 25.5


## Key Modeling Assumptions
- Pertaining to the roll-rate loop output in both the MATLAB model (as seen in the 'State_Deriv' file), as well as the SIMULINK model (main > Aircraft & Lateral Autopilot subsystem)
    - The roll-loop output `e` is treated as a generalized servo reference signal. (i.e. Unit scaling is absorbed into the servo gain `K_P`.)
        Author Comment: When creating both the script, and block-based model, following the math strictly would show that the final loop has a unit discrepancy between the error signal 'e' and the outputted deflection angle `δ_a` within the aileron servo subsystem. Because the provided assignment documentaion doesn't expand on this, an assumption had to be made that the gain scales and deals with these unit discrepancies.
    - The aileron servo outputs physical aileron deflection angle `δ_a`, which directly drives the aircraft roll dynamics.

## Status
- Phase 0: complete and frozen (`phase0-baseline`)
- Phase 1: actuator realism (next step)
- Phase 2: Robustness and further analysis (future work)

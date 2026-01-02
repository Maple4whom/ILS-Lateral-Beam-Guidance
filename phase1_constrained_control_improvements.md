# Author: Joshua Yandoc
# Phase 1 Log

## Phase Overview 
Now that the baseline is establish, this phase looks to implement/impose realistic design improvements. That being:
-Actuator Limits
-Structured tuning
-Robustness/sensitivity
-further comparisons/ monte carlo simulaiton (these are TBD operations, pending time constraints)

Much like the phase 0 log, this document is meant to log the system's updates as the project progresses/before any commits are done on the github repo, so as to keep trach of change history, new gain values, added simulink blocks, etc.

## Actuator Limits
1. Began implementation - Dec. 27, 2025
2. Added Saturation block to Simuliunk model; Revised model to include pre and post saturation logging (see 'Aileron Servo with Saturation'.png --> Plots --> Phase 1 -->Actuator Limits) - Dec 27, 2025
3. Added Rate limiter to Servo subsystem (see 'Aileron Servo with Rate Limiter'.png)
4. Learned, Implemented, and Debugged Variant Subsystem in aileron servo to include both ideal actuation, and actuation with rate limits - Dec 27, 2025
    Regarding the feedback behavior issue - I tried various solutions: variant subsystem post DC motor, changing the deflection subsystem into saturated DC motor, and finally, imposing the saturation and rate limit within the coupled DC motor system itself

    # Values
    - Saturation limit of +/- 20 deg - Dec. 27, 2025
    - Slew rate limit of +/- 10 deg/s - Dec. 27, 2025

## Testing/Validation with different initial conditions
1. Scenario 0 - Original Refernce Case
    
    # Values
    V_T = 60 m/s
    Y_R0 = 150 m
    psi_0 = -20 deg

2. Scenario 1 - Slow Approach Capture

    # Values
    V_T = 50 m/s
    Y_R0 = 250 m
    psi_0 = -20 deg

3. Scenario 2 - Fast Approach, small offset

    # Values
    V_T = 65 m/s
    Y_R0 = 100 m
    psi_0 = -10 deg

4. Scenario 3 - Large Misalignment Stress Test

    # Values
    V_T = 60 m/s
    Y_R0 = 150 m
    psi_0 = -20 deg


## Where to pick up next time/activity log
1. Pick-up: look into suspicious plots - investigate where proper feedback should go - Dec 28, 2025
2.  Update: Investigated the behavior - unresolved --> will continue - Dec 28/29, 2025
3.  Update: Resolved feedback behavior issue - Dec 29, 2025
4. Update: Cleaned up code a little and made a new plot (see commit titled "phase1-actuator-limits
    Update: Updated code and plots" for more detailed explanation) - Dec 30, 2025
5. Update: More code cleaning/optimization before moving on - Created new functions 'plot_script_sim' 'plot_simulink_sim_states', and 'plot_delA' to handle the plotting and make main script look much more clean/easier to read - Jan 1, 2026

## Future work/optimization
1. look into making another function 'compare_act_limits' that only runs the for-loop for plotting/comparing different actuator limits when a conditional is met (say, USE_ACT_LIMITS == true/false ?) - Jan 1, 2026

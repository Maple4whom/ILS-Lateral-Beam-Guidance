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

# Where to pick up next time
1. look into suspicious plots - investigate where proper feedback should go - Dec 28, 2025
    Update: Investigated the behavior - unresolved --> will continue - Dec 28/29, 2025
    Update: Resolved feedback behavior issue - Dec 29, 2025
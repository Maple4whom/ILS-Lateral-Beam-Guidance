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
2. Added Saturation block to Simuliunk model; Revised model to include pre and post saturation logging (see 'Aileron Servo with Saturation'.png) - Dec 27, 2025

    # Values
    - Saturation limit of +/- 20 deg
<details>
<summary>Follow Line</summary>

This project addresses a practical challenge proposed by the [**Unibotics**](https://unibotics.org/) educational robotics framework.

## Follow Line Project Objectives

The primary objective of this exercise is to implement a robust control system for the F1 car, enabling it to autonomously navigate the track by adhering to the red line.

This involves developing a vision-based algorithm capable of completing a full lap. A secondary performance goal is to tune the controller parameters to minimize the lap completion time, ensuring both stability and speed.

## Screenshoots

Here are some pictures about the track, the car and the line it must follow.

<p align="center">
  <img width="400" height="250" alt="0_f19SJryGW0aMLGmy" src="https://github.com/user-attachments/assets/be7a4679-e205-4c1e-92a2-cf67cf1b0887" />

<img width="400" height="250" alt="0_qxK5hBHO7OiWuM3J" src="https://github.com/user-attachments/assets/c183aeef-0910-40ab-ae8a-a95d43719975" />
</p>


## [Proportional (P) Controller](./src/Proporcional.py)

The Proportional controller is the simplest form of feedback control. Its logic is based on the present: the control output is directly proportional to the current error. In this context, the **error (`err`)** is the distance from the line's center to the car's center.

**Parameters:**
* **`KP` (Proportional Gain):** This is the main "strength" multiplier for the controller.
    * **Effect:** A higher `KP` makes the car react more aggressively (sharper turns) to the error. If it is set too high, the car will overshoot the line and weave back and forth very fast. If set too low, the car will be sluggish and slow to correct, potentially cutting corners.

## [Proportional-Derivative (PD) Controller](./src/Derivativo.py)

The Proportional-Derivative controller adds a "Derivative" component, which is based on the future. It considers the current error (P) and the **rate of change of the error (D)**. This "D" component effectively predicts where the error is heading and acts to counter rapid changes.

**Parameters:**
* **`KP` (Proportional Gain):** Provides the primary corrective force based on the current error, just like in the P controller.
* **`KD` (Derivative Gain):** This acts as a "damper" or "brake" on the steering.
    * **Effect:** A well-tuned `KD` dampens the oscillations caused by a high `KP`. It smooths the steering by "resisting" rapid changes, which prevents the car from overshooting the line and allows for a more stable and less "nervous" ride.

## [Proportional-Integral-Derivative (PID) Controller](./src/Integral.py)

The PID controller is the most complete, adding an "Integral" component based on the past. It sums the Proportional (present error), Integral (past accumulated error), and Derivative (future predicted error) components.

**Parameters:**
* **`KP` (Proportional Gain):** Provides the main steering force based on the *current* error.
* **`KD` (Derivative Gain):** Dampens oscillations by predicting the *future* error (acts like a brake on the steering).
* **`KI` (Integral Gain):** This component accumulates the error over time.
    * **Effect:** The `KI` term is designed to eliminate "steady-state error." This is a small, persistent error that a PD controller might never fix (e.g., the car always driving 1cm to the left of the line on a long curve). By accumulating this small error, the `KI` term slowly builds up a corrective force to push the car perfectly onto the center of the line.


</details>

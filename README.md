# XLR8PID

## Accelerated PID Library for Arduino

This repo contains the software library and Verilog rtl source code for the Alorium Technology PID Control Xcelerator Block (XB).  

The PID XB is intended for use with [Alorium Technology's](https://www.aloriumtech.com/) XLR8 family of boards - [XLR8](https://www.aloriumtech.com/xlr8), [Snō](https://www.aloriumtech.com/sno), and [Hinj](https://www.aloriumtech.com/hinj) - as part of the [OpenXLR8 flow](https://www.aloriumtech.com/openxlr8) for creating custom FPGA images.

## More on PID
The Proportional, Integral, Derivative, control algorithm (PID) is a feedback control loop designed to achieve stable control of an entity such as temperature, speed, or position, by taking into account the past, the present, and the future.  

In very basic terms:

- The 'P' component deals with the present, and makes adjustments based on how close a system is to its goal right now.
- The 'I' component focuses on the past, and makes adjustments based on how long it's been since the system has been at or near its goal.
- The 'D' term predicts the future.  Well, not really, but it does anticipate what may happen in the future and makes adjustments to try and minimize how far from its goals a system will drift.

## PID XB
The Alorium Technology PID XB implements this control algorithm, using simple 16 bit fixed-point math to help optimize performance, and an extremely accurate hardware timer to accurately implement the integral and derivative components of the the algorithm. 

Software scaling can be used to achieve ranges of inputs and outputs that makes sense for any given system, without the burden (both in performance and in size of the logic) of implementing floating point math in the hardware.

## Specifications
- 16 bit fixed point math
- 20ms time step for integral and derivative calculations
- Reset windup prevention to eliminate error accumulation
- Programmable coefficients for P, I, and D calculations (Kp, Ki, Kd)
- 2 inputs, 1 output
- Configurable number of PIDs (up to 6)
- Calculation update whenever an update to PV occurs (adjusted to 20ms boundaries)

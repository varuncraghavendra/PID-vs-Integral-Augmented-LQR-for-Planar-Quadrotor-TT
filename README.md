# PID-vs-Integral-Augmented-LQR-for-Planar-Quadrotor-Trajectory-Tracking

Quadrotors have become a standard platform for research and applications in inspection, map-
ping, logistics, and autonomous navigation. A key requirement in many missions is accurate tra-
jectory tracking in the horizontal plane, often under limited sensing and actuation constraints.
Classical proportional–integral–derivative (PID) controllers are widely adopted in practice be-
cause they are simple to implement and tune. However, model-based optimal control techniques
such as the Linear Quadratic Regulator (LQR) can exploit the system structure and provide a
systematic compromise between tracking performance and control effort.
In this report, we compare a PID controller against an integral-augmented LQR controller
for trajectory tracking using a simplified planar quadrotor model. Rather than a full six-
degree-of-freedom model, we consider a normalized point-mass moving in the xy-plane with
double-integrator dynamics, which is a common outer-loop abstraction for quadrotor position
control. The inner loops (attitude and thrust) are assumed to track the commanded accelera-
tions sufficiently quickly.
Two main controller variants are implemented in MATLAB:
• A baseline implementation , using hand-tuned PID gains and a moderately weighted
LQR.
• A research-informed implementation, where PID and LQR parameters are chosen with
reference to published quadrotor control studies by Sahrir & Basri [1], Dhewa et al. [2],
Darwito et al. [3], and Sharma et al. [4].
The benchmark trajectories are a slow circular path, a fast circular path, and a square path,
all of radius/half-side R = 3 m (side length 2R = 6 m for the square). For each case, we run
both the baseline and the research-informed controllers and evaluate performance using mean
absolute error (MAE) and root-mean-square error (RMSE) in both x and y, averaged over three
laps. The numerical results printed by the MATLAB scripts are inserted directly into the results
section, and we explicitly explain how the gains and weighting matrices used in the code are
anchored in the cited literature

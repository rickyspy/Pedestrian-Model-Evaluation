# Pedestrian-Model-Evaluation

Considering the fact that there is almost no general quatitative evaluation, calibration, and validation benchmark for pedestrian model, we introduce a pedestrian model evaluation framework based on trajectories from repeated experiments.


With this code, we formulate a framework for quantitatively evaluating a pedestrian model by comparing the trajectories in simulations and in experiments. 
The framework is described in this the paper [A generalized trajectories-based evaluation approach for pedestrian evacuation models, Safety Science,
Volume 147,
2022.](https://www.sciencedirect.com/science/article/pii/S0925753521004161?dgcid=coauthor#ak005)

We introduce four types of indexes here:
- macroscopic type (fundamental diagram index, speed choice index, and direction choice index)
- microscopic type (trajectories pattern index)
- distribution type (route length distribution index, travel time distribution index)
- time-series type (starting position distance time-series index, destination position distance time-series index). 

An improved dynamic time warping approach as well as the Kolmogorov-Smirnov approach are applied, and a detailed calculation process can be found in the code. 

This work is feasible for different experiments and different simulations, hence the researchers can find a quantitative benchmark to evaluate, calibrate and validate pedestrian models.

All in all, YOU input the empirical and simulation trajectories, WE can output a radar figure and a score for the similarities.

![](radar.jpg)


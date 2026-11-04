# Nonlinear Emotion Dynamics and Bounded Robot Behaviour Control



This repository contains the MATLAB implementation and numerical-study scripts for the manuscript:

> J. U. Yahaya and A. Nasir, \*Nonlinear Emotion Dynamics and Bounded Robot Behavior Control for Assistive Human-Robot Interaction\*.

The code implements a six-emotion nonlinear model for assistive human-robot interaction and a bounded linear quadratic regulator (LQR) that adjusts four robot-behaviour dimensions: support, predictability, task adaptation, and repair-oriented action. The repository accompanies a simulation-focused study; it is **not** a clinically validated model of individual human emotions and it is not intended for direct clinical decision-making.

### 

### What is included

The repository should contain the following categories of files.

|Category|Main files|Purpose|
|-|-|-|
|Model and parameters|`emotion\_model\_parameters.m`, `emotion\_dynamics.m`, `emotion\_dynamics\_with\_input.m`, `scenario\_inputs.m`, `reminder\_pulse.m`|Defines the nonlinear emotion model, parameter values, and interaction events.|
|Controller|`design\_lqr\_controller.m`, `robot\_control.m`, `robot\_control\_history.m`|Designs the nominal LQR controller and applies the physical command bound.|
|Simulation and metrics|`simulate\_emotion\_case.m`, `compute\_performance\_indices.m`|Runs the canonical open-loop and saturated-LQR simulations and calculates performance measures.|
|Main analyses|`run\_scenario1\_simulation.m`, `run\_validation\_studies.m`, `run\_scenario\_sensitivity\_study.m`, `run\_monte\_carlo\_robustness.m`, `run\_damping\_stiffness\_sensitivity.m`|Reproduces the nominal and robustness analyses reported in the manuscript.|
|Additional checks|`run\_measurement\_imperfection\_study.m`, `run\_saturation\_ablation\_fixed.m`|Reproduces the corrected imperfect-feedback and command-saturation studies.|

Some repositories may also contain plotting utilities, exported figures, and a `validation\_results` folder. These files are useful for inspection but are not required to execute the core model.



### Software requirements

* MATLAB with the **Control System Toolbox** (required for `lqr`);
* standard MATLAB numerical solvers, including `ode45`;
* no external datasets or hardware are required.

The code should be run from the repository root directory. Before running a study, add all repository folders to the MATLAB path:

```matlab
clear; clc; close all;
addpath(genpath(pwd));
```

### Recommended reproduction order

The following order makes it easiest to verify the reported calculations.

1. Run the nominal Scenario 2 simulation and verify that the model and controller execute without errors.

```matlab
p = emotion\_model\_parameters(2);
result = simulate\_emotion\_case(p);
```

2. Run the weighting and validation studies, if included in the repository.

```matlab
run\_validation\_studies
```

3. Run the corrected imperfect-feedback study.

```matlab
run\_measurement\_imperfection\_study
```

4. Run the corrected command-saturation ablation study.

```matlab
run\_saturation\_ablation\_fixed
```

5. Run the remaining robustness studies as needed.

```matlab
run\_scenario\_sensitivity\_study
run\_monte\_carlo\_robustness
run\_damping\_stiffness\_sensitivity
```

Depending on the release package, one or more optional plotting scripts may have slightly different names. The key scripts for reproducing the final numerical-consistency corrections are `run\_measurement\_imperfection\_study.m` and `run\_saturation\_ablation\_fixed.m`.

## 

### Nominal Scenario 2 consistency check

The repeated-reminder scenario (Scenario 2) is the reference nominal case used throughout the manuscript. The following values provide a quick implementation check for the balanced LQR design:

|Quantity|Expected value|
|-|-:|
|Open-loop peak adverse-emotion response|0.063683|
|Saturated-LQR peak adverse-emotion response|0.058916|
|Open-loop aggregate adverse-emotion IAE|1.2479|
|Saturated-LQR aggregate adverse-emotion IAE|0.89457|
|Peak reduction|7.49%|
|IAE reduction|28.32%|
|LQR control effort|0.20025|
|Command saturation at nominal severity|0%|

The nominal-feedback row from `run\_measurement\_imperfection\_study` and the unit-scale row from `run\_saturation\_ablation\_fixed` should reproduce the same peak reduction, IAE reduction, and control effort, subject only to displayed numerical precision.

## 

### Notes on the corrected supplementary studies

The imperfect-feedback and saturation-ablation scripts use the same nonlinear model equations and Scenario 2 event definitions as the canonical nominal simulation. In particular, the shared function `emotion\_dynamics\_with\_input.m` must remain in the same MATLAB folder as `emotion\_dynamics.m`.

For elevated event-severity stress tests, the saturation-ablation study applies the same fixed nominal LQR gain while scaling the event amplitudes. The unbounded LQR signal is included solely as a mathematical reference. The implemented controller is the saturated controller, which enforces the physical command interval `\[0, 1]` element by element.

## 

### Reproducibility notes

* Run the files in a clean MATLAB session to avoid retaining variables from an earlier study.
* Do not rename `emotion\_dynamics\_with\_input.m` or remove it from the MATLAB path.
* The Monte Carlo study uses a fixed random-number seed in its script so that the reported statistics can be reproduced.
* Numerical values may differ very slightly across MATLAB releases because of solver and floating-point implementation details; conclusions should remain unchanged.

## 

### Citation

If you use this code, please cite the associated manuscript:

```text
Yahaya, J.U.; Nasir, A. Nonlinear Emotion Dynamics and Bounded Robot Behavior Control for Assistive Human-Robot Interaction. Manuscript submitted to Modelling.
```



For questions about the implementation, please contact the corresponding author listed in the associated manuscript.


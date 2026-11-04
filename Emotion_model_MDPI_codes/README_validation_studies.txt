Validation studies added to this code folder
=============================================

Run this file in MATLAB from the Emotion_model_codes folder:

    run_validation_studies

The script uses Scenario 2 (the repeated unclear medication reminder) and
creates a new folder called validation_results. It does not change the
original Scenario 1 or Scenario 2 scripts and figures.

Study A: LQR weighting comparison
---------------------------------
Three LQR designs are compared:

1. Balanced: the nominal Q and W matrices used in the original study.
2. Protective: larger state weights on sadness, anger, fear, and surprise.
3. Low effort: the nominal Q matrix and a larger control-effort penalty W.

The script exports lqr_weighting_study.csv and lqr_weighting_study.png.

Study B: robustness analysis
----------------------------
One hundred parameter realizations are generated using independent uniform
perturbations of up to +/-20 percent in every nonzero damping, restoring,
appraisal, emotional-coupling, cubic, and robot-effect coefficient. The
nominal LQR gain is deliberately retained in every trial. Thus, the analysis
tests robustness to modelling uncertainty; it does not retune the controller
for each perturbed model.

The script exports robustness_peak_summary.csv,
robustness_global_summary.csv, and robustness_study.png.

Important interpretation
------------------------
The bounded-response rate is a numerical simulation result over the stated
uncertainty set. It is not a formal proof of robust stability for every
possible parameter uncertainty.

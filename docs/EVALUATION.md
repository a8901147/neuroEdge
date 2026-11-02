# Evaluation plan

The demo task has been completed once, by one person (README). This plan turns that into measured results. Each
experiment states the question, the conditions, what is recorded with which tool, and how the result is read. Nothing
here has been run yet unless it says so.

## E1. Task success rate

**Question.** How often does the full task (hang → forward → left → grip → lift → right → place → release) succeed?

- 20 trials per session, two sessions on different days; one EMG and pose calibration at the start of each session.
- Per trial: success or the first failed step (reach, grip, hold while carrying, place, release), and the time taken.
  Video of every trial.
- Report the success rate with a 95 % Wilson interval and the failures by step. Twenty trials bound the rate to
  roughly ±20 %, which is enough to tell "usually works" from "sometimes works".
- Then repeat one session with a second person, to separate the method from one user's practice.

## E2. Why arm motion triggers the grip

**Question.** Are the EMG spikes during arm motion (PRD §7) from cable motion, from the arm-lifting muscles, or both?

Conditions, each recorded with `tools/capture_arm_motion.py --set emg` (relaxed and gripping, still and during the task
motion), which replays the grip decision and counts false grips and false releases:

1. as now;
2. sensor cable strain-relieved along the arm;
3. as 2, with the electrodes moved further onto the forearm flexors.

Reading the result: if 2 removes the spikes, cable motion was the cause. If spikes remain while the arm is held raised
and still, the arm's own muscles or the limb position change the signal. Fougner et al. (2011) showed that limb
position alone changes the EMG enough to raise pattern-recognition error from 3.8 % to 18 %, and reduced it by training
in several positions and adding accelerometer data. For a threshold controller, the analogue is a threshold that
depends on arm pose, which this hardware can compute from the upper-arm IMU.

## E3. Grip latency

**Question.** How long from the start of a contraction to the claw moving, and is it within what users tolerate?

- Expected from the code: about 10 ms EMA, 150 ms on-debounce, then the 50 ms slew to fully closed, and up to 20 ms to
  the next servo frame.
- Measure with video at 120 fps or more (the forearm and the claw in one frame) and, on the board side, the time from
  the first tick line above the threshold to `gripping=1`.
- Compare with the 100–125 ms that Farrell & Weir (2007) found best for myoelectric control. The debounce alone is
  above that range, so a shorter on-debounce with the two-threshold hysteresis is the first change to test, using E2's
  false-grip count as the check that it does not trade latency for misfires.

## E4. Elbow-bend accuracy

**Question.** How far is the elbow reading from the true elbow angle across arm poses?

The firmware takes the elbow bend as the angle between the two gravity vectors (`mearm_input_filter.hpp`). That is the
elbow angle only when the two sensor frames coincide with the arm straight and the elbow's axis is horizontal; a
mounting misalignment adds a pose-dependent offset, a bend about a vertical axis reads zero, and forearm rotation
changes the reading (`tools/test_analyze_elbow_bend.py` shows each case).

- Baseline from existing data: `python3 tools/analyze_elbow_bend.py` on `data/arm_motion_20261004-021240.json`. From
  hanging to horizontal the reading changed by −7.2° to +4.2° depending on the raise direction (about 5 % of the
  129° elbow range). The true elbow angle was not recorded, so this mixes offset drift with any real elbow motion.
- Measurement: a goniometer at 0°, 45° and 90° elbow flexion, with the upper arm hanging, raised forward and raised to
  the side, each with the forearm neutral, pronated and supinated.
- If the error matters for the task, the standard remedy is a sensor-to-segment calibration with full orientation from
  gyroscope and accelerometer fusion (Seel et al., 2014).

## E5. EMG level across posture, wear time and days

**Question.** How much does the relaxed and gripping EMG level move with posture, with time worn and between days, and
can a stored calibration be reused (`--skip-emg-calibration`)? So far this was only seen on the live readings (PRD §7):
the relaxed level about doubled on standing up, changed when the electrodes were put back on, and drifted within
minutes.

- Within one wearing: calibrate sitting, then standing, then sitting again every 5 minutes for 30 minutes, without
  touching the electrodes. Then take the electrodes off, put them back on and calibrate again.
- Every interactive calibration is logged; `tools/mujoco_bridge/summarize_emg_calibration_logs.py` lists them.
- Calibrate at the start and the end of each E1 session, on at least three days. Report the relaxed level, its spread,
  the threshold and the grip level per calibration, and whether each day's threshold would have worked on the other days
  (the grip level above it, the relaxed level below the release threshold).

## Beyond the current signal

The firmware reads the MyoWare's envelope (`ENV`), already rectified and low-passed in hardware: one number per moment,
enough for an on/off grip. Reading the `RAW` output at 1 kHz or more would allow digital band-pass filtering, the
time-domain features of Hudgins et al. (1993) (mean absolute value, waveform length, zero crossings, slope sign
changes), and with them proportional grip force or several grip classes. The repository's generic pipeline (`Pipeline`,
`LdaClassifier`, `include/edgeneuro/`) is built for that case but does not drive the arm today.

## References

- Farrell, T. R. & Weir, R. F. (2007). The optimal controller delay for myoelectric prostheses. *IEEE Transactions on
  Neural Systems and Rehabilitation Engineering*, 15(1), 111–118.
- Fougner, A., Scheme, E., Chan, A. D. C., Englehart, K. & Stavdahl, Ø. (2011). Resolving the limb position effect in
  myoelectric pattern recognition. *IEEE Transactions on Neural Systems and Rehabilitation Engineering*, 19(6),
  644–651.
- Hudgins, B., Parker, P. & Scott, R. N. (1993). A new strategy for multifunction myoelectric control. *IEEE
  Transactions on Biomedical Engineering*, 40(1), 82–94.
- Seel, T., Raisch, J. & Schauer, T. (2014). IMU-based joint angle measurement for gait analysis. *Sensors*, 14(4),
  6891–6909.

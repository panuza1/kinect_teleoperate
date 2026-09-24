# Calibration status

No physical Kinect calibration has been performed.  The bridge rejects a real
output profile and defaults to observe/no-publish mode.

When Kinect hardware is available, run only the `KINECT_HARDWARE_REQUIRED`
phases in `IMPLEMENTATION_PLAN.md`: record device serial and mount identity,
fit floor and heading, collect neutral-pose samples, persist the signed
calibration artifact, and replay that recording through the software tests.
Do not use a synthetic result as a physical calibration result.

The current SDK-to-SONIC transform is covered only by synthetic tests.  A
physical calibration must verify units, all axes, left/right signs, scale, and
floor residual before it can be used for simulation input.

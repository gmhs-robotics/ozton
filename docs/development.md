# Troubleshooting

| Symptom | What to check |
| --- | --- |
| An embedded path fails to load | Check the [export format and path limits](routes.md#import-a-pathjerryio-route). |
| A route returns `PathStart` | Check the surveyed starting pose and travel direction against [the coordinate convention](coordinates.md#place-the-robot-in-the-field-frame). |
| Pose jumps or stops updating | Check encoder polarity, IMU calibration, and [pose trust](localization.md#rest-consistency-and-pose-trust). |
| Range alignment never settles | Check the [selected sensor and approach direction](routes.md#try-alignment-and-continue), the line of sight, and the command timeout. |
| A calibration fit is underdetermined | Collect independent motions and enough samples for [the experiment you are fitting](calibration.md#collect-reference-samples). |

For a timing or actuator issue, [capture a diagnostic sample](calibration.md#collect-reference-samples)
and record the selected route, the observed behavior, and when it occurred.

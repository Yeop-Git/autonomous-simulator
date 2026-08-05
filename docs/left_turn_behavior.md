# Protected left-turn per-tick policy

The server owns left-turn arbitration. Unity sends the driver's intent; it
must never turn or change lanes directly in response to the UI button. The
policy evaluates all priorities again on every V2X tick. A phase is an output
and path guard, not a transition that forces the following phase.

The controller discovers route roles from lane topology and headings:

`source lane -> target approach -> left-turn connector -> exit lane`

No scenario lane ID is referenced by the policy. A successor whose heading is
closest to 90 degrees left of the approach is selected as the turn connector;
its successor becomes the exit lane. Signal timing remains external traffic
configuration keyed by the discovered target approach lane.

| `left_turn_phase` | Permitted motion | Exit guard |
| --- | --- | --- |
| `LaneChangeWaiting` | Stay in the current lane and decelerate | Current-lane leader is safe, target front/rear gaps pass, and no predicted conflict exists |
| `LaneChanging` | Follow only the smooth lateral blend; do not expose intersection waypoints | Vehicle is classified on the discovered target approach and aligned |
| `ApproachStopLine` | Follow the left lane toward the buffered stop point | Signal and entry checks are evaluated continuously |
| `SignalWaiting` | Path ends 5.5 m before the line; speed targets zero at the buffer | Protected arrow is green, leader has departed, crosswalk is clear, connector and exit have space |
| `IntersectionEntry` | Enable the protected connector path and start smoothly | Vehicle enters the discovered left-turn connector |
| `IntersectionCrossing` | Stay on the fixed turn connector, with ACC active | Vehicle reaches the discovered exit lane |
| `ExitAligning` | Center on the destination lane and unwind steering | Lateral error <= 0.5 m and heading error <= 8 degrees |
| `Completed` | Cancel the left indicator and return to normal driving | Terminal left-turn phase |
| `AbortedStraight` | Cancel the lane request and use the current lane's straight connector | Used when no safe gap exists by the last stable lane-change point |

Safety priority is current-lane emergency braking, rear-gap acceptance, target
lane gap, smooth lateral motion, signal/stop-line compliance, pedestrian and
exit clearance, intersection traversal, then exit-lane alignment. An active
lane change is never suddenly reversed: a newly detected hazard commands a
stop while retaining the existing lateral direction.

ACC uses a 6 m stopped bumper gap and a 2 s following headway. When the bumper
gap enters 6 m, the server commands target speed zero; Unity reaches zero under
its configured deceleration limit rather than snapping velocity instantly.

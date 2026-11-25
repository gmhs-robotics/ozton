# Mechanisms and background work

Keep a lift, intake or other mechanism in its own struct. The mechanism owns its motors, position references and mechanical limits. Your robot can borrow the drive and mechanisms separately when it needs to coordinate them.

An async function can issue a target and then await completion. Do not keep a mutable RefCell borrow alive while waiting. If you need motion and scoring in parallel, use short synchronous commands or a scoped TaskSlot with an owned handle.

TaskSlot work belongs to the current competition activation. It cannot keep running after a mode switch simply because the same mode becomes active again. Raw detached vexide tasks are not automatically scoped by Ozton.

Use the stop hook to define disabled and fault behavior, and mode_changed to clear transient software state. Do not treat a mode switch as a guarantee that a latched motor command is cleared when moving directly between enabled modes. The driver must take ownership of its outputs.


The device capabilities live in `ozton::drivetrain::device` (also exposed through
`runtime::device`). `VoltageOutput`, `ReadPosition`, `SetPosition`,
`ReadVelocity`, `ReadTorque` and `HasBraking` are independent interfaces.
`MotorOutput` combines voltage output and braking with matching device error
types. `PidMotor::apply` accepts external feedback, while `update` additionally
requires position and velocity readers. Neither interface requires a VEX motor. Tracking wheels and mechanism encoders
share the same SI-valued position interface, including simulated sensors.

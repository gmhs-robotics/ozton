# Mechanisms and background work

Keep a lift, intake or other mechanism in its own struct. The mechanism owns its motors, position references and mechanical limits. Your robot can borrow the drive and mechanisms separately when it needs to coordinate them.

An async function can issue a target and then await completion. Do not keep a mutable RefCell borrow alive while waiting. If you need motion and scoring in parallel, use short synchronous commands or a scoped TaskSlot with an owned handle.

TaskSlot work belongs to the current competition activation. It cannot keep running after a mode switch simply because the same mode becomes active again. Raw detached vexide tasks are not automatically scoped by Ozton.

Use the stop hook to define disabled and fault behavior, and mode_changed to clear transient software state. Do not treat a mode switch as a guarantee that a latched motor command is cleared when moving directly between enabled modes. The driver must take ownership of its outputs.

For the integrated builder, see the Mechanisms and Program implementations in the [programmed example](https://github.com/gmhs-robotics/ozton/blob/fix/motion-convergence/packages/ozton/examples/programmed.rs).

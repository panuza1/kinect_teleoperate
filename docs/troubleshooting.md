# Troubleshooting

`HARDWARE_VALIDATION_REQUIRED` means software did not attempt to open or
control the missing hardware. It is not a passing hardware result.

For a bridge build failure, first validate the SDK CMake prefix and ZeroMQ
include/library paths. For a failed offline configuration report, remove
unknown keys or correct the strict JSON schema; do not disable safety fields.
For a CTest failure, preserve its trace/report and fix the earliest failing
boundary before retrying later phases. A simulator reset, a raw target limit
violation, or a stale command is a failure signal, not a reason to widen a
limit or hide the event.

If SONIC stops with `target derivatives limited`, inspect the requested and
applied command trace; the final guard is intentionally smoothing the command.
If it reports `position ... outside [...]`, the policy produced an invalid raw
target. Do not rely on MuJoCo clipping. Preserve the input trace, controller
logs, model hashes, joint name, target and bounds, then fix the policy/action
contract or safely restrict the supported motion envelope.

If the controller stops while waiting for start, verify it was built after the
WAIT-state hold-refresh fix. The independent writer watchdog must still reject
a genuine control-thread stall after 100 ms.

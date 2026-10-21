# HKG CAN-FD low-speed torque compatibility

The HKG low-speed torque toggle applies to compatible Hyundai, Kia, and Genesis **CAN-FD** platforms, not only the Kia EV6. The selected vehicle must pass `supports_low_speed_torque(CP)` at initialization. This is based on the actual fingerprinted CarParams and Panda safety configuration, not the model's displayed name.

## Opt-in envelope

- At 0–13 m/s (0–46.8 km/h): maximum steering command 409, up/down rates 4/6 per controller step.
- 13–17 m/s: torque linearly reduces from 409 to 270 and rates transition from 4/6 to 2/3.
- At 17 m/s and above: maximum steering command 270 and rates 2/3.

The controller and Panda safety must agree on the same dynamic profile; the feature defaults to **off** and takes effect on the next vehicle initialization.

## Exclusions and validation

Traditional CAN Hyundai/Kia/Genesis platforms are intentionally excluded: they have a different steering command, model-dependent maximums and Panda safety policies, so applying the CAN-FD 409/4/6 limits would be unsafe. Angle-steering, dashcam-only, alternative-limit, unknown-fingerprint, or incompatible Panda safety configurations are also excluded.

No EPS angle/timing protection is removed. A supported software fingerprint is **not evidence** that a given car's EPS can sustain the new torque throughout a tight bend. Validate the actual platform, panda safety, driver override, and CAN/rlog traces before enabling this experimental feature on the road. Never mask an AEB/FCA fault or misrepresent a vehicle's stock collision-avoidance capability.

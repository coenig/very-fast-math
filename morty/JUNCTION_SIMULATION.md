# Four-arm junction simulation

The highway-env junction is opt-in through the Boolean `UCD_JUNCTION` setting.
Use the model profile `master_templates/junction_tpl.json` as the active
`envmodel_config.tpl.json` when running it; the checked-in active config keeps
the flag disabled by default. The profile uses priority model selection, so run
Morty with `--live` (for example, `python -m morty.morty --live`).

The simulator supports two or more configured cars. Even IDs travel straight
west-to-east (sections 0→1), and odd IDs travel straight north-to-south
(sections 2→3); odd vehicle counts are allowed, so the two routes may have
unequal traffic. The model profile and road geometry must use the same
`NONEGOS` count.
The world origin is the crossing center, with east as +x and north as +y.
The model section centerlines extend 100 m on each side. Highway-env lane
centers use the same lateral offsets as the model: `-2 + 4*lane` meters for
the two-lane profile. For another lane count, use `-2*(lane_count-1) +
4*lane` on each route and shift each fixed model section's source y by
`-2*(lane_count-1)` meters from the respective centerline y (0 m for
sections 0, 1, 3; 100 m for section 2). The native model geometry and
`NUMLANES` must be changed together.

Cars are seeded on their inbound approaches behind the model's occupancy
threshold by a conservative stopping distance, with at least 25 m between
vehicles on the same route. Morty derives the threshold from the largest
vehicle length and width in the prioritized configurations using the same
scaled integer-rounding formula as the model:
`R = round(((N*lane_width_cm + vehicle_width_cm)/100)*(D/1000)) + round(vehicle_length_m*(D/1000)) + 2`;
the inbound occupancy threshold is `(R + round(vehicle_length_m*(D/1000))) /
(D/1000)` metres. At two lanes, 4 m lane width, and distance scaling 4000,
the threshold is 24.75 m for the 6x6 model profile (7 m by 2.2 m) and 48 m for
9x9 (18 m by 3.4 m). Spawn speed is capped when
needed to fit the threshold, stopping distance, and safety margin on the
100 m arms; configurations/counts that cannot fit the requested minimum speed
and same-route spacing fail explicitly. Random lane and station choices use
the run seed. The shipped four-car profile requests 5–15 m/s; with the largest
prioritized 9x9 vehicle, the safe cap is about 9.282 m/s, so initialization
uses a lower speed cap without weakening model collision constraints. A
focused test loads that exact profile and initializes/steps a real
four-vehicle highway-env instance at its worst-case dimensions. The Python
controller projects observed positions, velocities, headings, and
model-produced world-coordinate trajectory points into each vehicle's route
frame before calculating longitudinal acceleration and pure-pursuit steering.
Native model input remains global `presence,x,y,vx,vy,heading`.

Experiment 10 requires every configured vehicle to be on its parity-assigned
exit and more than 40 m beyond the crossing, within that road's lateral bounds
and with no crashes (the model threshold is `40 * (DISTANCESCALING / 1000)`,
e.g. 160 model units at a scale of 4000). `TIMESCALING=2000` represents 2 steps/s,
so one step is 0.5 s; with distance factor 4, velocity factor is 4/2 = 2.
The simulator rejects
profiles with a different vehicle count, backward-driving IDs, a different
experiment, shortcuts, or lane width other than 4 m rather than silently
using an incompatible route or geometry.

The junction profile enables `UCD_ROUND_INPUTS`: at 4 model units/m, rounding
longitudinal inputs to the nearest model unit contributes at most 0.125 m of
position quantization. Crossing reach and same-route longitudinal clearance
also include two model units (0.5 m) for geometry/position rounding. The
same-route lateral envelope adds vehicle length, width, and the lateral-bin
margin; together with the crossing reach this encloses the 5 m by 2 m
highway-env vehicle's projected footprint for arbitrary heading at the
sampled model state. This is a geometric envelope, not a bound on tracking
deviation between samples. Junction input mapping disables heading adaptation
and rejects positions outside the configured lateral road band or more than
1 m beyond a finite route endpoint instead of clamping them onto the road.
The connected section boundary and valid connector travel remain supported.

The junction-only longitudinal model now advances by the integer midpoint of
current and next velocity. Solver-backed tests cover acceleration, braking,
odd midpoint rounding, and a rear-accelerate/front-brake clearance boundary.
A highway-env integration test compares four acceleration/deceleration cases
over 30 `Vehicle.step(1/60)` substeps; for straight, unclipped, zero-steering
motion, velocity matches and longitudinal position differs by at most 0.25 m.
This test does not bound steering, clipping, controller tracking, or physical
plant uncertainty.

The live junction summary reports crashes and minimum oriented-body clearance
sampled at every highway-env pairwise collision check (60 Hz), before collision
resolution. This is sampled clearance, not a continuous swept-body proof.
The no-plan guard refuses to call `env.step` when the planner has no usable
plan. An isolated Windows live run used the final strengthened 6x6 model,
two cars, BMC 45, 5 m/s fixed speed, and the model witness's staggered starts:
car 609 at (-40,-2) m in section 0 with model lane bit `b1`; car 619 at
(2,70) m in section 2 with model lane bit `b0`. The generated
`main_live.smv` INIT clauses were verified against the witness before
releasing the solver. After the fixed 300 s wall-clock
budget, eight planning-input cycles had started; the final input was at
positions 300/180 model units (15 m forward from each initial station), and
the last solver query was still running. No live goal was reached and the
timeout prevented a terminal crash/clearance summary, so clearance and final
crash status are unavailable. This demonstrates that integrated repeated
replanning, not the standalone witness, remains the blocker: a separate
model-only query found the actual goal at bound 17 in 30.17 s with collision
rules intact. It is not a live safety proof. The shipped profile was not
narrowed and no runnable-default claim is made. Real-plant operation is
unsupported unless an independent safety supervisor and validated uncertainty
bounds are provided.

Focused checks: `python -m unittest morty.test_junction morty.test_junction_smv`.

# Junction Model-Checking Optimization Discussion

Conversation from November 27-28, 2026. This is a discussion and proposal, not a record of implemented or benchmarked changes.

## User

How could we make the model checking runs for task 11/UCD faster? Are there any specific optimizations which we could use just for this very use case?

## GitHub Copilot

I'd start with a **task-11-specific model encoding**, then repair shortcuts. Both target work repeated on nearly every cycle.

The active setup already uses priority selection, persistent nuXmv sessions and the compact encoding, but replans with eight cars and bound 50. In the latest ten-run results, priority 0 was selected in 340 of 376 recorded cycles; three cycles were blind. The runtime PDFs do not provide reliable per-query timings for selective planning, so these counts do not establish where the time went.

**Best candidates**

1. **Simplify the junction model.** The generated model has 1,600 crossing-overlap definitions: 25 lane-position pairs for each of 64 ordered car pairs. The fixed geometry and straight-through routes permit factoring lane-dependent crossing positions and eliminating impossible pairs. The 200 self-pair definitions are an immediately testable simplification; pruning further by car route requires proving observations and recycling respect that assignment. Preserve all collision constraints, including any desired between-step protection. See [EnvModel_Behavior_Nonego.tpl](../src/templates/EnvModel_Behavior_Nonego.tpl#L364).

2. **Compact road-location state.** Each car has section/connector variables plus 16 auxiliary location variables; the current fixed junction only needs two straight routes. A specialized route-progress representation could eliminate many impossible connectors and their transition cases. This is a larger change: preserve boundary behavior, observations and trace consumers, and check equivalence against the existing model.

3. **Make shortcuts junction-aware.** They are currently disabled, and [TODO.txt](TODO.txt) says they do not work for task 11. Saved waypoints include position, speed and lane, but not section or connector identity ([interactive_testing.cpp](../src/interactive_testing.cpp#L2303)). Include location identity and reconstruct the tail in the correct road frames; validate stitching and invalidate plans on recycling. Once correct, short bounded probes could avoid many full replans, but their benefit is unmeasured for task 11.

**Practical next step:** capture representative archived task-11 queries, including crossings and failed priorities, then benchmark each encoding against identical observations and goals. Compare feasibility, witness depth and replayed trajectories, followed by closed-loop crash and blind rates. The task-7 6.7x result is encouraging but is **not** a task-11 speedup estimate.

For faster exploratory runs today, `--live` removes diagnostics, but cannot be combined with your video or detailed-archive flags. I would retain all 16 fallbacks and bound 50 until task-11 evidence supports changing them; reducing either changes planning behavior. No files were changed or new benchmarks run.

## User

For candidate 1, what do you mean by "recycling respect that assignment"?

## GitHub Copilot

I meant that each vehicle's route assignment must remain true after it is teleported back for another crossing.

In [recycle_junction_vehicles](morty_helper.py#L198), even-indexed cars are recycled to section 0 and odd-indexed cars to section 2. That suggests even cars follow 0 -> 1 and odd cars follow 2 -> 3.

Before removing collision checks based on that assumption, we'd need to verify that the **observed model input and every allowed model transition** also keep each car on its assigned route. Recycling is one part of that guarantee; the input mapping and model rules matter too. Otherwise, a valid modeled state could lose a needed collision check.

## User

how many definitions could we get rid of, in the best case?

## GitHub Copilot

With eight cars, assuming four are permanently on each straight-through route:

- **Prune impossible car pairs:** 1,600 -> 400 definitions, eliminating **1,200 (75%)**. That leaves 4 x 4 crossing pairs, each with 25 lane combinations.
- **Also factor the lane combinations:** potentially 400 -> 16 crossing definitions, one per crossing pair: **1,584 fewer (99%)** overall. This would introduce some shared lane-dependent expressions, so it does not mean 99% less solver work.

Both estimates depend on preserving the current collision semantics; the first also depends on validating the route-assignment assumption.

## User

would this necessarily mean "task 11"-specific optimizations on MC side? I'd hope for a weaker assumption, such as: If the crossing is between two uni-directional straight seclet sequences which intersect perpendicularly, and the cars don't turn...?

## GitHub Copilot

No, this need not be task-11-specific. The optimization could apply to any fixed geometry with two perpendicular, unidirectional straight routes and no turns between them.

There are two distinct opportunities:

- **Factor lane combinations:** use the crossing geometry to share lane-dependent calculations. This does not require assigning cars to routes permanently.
- **Prune car pairs:** additionally prove from each car's observed location and allowed transitions that it cannot switch routes during the planning horizon. "Cars don't turn" alone doesn't identify which route each car occupies.

For the current eight-car setup, the 1,600-to-400 estimate assumes four cars are proven to remain on each route. If that cannot be established, the generic factored encoding can still retain checks for all potentially crossing pairs. Recycling happens outside the model, so the route proof must be re-established after a new observation.

## User

What would change if we did allow cars to turn, i.e. 0->3, 2->1? Collision checks become more complicated, but we could still use axis-aligned bounding boxes as approximation.

## GitHub Copilot

Yes. Turning removes the simple assumption that each car stays on one straight route, but the optimization can still be geometry-driven rather than task-specific.

- **Pair pruning becomes dynamic.** A car approaching on section 0 might exit on 1 or 3, so we can only omit a pair if their *reachable* paths cannot conflict within the bound.
- **Collision checks need position and orientation along turns.** A world-axis-aligned bounding box can conservatively enclose the rotated vehicle at each point on the connector. Precomputed, conservative piecewise bounds could keep the solver's arithmetic manageable, at the cost of false collision reports.
- **Merges matter too.** Cars taking different routes may enter the same outgoing section, so checks must cover turns, straight crossings and merges, including vehicle footprints spanning seclet boundaries.
- **Between-step motion still matters.** AABBs checked only at discrete states can miss collisions between states. Conservative swept bounds are needed to cover the full step.

The likely tradeoff is a simpler, more general collision encoding versus conservatism: oversized boxes may block valid plans, especially in a tight junction. Whether it is faster than the current specialized crossing checks would need benchmarking.

## User

But you'd say it's possible that it could even become faster, rather then slower?

## GitHub Copilot

Yes, it's possible. A compact, precomputed AABB encoding could be faster than the current 1,600 crossing definitions, even with turns enabled.

But turns add possible routes and collision cases, while conservative boxes can make finding a valid plan harder. The net effect depends on the encoding and scenario, so I'd treat a speedup as a plausible hypothesis to benchmark, not an expectation.

-- Fixed endpoint-aware, single-lane movement relation.
-- The graph compiler supplies ENDPOINT_TRANSITIONS for the selected edges only.
-- It assigns next position, velocity, section and connector together, converting
-- signed local coordinates at start/start and end/end connections.
-- Legacy movement assignments are omitted when ENDPOINT_CONNECTIONS is enabled.
-- ENDPOINT_TRANSITIONS includes incident-road boundary clearance and swept
-- attachment checks supplied by the endpoint compiler (also in baseline mode).
-- They are environment constraints, independent of the scenario monitor.
@{ENDPOINT_TRANSITIONS}@.printHeap

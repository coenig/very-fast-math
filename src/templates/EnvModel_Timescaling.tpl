@{@{scalingList}@*.pushBack[-- #0#,#1#]#0#}@***.newMethod[scalingVariable, 1]

@{@{#0# * TIMESCALING}@**.eval[0]}@***.newMethod[timeWorldToEnvModelConst, 0]
@{@{#0# * DISTANCESCALING}@**.eval[0]}@***.newMethod[distanceWorldToEnvModelConst, 0]
@{@{#0# * DISTANCESCALING / TIMESCALING}@**.eval[0]}@***.newMethod[velocityWorldToEnvModelConst, 0]
@{@{#0# * DISTANCESCALING / TIMESCALING / TIMESCALING}@**.eval[0]}@***.newMethod[accelerationWorldToEnvModelConst, 0]

@{@{#0#}@*.scalingVariable[time] := @{#1#}@*.timeWorldToEnvModelConst}@**.newMethod[timeWorldToEnvModelDef, 1]
@{@{#0#}@*.scalingVariable[distance] := @{#1#}@*.distanceWorldToEnvModelConst}@**.newMethod[distanceWorldToEnvModelDef, 1]
@{@{#0#}@*.scalingVariable[velocity] := @{#1#}@*.velocityWorldToEnvModelConst}@**.newMethod[velocityWorldToEnvModelDef, 1]
@{@{#0#}@*.scalingVariable[acceleration] := @{#1#}@*.accelerationWorldToEnvModelConst}@**.newMethod[accelerationWorldToEnvModelDef, 1]

-- TIMESCALING((((~{TIMESCALING}~))))GNILACSEMIT
-- DISTANCESCALING((((~{DISTANCESCALING}~))))GNILACSECNATSID
-- SCALING DESCRIPTIONS
@{scalingList}@.printList
-- EO SCALING DESCRIPTIONS

-- Translating TIMESCALING unit from milliseconds (old value: @{TIMESCALING}@*****.eval) to seconds (new value: @{@TIMESCALING = TIMESCALING / 1000; TIMESCALING}@*****.eval)
-- Translating DISTANCESCALING unit from millimeters (old value: @{DISTANCESCALING}@*****.eval) to meters (new value: @{@DISTANCESCALING = DISTANCESCALING / 1000; DISTANCESCALING}@*****.eval)

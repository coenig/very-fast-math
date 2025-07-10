------------------------------------------------------------------------------------------------------------------
-- MATH functions on vfm level which are directly converted to SMV code without further modularizing on their side
------------------------------------------------------------------------------------------------------------------

-- The vfmacro function 'syntacticSegmentAndLineIntersect' is an implementation of this C++ code:
-- bool onSegment(Vec2D p, Vec2D q, Vec2D r) {
--    return (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
--            q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y));
-- }
-- bool doesLineIntersectSegment(Vec2D L1, Vec2D L2, Vec2D M1, Vec2D M2, Image&& img) {
--    float A1 = L2.y - L1.y;
--    float B1 = L1.x - L2.x;
--    float C1 = (L2.x * L1.y) - (L1.x * L2.y);
--    float A2 = M2.y - M1.y;
--    float B2 = M1.x - M2.x;
--    float C2 = (M2.x * M1.y) - (M1.x * M2.y);
-- 
--    float denominator = A1 * B2 - A2 * B1;
-- 
--    if (denominator == 0) throw std::runtime_error("The lines are parallel and do not intersect.");
-- 
--    float x = -(B1 * (-C2) - B2 * (-C1)) / denominator;
--    float y = -(A2 * (-C1) - A1 * (-C2)) / denominator;
-- 
--    // Check if the intersection Vec2D lies on the line segment
--    return onSegment(M1, {x, y}, M2);
-- }

@{@{@f(vec, x, y, 0); @f(line, x, y, 0); @f(lines, x, y, 0)}@.eval}@.nil

@{ ((#0#) * (#0#)) }@***.newMethod[syntacticSquare, 0]
@{ (@{@{#1#}@.atVfmTupel[0] - @{#0#}@.atVfmTupel[0]}@.syntacticSquare + @{@{#1#}@.atVfmTupel[1] - @{#0#}@.atVfmTupel[1]}@.syntacticSquare) }@***.newMethod[syntacticSquareOfVecDistance, 1]

@{ ( abs( @{#0#}@.atVfmTupel[0] - @{#1#}@.atVfmTupel[0] ) + abs( @{#0#}@.atVfmTupel[1] - @{#1#}@.atVfmTupel[1] ) ) }@***.newMethod[syntacticManhattenDistance, 1]
@{ ( max( abs( @{#0#}@.atVfmTupel[0] - @{#1#}@.atVfmTupel[0] ), abs( @{#0#}@.atVfmTupel[1] - @{#1#}@.atVfmTupel[1] ) ) ) }@***.newMethod[syntacticMaxCoordDistance, 1]

@{ ((@{#0#}@.atVfmTupel[0] <= max(@{#1#}@.atVfmTupel[0].atVfmTupel[0], @{#1#}@.atVfmTupel[1].atVfmTupel[0]) && @{#0#}@.atVfmTupel[0] >= min(@{#1#}@.atVfmTupel[0].atVfmTupel[0], @{#1#}@.atVfmTupel[1].atVfmTupel[0]) && @{#0#}@.atVfmTupel[1] <= max(@{#1#}@.atVfmTupel[0].atVfmTupel[1], @{#1#}@.atVfmTupel[1].atVfmTupel[1]) && @{#0#}@.atVfmTupel[1] >= min(@{#1#}@.atVfmTupel[0].atVfmTupel[1], @{#1#}@.atVfmTupel[1].atVfmTupel[1]))) }@***.newMethod[syntacticIsPointOnSegment, 1]

@{ (@{#0#}@.atVfmTupel[1].atVfmTupel[1] - @{#0#}@.atVfmTupel[0].atVfmTupel[1]) }@***.newMethod[syntacticGFCoeffA, 0]
@{ (@{#0#}@.atVfmTupel[0].atVfmTupel[0] - @{#0#}@.atVfmTupel[1].atVfmTupel[0]) }@***.newMethod[syntacticGFCoeffB, 0]
@{ ((@{#0#}@.atVfmTupel[1].atVfmTupel[0] * @{#0#}@.atVfmTupel[0].atVfmTupel[1]) - (@{#0#}@.atVfmTupel[0].atVfmTupel[0] * @{#0#}@.atVfmTupel[1].atVfmTupel[1])) }@***.newMethod[syntacticGFCoeffC, 0]
@{ (@{@{#0#}@.atVfmTupel[0]}@.syntacticGFCoeffA * @{@{#0#}@.atVfmTupel[1]}@.syntacticGFCoeffB - @{@{#0#}@.atVfmTupel[1]}@.syntacticGFCoeffA * @{@{#0#}@.atVfmTupel[0]}@.syntacticGFCoeffB) }@***.newMethod[syntacticIntersectionDenominator, 0]
@{ -(@{@{#0#}@.atVfmTupel[0]}@.syntacticGFCoeffB * (-@{@{#0#}@.atVfmTupel[1]}@.syntacticGFCoeffC) - @{@{#0#}@.atVfmTupel[1]}@.syntacticGFCoeffB * (-@{@{#0#}@.atVfmTupel[0]}@.syntacticGFCoeffC)) / @{#0#}@.syntacticIntersectionDenominator }@***.newMethod[syntacticLinesIntersectX, 0]
@{ -(@{@{#0#}@.atVfmTupel[1]}@.syntacticGFCoeffA * (-@{@{#0#}@.atVfmTupel[0]}@.syntacticGFCoeffC) - @{@{#0#}@.atVfmTupel[0]}@.syntacticGFCoeffA * (-@{@{#0#}@.atVfmTupel[1]}@.syntacticGFCoeffC)) / @{#0#}@.syntacticIntersectionDenominator }@***.newMethod[syntacticLinesIntersectY, 0]
@{ @{vec(@{#0#}@.syntacticLinesIntersectX; @{#0#}@.syntacticLinesIntersectY)}@.syntacticIsPointOnSegment[@{#0#}@.atVfmTupel[0]] }@***.newMethod[syntacticSegmentAndLineIntersect, 0]


--------------------
-- EO MATH functions 
--------------------

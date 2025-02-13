------------------------------------------------------------------------------------------------------------------
-- MATH functions on vfm level which are directly converted to SMV code without further modularizing on their side
------------------------------------------------------------------------------------------------------------------

-- (Will not be displayed in final SMV file.)
bool doesLineIntersectSegment(Vec2D L1, Vec2D L2, Vec2D M1, Vec2D M2, Image& img) {
   float A1 = L2.y - L1.y;
   float B1 = L1.x - L2.x;
   float C1 = (L2.x * L1.y) - (L1.x * L2.y);
   float A2 = M2.y - M1.y;
   float B2 = M1.x - M2.x;
   float C2 = (M2.x * M1.y) - (M1.x * M2.y);

   float denominator = A1 * B2 - A2 * B1;

   if (denominator == 0) throw std::runtime_error("The lines are parallel and do not intersect.");

   float x = -(B1 * (-C2) - B2 * (-C1)) / denominator;
   float y = -(A2 * (-C1) - A1 * (-C2)) / denominator;

   // Check if the intersection Vec2D lies on the line segment
   return onSegment(M1, {x, y}, M2);
}

@{ ((#0#) * (#0#)) }@***.newMethod[syntacticSquare, 0]
@{ (@{#1#.x-#0#.x}@.syntacticSquare + @{#1#.y-#0#.y}@.syntacticSquare) }@***.newMethod[syntacticSquareOfVecDistance, 1]

@{ ((@{#0#}@.split[;].at[0] <= max(@{#1#}@.split['].at[0].split[;].at[0], @{#1#}@.split['].at[1].split[;].at[0]) & @{#0#}@.split[;].at[0] >= min(@{#1#}@.split['].at[0].split[;].at[0], @{#1#}@.split['].at[1].split[;].at[0]) & @{#0#}@.split[;].at[1] <= max(@{#1#}@.split['].at[0].split[;].at[1], @{#1#}@.split['].at[1].split[;].at[1]) & @{#0#}@.split[;].at[1] >= min(@{#1#}@.split['].at[0].split[;].at[1], @{#1#}@.split['].at[1].split[;].at[1]))) }@***.newMethod[syntacticIsPointOnSegment, 1]

@{ (#0#.end.y - #0#.begin.y) }@***.newMethod[syntacticGFCoeffA, 0]
@{ (#0#.begin.x - #0#.end.x) }@***.newMethod[syntacticGFCoeffB, 0]
@{ ((#0#.end.x * #0#.begin.y) - (#0#.begin.x * #0#.end.y)) }@***.newMethod[syntacticGFCoeffC, 0]
@{ (@{#0#.line1}@.syntacticGFCoeffA * @{#0#.line2}@.syntacticGFCoeffB - @{#0#.line2}@.syntacticGFCoeffA * @{#0#.line1}@.syntacticGFCoeffB) }@***.newMethod[syntacticIntersectionDenominator, 0]
@{ -(@{#0#.line1}@.syntacticGFCoeffB * (-@{#0#.line2}@.syntacticGFCoeffC) - @{#0#.line2}@.syntacticGFCoeffB * (-@{#0#.line1}@.syntacticGFCoeffC)) / @{#0#}@.syntacticIntersectionDenominator }@***.newMethod[syntacticIntersectionX, 0]
@{ -(@{#0#.line2}@.syntacticGFCoeffA * (-@{#0#.line1}@.syntacticGFCoeffC) - @{#0#.line1}@.syntacticGFCoeffA * (-@{#0#.line2}@.syntacticGFCoeffC)) / @{#0#}@.syntacticIntersectionDenominator }@***.newMethod[syntacticIntersectionY, 0]

--------------------
-- EO MATH functions 
--------------------

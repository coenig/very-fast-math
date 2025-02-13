------------------------------------------------------------------------------------------------------------------
-- MATH functions on vfm level which are directly converted to SMV code without further modularizing on their side
------------------------------------------------------------------------------------------------------------------

-- (Will not be displayed in final SMV file.)

sqrt := case
   @{veh___6[i]9___.v = [x] : @{[x] ** 2}@.eval[0];
   }@*.for[[x], 0, @{@{MAXSPEEDNONEGO}@***.velocityWorldToEnvModelConst - 1}@.eval[0]]
   TRUE: @{@{MAXSPEEDNONEGO}@***.velocityWorldToEnvModelConst ** 2}@.eval[0];
esac;

@{ ((#1#-#3#) * (#1#-#3#) + (#2#-#4#) * (#2#-#4#)) }@***.newMethod[squareOfDistance, 4]

--------------------
-- EO MATH functions 
--------------------

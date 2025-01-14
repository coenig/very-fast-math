# Model Checking for ADAS
This is the first version of the open source project first announced in *Towards Safe Autonomous Driving: Model Checking a Behavior Planner during Development* (https://link.springer.com/chapter/10.1007/978-3-031-57249-4_3). 
We make all the sources of the model checking toolchain public which have so far been published in binary format only (https://zenodo.org/records/10013662).
TODO: This is a preliminary readme file that will be replaced with a more meaningful one, soon.

Thank you for using
~~~
         ___           
.--.--..'  _|.--------.
|  |  ||   _||        |
 \___/ |__|  |__|__|__|
   very fast math
~~~

# VFM Library
The `vfm` library provides in its core a C++ representation of arithmetic/logic formulas. 
The formula structure is Turing-complete, i.e., provides a full programming language.
It can be used to parse C++ code and translate it into a model for the nuXmv model checker.
Optionally, an environment model can be integrated into the model, providing a discrete traffic simulation for the model checker.

More info to come...
# Examples
![Image of a counterexample generated through model checking](examples/cex.png?raw=true "Title")

# Authors
Lukas Koenig,
Alexander Georgescu,
Christian Heinzemann,
Christian Schildwaechter,
Michaela Klauck,
Henning Koch
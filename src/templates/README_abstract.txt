To run abstract scenario generation you need to edit `envmodel_config.tpl.json` setting the CONCRETE_MODEL to false.

"CONCRETE_MODEL" : false

To change the amount of cars, sections you can edit the file changing @sections and  @nonegos.
The abstract model uses the parameter MAXSECPARTS, which represents the amount of logical parts of sections. 
Speed is modelled by using stuttering, at each step either the ego move from section part i to section part i+1 or
it stutters (which in the concrete model means that it is not fast enough to pass to the next part of the section).

Currently the abstract model supports only 1 lane and 1 segment for section.
If you wish to extend the abstract model to support multiple lane, you need to update the behind constraints (see EnvModel_Abstract_Macros.tpl) to allow overtake.

To launch the generation, you need to be in bin folder and execute 

./vfm --execute-script

Multiple configurations will be generated (envmodel_config.tpl.json uses range of parameters) in examples/ folder and model checking query will be executed.

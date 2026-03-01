@{../src/templates/}@.stringToHeap[MY_PATH]

@{nuXmv}@.killAfter[3600].Detach.setScriptVar[scriptID, force]

@{envmodel_config.tpl.json}@.generateEnvmodels
@{envmodel_config.tpl.json}@.runMCJobs[10]
@{envmodel_config.tpl.json}@.generateTestCases[all]

@{scriptID}@.scriptVar.StopScript

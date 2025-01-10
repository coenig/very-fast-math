//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "model_checking/yaaa_activity.h"
#include "failable.h"
#include "fsm.h"
#include <fstream>
#include <vector>
#include <string>
#include <cctype>

namespace vfm {

class YaaaGraph : public Failable {
public:
   YaaaGraph();
   YaaaGraph(const std::string& json_file);

   void addActivity(const YaaaActivityPtr activity);
   YaaaActivityPtr getActivityByName(const std::string& name);
   YaaaRunnablePtr getRunnableByName(const std::string& name);
   void addConnection(const std::string& from, const std::string& to);
   std::string getGraphviz();
   void createGraphvizDot(const std::string& base_filename, const std::string& format = "pdf");

   /// \brief Parses a json file as created by the yaaa compiler using this command:
   /// yaaac applications/aca5/deployment/arch/InlaneDriving.activity_graph.yaml -I applications/aca5 -r -o yaaac-test -j
   void loadFromFile(const std::string& json_file);
   void addSendConditions(const std::string& root_path);

   fsm::FSMs createStateMachineForMC(const std::string& base_file_name) const;

   //void setOutputLevels(const ErrorLevelEnum toStdOutFrom = ErrorLevelEnum::note, const ErrorLevelEnum toStdErrFrom = ErrorLevelEnum::error) override;

private:
   std::vector<YaaaActivityPtr> activities_;
   std::vector<std::pair<std::string, std::string>> connections_;
   void addSendConditionsToRunnable(YaaaRunnablePtr runnable, const std::string& file_name);
};

extern "C" const char* yaaaModelCheckerWrapper(const char* yaaa_json_filename, const char* root_path, const bool debug = false); /// \brief Don't forget to free afterward.
extern "C" void freeModelCheckerReturnValue(); /// \brief Call this after yaaaModelCheckerWrapper since the result will leak otherwise.

} // vfm
//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2025 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "model_checking/mc_workflow.h"
#include "static_helper.h"
#include "testing/interactive_testing.h"

using namespace vfm;
using namespace mc;

McWorkflow::McWorkflow() : Failable("McWorkflow")
{
}

std::vector<std::string> McWorkflow::runMCJobs(const std::filesystem::path& working_dir, const std::function<> job_selector)
{
   std::vector<std::string> possibles{};
   std::string prefix{ working_dir.filename().string() };

   for (const auto& entry : std::filesystem::directory_iterator(working_dir)) {
      std::string possible{ entry.path().filename().string() };
      if (std::filesystem::is_directory(entry) && possible != prefix && StaticHelper::stringStartsWith(possible, prefix)) {
         possibles.push_back(entry.path().string());
      }
   }

   { // Scope which makes us wait for finishing the jobs before entering the next line.
      ThreadPool pool(test::MAX_THREADS);

      for (const auto& folder : possibles) {
         std::this_thread::sleep_for(std::chrono::seconds(1));

         const std::string config_name{ std::filesystem::path(folder).filename().string() };

         if (StaticHelper::isBooleanTrue(mc_scene->getOptionFromSECConfig(config_name, SecOptionLocalItemEnum::selected_job))) {
            pool.enqueue([mc_scene, folder, working_dir, prefix] {
               MCScene::runMCJob(mc_scene, folder, folder.substr(working_dir.string().size() + prefix.size() + 1));
               });
         }
      }
   }

   return possibles;
}

//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2025 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "model_checking/mc_workflow.h"
#include "static_helper.h"
#include "testing/interactive_testing.h"
#include "vfmacro/script.h"

using namespace vfm;
using namespace mc;

McWorkflow::McWorkflow() : Failable("McWorkflow")
{
}

std::vector<std::string> McWorkflow::runMCJobs(
   const std::filesystem::path& working_dir, 
   const std::function<bool(const std::string& folder)> job_selector, 
   const std::string& path_template)
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

         if (job_selector(config_name)) {
            pool.enqueue([this, folder, working_dir, prefix, path_template] {
               runMCJob(folder, folder.substr(working_dir.string().size() + prefix.size() + 1), path_template);
            });
         }
      }
   }

   return possibles;
}

void McWorkflow::runMCJob(const std::string& path_generated_raw, const std::string& config_name, const std::string& path_template)
{
   std::string path_generated{ StaticHelper::replaceAll(path_generated_raw, "\\", "/") };

   {
      std::lock_guard<std::mutex> lock{ main_file_mutex_ };

      addNote("Running model checker and creating preview for folder '" + path_generated + "' (config: '" + config_name + "').");
      deleteMCOutputFromFolder(path_generated, true);
      mc_scene->preprocessAndRewriteJSONTemplate();

      if (!mc_scene->putJSONIntoDataPack()) return;
      if (!mc_scene->putJSONIntoDataPack(config_name)) return;

      std::string main_smv{ StaticHelper::readFile(path_generated + "/main.smv") };

      std::string script_template{ StaticHelper::readFile(mc_scene->getTemplateDir() + "/script.tpl") };
      std::string main_template{ StaticHelper::readFile(mc_scene->getTemplateDir() + "/main.tpl") };
      std::string generated_script{ CppParser::generateScript(script_template, mc_scene->data_, mc_scene->parser_) };
      StaticHelper::writeTextToFile(generated_script, path_generated + "/script.cmd");

      addNote("Created script.cmd with the following content:\n" + StaticHelper::readFile(path_generated + "/script.cmd") + "<EOF>");

      static const std::string SPEC_BEGIN{ "--SPEC-STUFF" };
      static const std::string SPEC_END{ "--EO-SPEC-STUFF" };
      static const std::string ADDONS_BEGIN{ "--ADDONS" };
      static const std::string ADDONS_END{ "--EO-ADDONS" };
      mc_scene->data_->addStringToDataPack(mc_scene->getTemplateDir(), macro::MY_PATH_VARNAME); // Set the script processors home path (for the case it's not already been set during EnvModel generation).

      // Re-generate SPEC stuff.
      main_smv = StaticHelper::removeMultiLineComments(main_smv, SPEC_BEGIN, SPEC_END);
      main_smv += SPEC_BEGIN + "\n";

      auto spec_part = StaticHelper::removePartsOutsideOf(main_template, SPEC_BEGIN, SPEC_END);
      mc_scene->data_->addStringToDataPack(path_generated, "FULL_GEN_PATH");
      spec_part = vfm::macro::Script::processScript(spec_part, macro::Script::DataPreparation::both, mc_scene->data_, mc_scene->parser_);
      main_smv += spec_part + "\n";
      main_smv += SPEC_END + "\n";

      StaticHelper::writeTextToFile(main_smv, path_generated + "/main.smv");
   }

   test::convenienceArtifactRunHardcoded(test::MCExecutionType::mc, path_generated, "FAKE_PATH_NOT_USED", path_template, "FAKE_PATH_NOT_USED", mc_scene->getCachedDir(), mc_scene->path_to_external_folder_);
   mc_scene->generatePreview(path_generated, 0);

   mc_scene->addNote("Model checker run finished for folder '" + path_generated + "'.");
}

void McWorkflow::deleteMCOutputFromFolder(const std::string& path_generated, const bool actually_delete_gif)
{
   //StaticHelper::removeFileSafe(path_generated + "/" + "main.smv"); // Do NOT remove main.smv. It only gets changed at this point.


   std::string path_result{ path_generated + "/debug_trace_array.txt" };
   std::string path_runtimes{ path_generated + "/mc_runtimes.txt" };
   std::string path_script{ path_generated + "/" + "script.cmd" };

   std::string path_prose_description{ path_generated + "/" + PROSE_DESC_NAME };
   std::string path_planner_smv{ path_generated + "/planner.cpp_combined.k2.smv" };
   std::string path_generated_preview{ path_generated + "/preview" };

   if (actually_delete_gif) {
      for (int i = 0; i < 100; i++) StaticHelper::removeAllFilesSafe(path_generated + "/" + std::to_string(i));
      addNote("Deleting folder '" + path_generated_preview + "'.");
      StaticHelper::removeAllFilesSafe(path_generated_preview);
   }
   else {
      addNote("Overwriting folder '" + path_generated_preview + "'.");
      copyWaitingForPreviewGIF();
   }

   if (std::filesystem::exists(path_prose_description)) {
      addNote("Deleting file '" + path_prose_description + "'.");
      StaticHelper::removeFileSafe(path_prose_description);
   }

   if (std::filesystem::exists(path_planner_smv)) {
      addNote("Deleting file '" + path_planner_smv + "'.");
      StaticHelper::removeFileSafe(path_planner_smv);
   }

   if (std::filesystem::exists(path_result)) {
      addNote("Deleting file '" + path_result + "'.");
      StaticHelper::removeFileSafe(path_result);
   }

   if (std::filesystem::exists(path_runtimes)) {
      addNote("Deleting file '" + path_runtimes + "'.");
      StaticHelper::removeFileSafe(path_runtimes);
   }

   if (std::filesystem::exists(path_script)) {
      addNote("Deleting file '" + path_script + "'.");
      StaticHelper::removeFileSafe(path_script);
   }

   for (auto& sec : se_controllers_) {
      sec.selected_ = false;
   }
}

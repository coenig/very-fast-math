//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2024 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "testing/interactive_testing.h"

#include <stdio.h>
#include <string>
#include <filesystem>
#include <cstdlib>
#include <filesystem>

using namespace vfm;
using namespace test;



std::map<std::string, std::string> vfm::test::retrieveEnvModelDefinitionFromJSON(const std::string json_file, const EnvModelCachedMode cached_mode)
{
   std::map<std::string, std::string> env_model_definitions{};

   std::ifstream i{ json_file };
   nlohmann::json j{};

   if (!i.good()) {
      return {};
   }

   i >> j;

   for (auto& [key_config, value_config] : j.items()) {
      std::string env_model_definition{ cached_mode == EnvModelCachedMode::always_regenerate ? "G" : "" };

      for (auto& [key, value] : value_config.items()) {
         std::string val_str{ nlohmann::to_string(value) };
         bool is_string{ value.type() == nlohmann::detail::value_t::string };
         std::string processed_quotes{};

         if (is_string) {
            processed_quotes = StaticHelper::replaceAll(val_str, "\\\"", "\"");
            processed_quotes = processed_quotes.substr(1, processed_quotes.size() - 2);
         }

         env_model_definition += "," + key + "=" + (is_string ? STRING_PREFIX_ENVMODEL_GEN + StaticHelper::safeString(processed_quotes) : val_str);
      }

      env_model_definition = (!env_model_definition.empty() && env_model_definition[0] == ',')
         ? env_model_definition.substr(1)
         : env_model_definition;

      env_model_definitions.insert({ key_config, env_model_definition });
   }

   return env_model_definitions;
}

struct MCScene;

std::map<std::string, std::string> getRelevantVariablesFromEnvModel(
   const std::string& env_model, std::map<std::string, std::pair<std::string, std::string>>& default_values)
{
   std::map<std::string, std::string> res{};

   std::regex variableRegex1(R"(\bFound variable (\w+) with value (\S+) during generation of EnvModel)");
   std::smatch match1;
   std::string::const_iterator searchStart1(env_model.cbegin());

   while (std::regex_search(searchStart1, env_model.cend(), match1, variableRegex1)) {
      res[match1[1]] = match1[2];
      //res.insert(match1[1]);
      searchStart1 = match1.suffix().first;
   }

   std::regex variableRegex2(R"(\bUndeclared variable (\w+) found during generation of EnvModel\. Setting to default value (\S+))");
   std::smatch match2;
   std::string::const_iterator searchStart2(env_model.cbegin());

   while (std::regex_search(searchStart2, env_model.cend(), match2, variableRegex2)) {
      res[match2[1]] = match2[2];
      //res.insert(match2[2]);
      searchStart2 = match2.suffix().first;
   }

   std::string envmodel_copy{ env_model };
   std::regex pattern("-- Found variable (\\w+) with value (\\S+) during generation of EnvModel \\(default would be (\\S+)\\).");
   std::smatch matches;

   std::string::const_iterator searchStart(envmodel_copy.cbegin());
   while (std::regex_search(searchStart, envmodel_copy.cend(), matches, pattern)) {
      default_values[matches[1].str()] = { matches[2].str(), matches[3].str() };
      //std::cout << "Variable: " << matches[1] << std::endl;
      //std::cout << "Value: " << matches[2] << std::endl;
      //std::cout << "Default Value: " << matches[3] << std::endl;
      //std::cout << std::endl;

      searchStart = matches.suffix().first;
   }

   return res;
}

bool checkEqual(const std::string& val_desired, const std::string& val_cached)
{
   bool equals{ true };

   std::string actual_val_desired{ StaticHelper::toLowerCase(StaticHelper::trimAndReturn(val_desired)) };
   std::string actual_val_cached{ StaticHelper::toLowerCase(StaticHelper::trimAndReturn(val_cached)) };

   if (actual_val_desired == "true") actual_val_desired = "1";
   if (actual_val_desired == "false") actual_val_desired = "0";
   if (actual_val_cached == "true") actual_val_cached = "1";
   if (actual_val_cached == "false") actual_val_cached = "0";

   if (StaticHelper::isParsableAsFloat(actual_val_desired)) actual_val_desired = StaticHelper::floatToStringNoTrailingZeros(std::stof(actual_val_desired));
   if (StaticHelper::isParsableAsFloat(actual_val_cached)) actual_val_cached = StaticHelper::floatToStringNoTrailingZeros(std::stof(actual_val_cached));

   if (actual_val_desired != actual_val_cached) equals = false;

   return equals;
}

std::string retrievePathOfCachedEnvModel(const std::string& cached_dir, const std::string& desired_envmodel_definition)
{
   if (!cached_dir.empty() && std::filesystem::is_directory(cached_dir)) {
      auto split_desired{ StaticHelper::split(desired_envmodel_definition.substr(2), ",") };

      for (const auto& entry : std::filesystem::directory_iterator(cached_dir)) {
         if (std::filesystem::is_directory(entry)) {
            std::string path{ entry.path().string() };
            std::string envmodel_path{ path + "/EnvModel.smv" };

            if (StaticHelper::existsFileSafe(envmodel_path)) {
               // We need to check that:
               // (1) All variables defined in the new JSON have the same values as in the cached EnvModel.
               // (2) All variables that are NOT defined in the new JSON have default value in the cached EnvModel.

               bool equals{ true };
               std::map<std::string, std::pair<std::string, std::string>> overwritten_default_values{};
               auto config{ getRelevantVariablesFromEnvModel(StaticHelper::readFile(envmodel_path), overwritten_default_values) };

               for (const auto& varval_cached : config) {
                  std::set<std::string> all_desired_variables{};
                  const std::string var_cached{ varval_cached.first };
                  const std::string val_cached{ varval_cached.second };

                  // (1) Check if all defined new (desired) variables have the same value as the cached.
                  for (const auto& varval_desired_str : split_desired) {
                     const auto varval_desired{ StaticHelper::split(varval_desired_str, "=") };

                     if (varval_desired.size() >= 2) { // TODO: Only special case is "G" which is deprecated.
                        assert(varval_desired.size() == 2);
                        const std::string var_desired{ varval_desired[0] };
                        const std::string val_desired{ varval_desired[1] };
                        all_desired_variables.insert(var_desired);

                        if (var_desired == var_cached) {
                           equals = equals && checkEqual(val_desired, val_cached);
                           if (!equals) break;
                        }
                     }
                  }

                  // (2) Check if undefined new variables have default value in cached EnvModel.
                  if (!all_desired_variables.count(var_cached)) { // We check only the overwritten values in the cached EnvModel.
                     if (overwritten_default_values.count(var_cached)) { // The non-overwritten default values match the non-defined new values.
                        std::pair<std::string, std::string> value_and_default_value_cached{ overwritten_default_values.at(var_cached) };
                        assert(val_cached == value_and_default_value_cached.first);
                        equals = equals && checkEqual(value_and_default_value_cached.first, value_and_default_value_cached.second);
                     }
                  }

                  if (!equals) break;
               }

               if (equals) return path;
            }
         }
      }
   }

   return ""; // No cached env model found.
}

std::string getFirstFreeCachedDirPath(const std::string& cached_dir)
{
   static const std::string CACHED_BASE_NAME{ "cached_" };

   for (int i = 0;; i++) {
      std::string name{ cached_dir + "/" + CACHED_BASE_NAME + std::to_string(i) };

      if (!StaticHelper::existsFileSafe(name)) {
         return name;
      }
   }
}

void checkForChangedEnvModelTemplateOrCreateOneIfMissing(const std::string& cached_path_str, const std::string& envmodel_template_path_str, const std::string& gui_name)
{
   const std::filesystem::path cached_path{ cached_path_str };
   const std::filesystem::path envmodel_template_path{ envmodel_template_path_str };
   const std::filesystem::path cached_envmodel_template_path{ cached_path / ENVMODEL_TEMPLATE_FILENAME };

   if (!StaticHelper::existsFileSafe(cached_path)) { // Create cached dir if missing.
      Failable::getSingleton(gui_name)->addNote("Cache not found, creating one in '" + cached_path.string() + "'.");
      StaticHelper::createDirectoriesSafe(cached_path, false);
   }

   if (!StaticHelper::existsFileSafe(cached_envmodel_template_path)) { // Copy EnvModel.tpl if missing.
      Failable::getSingleton(gui_name)->addNote("Cached '" + cached_envmodel_template_path.string() + "' file not found, copying the one from '" + envmodel_template_path.string() + "'.");

      try {
         std::filesystem::copy(envmodel_template_path, cached_envmodel_template_path);
      }
      catch (const std::exception& e) {
         Failable::getSingleton()->addWarning("Could not copy from '" + envmodel_template_path.string() + "' to '" + cached_envmodel_template_path.string() + "'.");
      }
   }

   // Assuming Cached EnvModel.tpl exists now.

   // Check if old and new template are equal.
   try { // This is for the read file stuff
      std::string envmodel_template{ StaticHelper::readFile(envmodel_template_path.string()) };
      std::string cached_envmodel_template{ StaticHelper::readFile(cached_envmodel_template_path.string()) };

      if (envmodel_template != cached_envmodel_template) { // If not, delete complete cached folder and call recursively to re-generate cache structure.
         Failable::getSingleton(gui_name)->addNote("Cached file '" + cached_envmodel_template_path.string() + "' is different from current one in '" + envmodel_template_path.string() + "'. I will delete the cached folder.");
         StaticHelper::removeAllFilesSafe(cached_path);
         checkForChangedEnvModelTemplateOrCreateOneIfMissing(cached_path_str, envmodel_template_path_str, gui_name);
      }
   }
   catch (const std::exception& e) {
      Failable::getSingleton()->addWarning("Could not compare EnvModels from '" + envmodel_template_path.string() + "' and '" + cached_envmodel_template_path.string() + "'.");
   }
}

void cleanUpCache(const std::string& gui_name, const std::string& cache_dir)
{ // Removes broken cache folders (particularly those that have been aborted before termination).
   Failable::getSingleton(gui_name)->addNote("Looking for broken cache entries...");

   const std::vector<std::string> required_files{ "EnvModel.smv", "main.smv", "planner.cpp_combined.k2", "script.cmd" };
   int cnt{ 0 };
   int broken{ 0 };

   if (StaticHelper::existsFileSafe(cache_dir)) {
      for (const auto& folder : std::filesystem::directory_iterator(cache_dir)) {
         if (std::filesystem::is_directory(folder)) {
            bool fine{ true };

            for (const auto& filename : required_files) {
               fine = fine && StaticHelper::existsFileSafe(folder / std::filesystem::path(filename));
            }

            if (fine) {
               Failable::getSingleton()->addNote("Cache folder '" + StaticHelper::absPath(folder.path().string()) + "' is fine.");
            }
            else {
               Failable::getSingleton(gui_name)->addNote("Found broken cache folder '" + StaticHelper::absPath(folder.path().string()) + "'. I will delete it.");
               StaticHelper::removeAllFilesSafe(folder, false);
               broken++;
            }

            cnt++;
         }
      }
   }

   const std::string summary{ broken > 0 ? std::to_string(broken) + "/" + std::to_string(cnt) + " were broken and had to be deleted" : "All " + std::to_string(cnt) + " were fine"};

   Failable::getSingleton(gui_name)->addNote("Finished looking for broken cache entries. (" + summary + ".)");
}

std::string vfm::test::doParsingRun(
   const std::pair<std::string, std::string>& envmodel_definition,
   const std::string& root_dir,
   const std::string& envmodel_tpl,
   const std::string& planner_path,
   const std::string& target_dir,
   const std::string& cached_dir,
   const std::string& gui_name
)
{
   const bool HAS_ENVMODEL{ !envmodel_definition.first.empty() || !envmodel_definition.second.empty() };
   cleanUpCache(gui_name, cached_dir);

   std::string target_dir_without_trailing_slashes{ target_dir };

   while (!target_dir_without_trailing_slashes.empty() && target_dir_without_trailing_slashes.back() == '/') {
      target_dir_without_trailing_slashes.pop_back();
   }

   std::string full_target_path{ root_dir + "/" + target_dir_without_trailing_slashes + envmodel_definition.first + "/" };
   Failable::getSingleton(gui_name)->addNote("Generating EnvModel based on config '" + envmodel_definition.first + (envmodel_definition.first.empty() ? "<noname>" : "") + "'.");

   StaticHelper::createDirectoriesSafe(full_target_path, false);

   CppParser::TARGET_PATH_FOR_MORTY_GUY_PROGRESS = full_target_path;

   std::string env_model_description{
      NATIVE_SMV_ENV_MODEL_DENOTER_OPEN
      + envmodel_definition.second
      + NATIVE_SMV_ENV_MODEL_DENOTER_CLOSE
      + root_dir
      + "/"
      + envmodel_tpl
   };

   std::string includes_file_path{
      root_dir
      + "/"
      + planner_path
   };

   checkForChangedEnvModelTemplateOrCreateOneIfMissing(cached_dir, envmodel_tpl, gui_name);

   std::string generated_dir{ cached_dir.empty() ? full_target_path : getFirstFreeCachedDirPath(cached_dir) };
   std::string generated_file{ generated_dir + "/planner.cpp"};
   std::string existing_path{ retrievePathOfCachedEnvModel(cached_dir, env_model_description) };

   if (existing_path.empty()) {
      StaticHelper::createDirectoriesSafe(generated_dir, false);

      bool success{ performFSMCodeGeneration(
         includes_file_path.c_str(),
         HAS_ENVMODEL ? env_model_description.c_str() : "",
         generated_file.c_str(),
         // TODO: This needs to be input from actual command line.
         "\
// #vfm-option[[ target_mc << kratos ]] \
// #vfm-option[[ optimization_mode << inner_only ]] \
// #vfm-option[[ general_mode << debug ]] \
// #vfm-option[[ create_additional_files << all ]]" // Command line input.
      ) };

      existing_path = StaticHelper::removeLastFileExtension(generated_file, "/");
   }
   else {
      Failable::getSingleton(gui_name)->addNote("Using cached EnvModel found in '" + existing_path + "'.");
   }

   if (!cached_dir.empty()) {
      auto dir_source{ StaticHelper::absPath(existing_path) };
      auto dir_target{ StaticHelper::absPath(full_target_path) };

      if (!StaticHelper::existsFileSafe(dir_source)) Failable::getSingleton(gui_name)->addError("Directory does not exist: '" + dir_source + "'.");
      if (!StaticHelper::existsFileSafe(dir_target)) Failable::getSingleton(gui_name)->addError("Directory does not exist: '" + dir_target + "'.");

      try {
         std::filesystem::copy(dir_source, dir_target, std::filesystem::copy_options::recursive | std::filesystem::copy_options::overwrite_existing);
      }
      catch (const std::exception& e) {
         Failable::getSingleton()->addWarning("Could not copy from '" + dir_source + "' to '" + dir_target + "'.");
      }
   }

   return full_target_path;
}

void vfm::test::termnate() {
   std::cout << std::endl << "<TERMINATED>" << std::endl;

   #ifdef _WIN32
      std::cin.get(); // Prevent Windows from closing the terminal immediately after termination.
   #endif

   std::exit(0);
}

void vfm::test::runInterpreter()
{
   std::string s = "println(\"---\nw-vfm-i, your werycool-vfm-interpreter.\n---\n\")";
   //s = "0";
   auto parser = SingletonFormulaParser::getInstance();
   auto data = std::make_shared<DataPack>();
   auto rules = std::make_shared<std::map<std::string, std::map<int, std::set<MetaRule>>>>(parser->getAllRelevantSimplificationRulesOrMore());
   bool simplify_runtime = false;
   bool simplify_hardcoded = false;
   bool print_simplification_trace = true;

   StaticHelper::addNDetDummyFunctions(parser, data);

   while (s != "#exit") {
      if (s == "#sr" || s == "#s") {
         simplify_runtime = !simplify_runtime;
         std::cout << "simplify_runtime set to: " << simplify_runtime << " (" << HIGHLIGHT_COLOR << MetaRule::countMetaRules(*rules) << " rules" << RESET_COLOR << ")" << std::endl;

         if (s != "#s") {
            std::cout << std::endl;
            std::getline(std::cin, s);
            continue;
         }
      }

      if (s == "#sh" || s == "#s") {
         simplify_hardcoded = !simplify_hardcoded;
         std::cout << "simplify_hardcoded set to: " << simplify_hardcoded << std::endl << std::endl;
         std::getline(std::cin, s);
         continue;
      }

      if (s == "#c") {
         std::cout << "Clearing all " << HIGHLIGHT_COLOR << MetaRule::countMetaRules(*rules) << RESET_COLOR << " runtime simplification rules." << std::endl << std::endl;
         rules->clear();
         std::getline(std::cin, s);
         continue;
      }

      if (s == "#l") {
         for (const auto& rules_per_stage : MetaRule::getMetaRulesByStage(*rules)) {
            std::cout << std::endl <<"STAGE " << std::to_string(rules_per_stage.first) << std::endl;
            std::cout << "=========" << std::endl;
            for (const auto& rules_by_op : rules_per_stage.second) {
               for (const auto& rules_by_op_arg_num : rules_by_op.second) {
                  for (const auto& rule : rules_by_op_arg_num.second) {
                     std::cout << rule.serialize() << std::endl;
                  }
               }
            }
         }

         std::cout << "Overall " << HIGHLIGHT_COLOR << MetaRule::countMetaRules(*rules) << RESET_COLOR << " runtime simplification rules." << std::endl << std::endl;
         std::getline(std::cin, s);
         continue;
      }

      if (s == "#d") {
         print_simplification_trace = !print_simplification_trace;
         std::cout << "Debug mode set to " << HIGHLIGHT_COLOR << print_simplification_trace << RESET_COLOR << "." << std::endl << std::endl;
         std::getline(std::cin, s);
         continue;
      }

      if (s == "#r") {
         rules = std::make_shared<std::map<std::string, std::map<int, std::set<MetaRule>>>>(parser->getAllRelevantSimplificationRulesOrMore());
         std::cout << "Reset runtimes rules to " << HIGHLIGHT_COLOR << MetaRule::countMetaRules(*rules) << RESET_COLOR << " rules." << std::endl << std::endl;
         std::getline(std::cin, s);
         continue;
      }

      if (s == "#cls") {
         for (int i = 0; i < 10000; i++) {
            std::cout << std::endl;
         }

         std::getline(std::cin, s);
         continue;
      }

      if (s == "#data") {
         std::cout << data->toString() << std::endl;
         std::getline(std::cin, s);
         continue;
      }

      if (s == "#heap") {
         std::cout << data->toStringHeap() << std::endl;
         std::getline(std::cin, s);
         continue;
      }

      if (StaticHelper::stringContains(s, FROM_TO_SYMBOL)) {
         MetaRule m(s, 10); // Default stage to 10. Will be overriden by optional "$i" notation at the end of the rule string.
         m.convertVeriablesToMetas();
         std::string op_name{ m.getFrom()->getOptor() };
         int arg_num{ m.getFrom()->getOpStruct().arg_num };

         rules->insert({ op_name, {} });
         rules->at(op_name).insert({arg_num, {}});
         rules->at(op_name).at(arg_num).insert(m);
         std::cout << "Adding meta rule '" << HIGHLIGHT_COLOR << m.serialize() << RESET_COLOR << "' to runtime rule set (" + HIGHLIGHT_COLOR + "now " << MetaRule::countMetaRules(*rules) << " rules" + RESET_COLOR + ")." << std::endl << std::endl;
         std::cout << simplification::CodeGenerator::createCodeFromMetaRuleString(m, simplification::CodeGenerationMode::negative, "rule", false, "parser", parser) << std::endl << std::endl;
         std::getline(std::cin, s);
         continue;
      }

      auto fmla = MathStruct::parseMathStruct(s, parser, data)->toTermIfApplicable();
      std::cout << fmla->serializePlainOldVFMStyle() << "\t(parsed)" << std::endl;
      auto simplified_math_struct = _val(0);
      auto simplified_hard_coded = _val(1);
      //auto simplified_hard_coded_vf = _val(2);
      StaticHelper::createImageFromGraphvizDot(fmla->generateGraphvizWithFatherFollowing(0, true), "parsed");
      auto fmla2 = MathStruct::flattenFormula(fmla);
      std::cout << fmla2->serialize() << "\t(flattened)" << std::endl;

      if (simplify_runtime) {
         std::chrono::steady_clock::time_point begin_math_struct = std::chrono::steady_clock::now();
         simplified_math_struct = MathStruct::simplify(fmla->copy(), print_simplification_trace, nullptr, false, nullptr, rules); // Simp MathStruct
         std::chrono::steady_clock::time_point end_math_struct = std::chrono::steady_clock::now();
         //simplified_math_struct->checkIfAllChildrenHaveConsistentFather();
         //simplified_math_struct->setPrintFullCorrectBrackets(true);
         std::cout << simplified_math_struct->serializePlainOldVFMStyle() << "\t(simplified_runtime: " << std::chrono::duration_cast<std::chrono::microseconds>(end_math_struct - begin_math_struct).count() << " micros)" << std::endl;
         StaticHelper::createImageFromGraphvizDot(simplified_math_struct->generateGraphvizWithFatherFollowing(0, false), "simplified_runtime");
      }
      else {
         StaticHelper::createImageFromGraphvizDot("digraph G {NONE [shape=diamond,color=red,style=filled]}", "simplified_runtime");
      }

      if (simplify_hardcoded) {
         std::chrono::steady_clock::time_point begin_hard_coded = std::chrono::steady_clock::now();
         simplified_hard_coded = simplification::simplifyFast(fmla->copy());                 // Simp HardCoded
         std::chrono::steady_clock::time_point end_hard_coded = std::chrono::steady_clock::now();
         //std::chrono::steady_clock::time_point begin_hard_coded_vf = std::chrono::steady_clock::now();
         //simplified_hard_coded_vf = simplification::simplifyVeryFast(fmla->copy());                 // Simp HardCoded
         //std::chrono::steady_clock::time_point end_hard_coded_vf = std::chrono::steady_clock::now();
         simplified_hard_coded->checkIfAllChildrenHaveConsistentFatherQuiet();
         //simplified_hard_coded_vf->checkIfAllChildrenHaveConsistentFather();
         //simplified_hard_coded->setPrintFullCorrectBrackets(true);
         std::cout << simplified_hard_coded->serializePlainOldVFMStyle() << "\t(simplified_hard_coded:   " << std::chrono::duration_cast<std::chrono::microseconds>(end_hard_coded - begin_hard_coded).count() << " micros)" << std::endl;
         StaticHelper::createImageFromGraphvizDot(simplified_hard_coded->generateGraphvizWithFatherFollowing(0, false), "simplified_hardcoded");
         //std::cout << simplified_hard_coded_vf->serializePlainOldVFMStyle() << "\t(simplified_very_fast:   " << std::chrono::duration_cast<std::chrono::microseconds>(end_hard_coded_vf - begin_hard_coded_vf).count() << " micros)" << std::endl << std::endl;
         //StaticHelper::createImageFromGraphvizDot(simplified_hard_coded_vf->generateGraphvizWithFatherFollowing(0, false), "simplified_hardcoded_vf");
      }
      else {
         StaticHelper::createImageFromGraphvizDot("digraph G {NONE [shape=diamond,color=red,style=filled]}", "simplified_hardcoded");
      }

      if (simplify_runtime && simplify_hardcoded &&
         !simplified_math_struct->isStructurallyEqual(simplified_hard_coded)
            //|| !simplified_math_struct->isStructurallyEqual(simplified_hard_coded_vf))
         ) {
         Failable::getSingleton()->addError("Simplified formulas differ.");
      }

      std::cout << fmla->eval(data, parser) << std::endl << std::endl;
      std::getline(std::cin, s);
   }
}

void vfm::test::loopKratos()
{
   std::string file_content{};

   for (;;) {
      std::ifstream t("../examples/cpp_parsing/kratos/test_code.c");
      std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());

      if (file_content != str) {
         performFSMCodeGeneration(
            "../examples/cpp_parsing/kratos/vfm-includes-kratos.txt",
            //(NATIVE_SMV_ENV_MODEL_DENOTER_OPEN + "3G../examples/env_model_devel/generator/EnvModel_ICars_WithViper.tpl").c_str(),
            "",
            "../examples/cpp_parsing/generated_kratos/kratos.cpp",
            "" // Command line input.
         );
      }

      file_content = str;
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
   }
}

// Instantiation of the command line parser for running the MC workflow.
// Required inputs.
const std::string CMD_MODE{ "--mode" };

const std::string MODE_PARSER{ "parser" };
const std::string MODE_MC{ "mc" };
const std::string MODE_CEX{ "cex" };

// Optional inputs (with default values).
const std::string CMD_DIR_ROOT{ "--rootdir" };
const std::string CMD_DIR_TARGET{ "--targetdir" };
const std::string CMD_ENV_MODEL_JSON_FILE{ "--env-model-json-file" };
const std::string CMD_ENV_MODEL_TEMPLATE{ "--env-model-template-file" };
const std::string CMD_PLANNER_FILENAME{ "--planner-include-file" };
const std::string CMD_MAIN_TEMPLATE{ "--main-template-file" };
const std::string CMD_CACHE_DIR{ "--chache-dir" };
const std::string CMD_CEX_FILE{ "--cex-file-name" };
const std::string CMD_NUXMV_EXEC{ "--path-to-nuxmv" };
const std::string CMD_KRATOS_EXEC{ "--path-to-kratos" };

#ifdef _WIN32
const std::string CMD_CPP_EXEC{ "--path-to-cpp" };
#endif

const std::string CMD_HELP{ "--help" };

InputParser createInputParserForMC(int& argc, char** argv)
{
   InputParser parser{ argc, argv };

   parser.addParameter(CMD_DIR_ROOT, "directory to read input files from");

   parser.addParameter(CMD_DIR_ROOT, "directory to read input files from", ".");
   parser.addParameter(CMD_DIR_TARGET, "directory for generated files, realtive to '" + CMD_DIR_ROOT + "'", "generated");
   parser.addParameter(CMD_ENV_MODEL_JSON_FILE, "config json for environment model definition, relative to '" + CMD_DIR_ROOT + "'", "envmodel_config.json");
   parser.addParameter(CMD_ENV_MODEL_TEMPLATE, "path to the env model template file, relative to '" + CMD_DIR_ROOT + "'", "EnvModel.tpl");
   parser.addParameter(CMD_PLANNER_FILENAME, "path to the planner include file, relative to '" + CMD_DIR_ROOT + "'", "vfm-includes-planner.txt");
   parser.addParameter(CMD_MAIN_TEMPLATE, "path to the main template file, relative to '" + CMD_DIR_ROOT + "'", "main.tpl");
   parser.addParameter(CMD_CACHE_DIR, "path to the cache directory, relative to '" + CMD_DIR_ROOT + "'", "./tmp");
   parser.addParameter(CMD_CEX_FILE, "name of counterexample file in SMV format", "debug_trace_array.txt");
   parser.addParameter(CMD_NUXMV_EXEC, "path to NUXMV executable", "nuxmv.exe");
   parser.addParameter(CMD_KRATOS_EXEC, "path to KRATOS executable", "kratos.exe");
#ifdef _WIN32
   parser.addParameter(CMD_CPP_EXEC, "path to CPP preprocessor executable", "cpp.exe");
#endif
   parser.addFlag(CMD_HELP, "shows parameter values and help");

   parser.addParameter(CMD_MODE, "general run mode, '" + MODE_PARSER + "' or '" + MODE_MC + "' or '" + MODE_CEX + "'");

   return parser;
}
// EO Instantiation of the command line parser for running the MC workflow.

int vfm::test::artifactRun(int argc, char* argv[])
{
   InputParser inputs{ createInputParserForMC(argc, argv) };
   bool success{ true };

   if (inputs.getCmdOption(CMD_HELP) != "true") inputs.triggerErrorifAnyArgumentMissing();

   if (inputs.getCmdOption(CMD_HELP) == "true" || inputs.hasErrorOccurred()) {
      inputs.printArgumentsForMC();
      return EXIT_FAILURE;
   }
   
   std::string generated_dir{ inputs.getCmdOption(CMD_DIR_ROOT) + "/" + inputs.getCmdOption(CMD_DIR_TARGET) + "/" };

   if (inputs.getCmdMultiOption(CMD_MODE).count(MODE_PARSER)) // *** Parsing and EnvModel generation ***
   {
      StaticHelper::createDirectoriesSafe(inputs.getCmdOption(CMD_DIR_TARGET), false);

      auto env_model_definitions{ retrieveEnvModelDefinitionFromJSON(inputs.getCmdOption(CMD_ENV_MODEL_JSON_FILE), EnvModelCachedMode::use_cached_if_available) };

      //if (env_model_definitions.size() > 1) { // TODO: This error message makes sense, but needs to be done differently, because mode "all" will also yield "parser" here.
      //   Failable::getSingleton()->addFatalError("More than one (" + std::to_string(env_model_definitions.size()) + ") env model configs found, but mode is '" + MODE_PARSER + "'. Only 'parser' is allowed for generation of multiple configurations.");
      //}

      std::vector<std::string> generated_paths{};
      std::map<EnvModelConfig, std::string> env_model_configs{};
      std::set<std::string> relevant_variables{};

      if (env_model_definitions.empty()) {
         env_model_definitions.insert({ "", "" });
      }

      for (const auto& envmodel_definition : env_model_definitions) {
         std::string actual_generated_path{ doParsingRun(
            envmodel_definition,
            inputs.getCmdOption(CMD_DIR_ROOT),
            inputs.getCmdOption(CMD_ENV_MODEL_TEMPLATE),
            inputs.getCmdOption(CMD_PLANNER_FILENAME),
            inputs.getCmdOption(CMD_DIR_TARGET),
            inputs.getCmdOption(CMD_CACHE_DIR), // This is the cached path.
            "no-gui")};

         generated_paths.push_back(actual_generated_path);
         success = success && !actual_generated_path.empty();
      }

      std::string paths{};
      for (const auto& path : generated_paths) {
         paths += path + "\n";
      }
      
      StaticHelper::createDirectoriesSafe(inputs.getCmdOption(CMD_DIR_TARGET), false);
      StaticHelper::writeTextToFile(paths, inputs.getCmdOption(CMD_DIR_TARGET) + "generated_paths.txt");
   }

   if (success && inputs.getCmdMultiOption(CMD_MODE).count(MODE_MC)) { // *** Model Checking ***
      std::string kratos_fulldir{ inputs.getCmdOption(CMD_DIR_ROOT) + "/" + inputs.getCmdOption(CMD_KRATOS_EXEC) };
      std::string nuxmv_fulldir{ inputs.getCmdOption(CMD_DIR_ROOT) + "/" + inputs.getCmdOption(CMD_NUXMV_EXEC) };

#ifdef _WIN32
      std::string cpp_fulldir{ inputs.getCmdOption(CMD_DIR_ROOT) + "/" + inputs.getCmdOption(CMD_CPP_EXEC) };
      kratos_fulldir = StaticHelper::replaceAll(kratos_fulldir, "/", "\\");
      cpp_fulldir = StaticHelper::replaceAll(cpp_fulldir, "/", "\\");
      nuxmv_fulldir = StaticHelper::replaceAll(nuxmv_fulldir, "/", "\\");
      //std::string command_cpp{ "set CPP=" + cpp_fulldir };
#endif
      std::filesystem::path kratos_exec{ kratos_fulldir };
      std::filesystem::path nuxmv_exec{ nuxmv_fulldir };

      try {
         std::filesystem::permissions(kratos_exec, std::filesystem::perms::owner_all);
         std::filesystem::permissions(nuxmv_exec, std::filesystem::perms::owner_all);
      }
      catch (std::exception& e) {
         Failable::getSingleton()->addError("Could not set permissions for '" + nuxmv_fulldir + "' or '" + kratos_fulldir + "'.");
      }

      std::string command_kratos{ kratos_fulldir
         + " -trans_output_format=nuxmv-module -trans_enum_mode=symbolic -output_file=\"" 
         + generated_dir + "planner.cpp_combined.k2.smv\""
         + " \"" + generated_dir + "planner.cpp_combined.k2\""};
      std::string command_nuxmv{ nuxmv_fulldir
         + " -int -pre cpp -source " + generated_dir + "script.cmd " + generated_dir + "main.smv"};

      ScaleDescription::createTimescalingFile(generated_dir);

      inputs.addNote("RUNNING KRATOS with command '" + command_kratos + "'.");
      int success_code_kratos{};
      int success_code_nuxmv{};
      StaticHelper::execWithSuccessCode(command_kratos, true, success_code_kratos, std::make_shared<std::string>(generated_dir + "/mc_runtimes.txt"));

#ifdef _WIN32
      inputs.addNote("SETTING CPP PATH TO '" + cpp_fulldir + "'");
      _putenv(("CPP=" + cpp_fulldir + "").c_str());
#endif

      inputs.addNote("RUNNING NUXMV with command '" + command_nuxmv + "'.");
      std::string result{ StaticHelper::execWithSuccessCode(command_nuxmv, true, success_code_nuxmv, std::make_shared<std::string>(generated_dir + "/mc_runtimes.txt")) };
      StaticHelper::writeTextToFile(result, generated_dir + "debug_trace_array.txt");

      success == success && (success_code_kratos == EXIT_SUCCESS) && (success_code_nuxmv == EXIT_SUCCESS);
   }

   if (success && inputs.getCmdMultiOption(CMD_MODE).count(MODE_CEX)) // *** Explainability toolchain ***
   {
      inputs.addNote("Executing explainability toolchain on '" + inputs.getCmdOption(CMD_CEX_FILE) + "' in '" + generated_dir + "'.");
      success = success && mc::trajectory_generator::VisualizationLaunchers::quickGenerateGIFs(
         generated_dir,
         StaticHelper::removeLastFileExtension(inputs.getCmdOption(CMD_CEX_FILE)),
         mc::trajectory_generator::CexTypeEnum::smv, // TODO: Make this parametrizable
         true,
         true);
   }

   return success && !inputs.hasErrorOccurred()
      ? EXIT_SUCCESS
      : EXIT_FAILURE;
}

// The calls in the strings of this function can be directly used to run the respective functionality 
// from terminal on win or linux (from bin folder), here the calls are wrapped for convenience.
void vfm::test::convenienceArtifactRunHardcoded(
   const MCExecutionType exec,
   const std::string& target_directory,
   const std::string& json_config_path,
   const std::string& env_model_template_path,
   const std::string& vfm_includes_file_path,
   const std::string& cache_dir,
   const std::string& path_to_external_folder)
{
   std::vector<std::string> executions{};

#ifdef _WIN32
   if (exec == MCExecutionType::all || exec == MCExecutionType::parser_and_mc || exec == MCExecutionType::parser) {
      executions.push_back("./vfm.exe --mode parser --env-model-json-file " + json_config_path + " --env-model-template-file " + env_model_template_path + " --planner-include-file " + vfm_includes_file_path + " --targetdir " + target_directory + "/" + " --chache-dir " + cache_dir);
   }

   if (exec == MCExecutionType::all || exec == MCExecutionType::parser_and_mc || exec == MCExecutionType::mc_and_cex || exec == MCExecutionType::mc) {
      executions.push_back("./vfm.exe --mode mc --targetdir " + target_directory + " --path-to-kratos " + path_to_external_folder + "/win32/kratos.exe --path-to-cpp " + path_to_external_folder + "/win32/cpp.exe --path-to-nuxmv " + path_to_external_folder + "/win32/nuXmv.exe");
   }

   if (exec == MCExecutionType::all || exec == MCExecutionType::mc_and_cex || exec == MCExecutionType::cex) {
      executions.push_back("./vfm.exe --mode cex --targetdir " + target_directory + " --rootdir .");
   }
#elif __linux__ 
   if (exec == MCExecutionType::all || exec == MCExecutionType::parser_and_mc || exec == MCExecutionType::parser) {
      executions.push_back("./vfm --mode parser --env-model-json-file " + json_config_path + " --env-model-template-file " + env_model_template_path + " --planner-include-file " + vfm_includes_file_path + " --targetdir " + target_directory + "/" + " --chache-dir " + cache_dir);
   }

   if (exec == MCExecutionType::all || exec == MCExecutionType::parser_and_mc || exec == MCExecutionType::mc_and_cex || exec == MCExecutionType::mc) {
      executions.push_back("./vfm --mode mc --targetdir " + target_directory + " --path-to-kratos " + path_to_external_folder + "/linux64/kratos --path-to-nuxmv " + path_to_external_folder + "/linux64/nuXmv");
   }

   if (exec == MCExecutionType::all || exec == MCExecutionType::mc_and_cex || exec == MCExecutionType::cex) {
      executions.push_back("./vfm --mode cex --targetdir " + target_directory + " --rootdir .");
   }
#endif

   for (const auto& execution : executions) StaticHelper::fakeCallWithCommandLineArguments(execution, artifactRun);
}

void vfm::test::cameraRotationTester()
{
   auto trans{ std::make_shared<Plain3DTranslator>(false) };
   auto perspective{ trans->getPerspective() };
   constexpr float PI{ 3.14159265359 };
   StraightRoadSection lanes{ 3, {} };
   CarPars ego{ 1, 0, 0, HighwayImage::EGO_MOCK_ID };
   CarParsVec others{ { 2, 100, 100, 0 }, { 0, 150, 150, 1 }, { 1, 130, 130, 2 } };
   lanes.addLaneSegment(LaneSegment(-50, 0, 4));
   lanes.addLaneSegment(LaneSegment(0, 2, 2));
   lanes.addLaneSegment(LaneSegment(50, 0, 4));
   lanes.addLaneSegment(LaneSegment(150, 0, 4));

   // [(x=1.899996, y=0.000000, z=-660.000000), <rot_x=0.000000, rot_y=6.200000, rot_z=1.570796>] <disp_x=0.000000, disp_y=0.000000, disp_z=27.000000>]
   perspective->setCameraX(10);
   perspective->setCameraY(0);
   perspective->setCameraZ(-30);
   perspective->setCameraRotationX(0.3);
   perspective->setCameraRotationY(0);
   perspective->setCameraRotationZ(PI / 2.0);
   perspective->setDisplayWindowX(0);
   perspective->setDisplayWindowY(-8);
   perspective->setDisplayWindowZ(15);
   float one{ 1 };

   for (float i = 0; i > -300; i += 1) {
      HighwayImage img_new{ 1000, 1000, trans, 3 };
      img_new.startOrKeepUpPDF();
      lanes.setEgo(std::make_shared<CarPars>(ego.car_lane_, ego.car_rel_pos_, ego.car_velocity_, HighwayImage::EGO_MOCK_ID));
      lanes.setOthers(others);
      lanes.setFuturePositionsOfOthers({});

      img_new.paintStraightRoadScene(lanes, true);
      img_new.store("test_new.png");
      img_new.store("test_new.pdf", OutputType::pdf);

      std::string line;
      std::getline(std::cin, line);
      bool plus{ line[0] != '-' };
      std::string item{ plus ? line : line.substr(1) };

      if (item == "M") one *= 10;
      else if (item == "m") one *= 0.1;
      else if (item == "x")  perspective->setCameraX(perspective->getCameraX() + (plus ? one : -one));
      else if (item == "y")  perspective->setCameraY(perspective->getCameraY() + (plus ? one : -one));
      else if (item == "z")  perspective->setCameraZ(perspective->getCameraZ() + (plus ? one : -one));
      else if (item == "cx") perspective->setCameraRotationX(perspective->getRotationCameraX() + (plus ? one : -one));
      else if (item == "cy") perspective->setCameraRotationY(perspective->getRotationCameraY() + (plus ? one : -one));
      else if (item == "cz") perspective->setCameraRotationZ(perspective->getRotationCameraZ() + (plus ? one : -one));
      else if (item == "dx") perspective->setDisplayWindowX(perspective->getDisplayWindowX() + (plus ? one : -one));
      else if (item == "dy") perspective->setDisplayWindowY(perspective->getDisplayWindowY() + (plus ? one : -one));
      else if (item == "dz") perspective->setDisplayWindowZ(perspective->getDisplayWindowZ() + (plus ? one : -one));

      std::cout << perspective->serialize() << std::endl;
      std::cout << one << std::endl;

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
   }
}

void vfm::test::runMCExperiments(const MCExecutionType type_raw)
{
   std::string tardir{ "../examples/gp" };
   auto type{ type_raw };

   if (type == MCExecutionType::parser || type == MCExecutionType::parser_and_mc || type == MCExecutionType::all) {
      convenienceArtifactRunHardcoded(MCExecutionType::parser, tardir,
         "../src/templates/envmodel_config.json",
         "../src/templates/EnvModel.tpl",
         "../src/examples/bp/prototype/motion_planning/rule_based_planning/vfm-includes-viper.txt",
         "./tmp");
   }

   if (type == MCExecutionType::parser_and_mc) type = MCExecutionType::mc;
   if (type == MCExecutionType::all) type = MCExecutionType::mc_and_cex;

   if (type == MCExecutionType::mc || type == MCExecutionType::mc_and_cex || type == MCExecutionType::cex) {
      { // TODO: Additional scope not needed, but I'll keep it for now for clarification.
         ThreadPool pool(test::MAX_THREADS);

         for (const auto& gendir : StaticHelper::split(StaticHelper::readFile(tardir + "/generated_paths.txt"), "\n")) {
            if (!StaticHelper::isEmptyExceptWhiteSpaces(gendir)) {
               pool.enqueue([&type, &gendir] {
                  convenienceArtifactRunHardcoded(
                     type, 
                     gendir, 
                     "../src/templates/envmodel_config.json", 
                     "../src/templates/EnvModel.tpl",
                     "../src/examples/bp/prototype/motion_planning/rule_based_planning/vfm-includes-viper.txt",
                     "./tmp");
                  });
            }
         }
      }
   }
}

void vfm::test::extractInitValuesAca4_1()
{
   const std::string path = "../src/examples/aca4/prm/internal/";
   std::set<std::string> files{
      "Acu",
      "Asm",
      "BaseLci",
      "Drc",
      "Fct",
      "Fip",
      "Fsm",
      "Hmi",
      "Lci",
      "Mon",
      "Sim",
      "Spp" };

   auto inits_str = StaticHelper::split(StaticHelper::readFile(path + "init_values.txt"), "\n");
   std::map<std::string, std::string> inits{};

   for (const auto& str : inits_str) {
      if (!str.empty()) {
         auto pair = StaticHelper::split(str, "=");
         inits.insert({ pair[0], pair[1] });
      }
   }

   for (const auto& file : files) {
      auto file_content = StaticHelper::readFile(path + "prm_fctcf_cal_" + file + "_data.hpp");
      std::string new_content{};

      for (const auto& line : StaticHelper::split(file_content, "\n")) {
         size_t index = line.find(" PRM_");

         if (index == std::string::npos) {
            auto vec = StaticHelper::split(line, " ");
            std::string to_find{ "PRM_fctcf_CAL_" + file + "_" + StaticHelper::replaceAll(StaticHelper::replaceAll(vec.at(vec.size() - 1), ";", ""), "m_", "") + "_INIT_VAR_defaults" };

            if (inits.count(to_find)) {
               std::string value = inits.at(to_find);
               new_content += StaticHelper::replaceAll(line, ";", "") + " = " + value + ";\n";
            }
            else {
               new_content += line + "\n";
            }
         }
         else {
            std::string new_line{ line.substr(0, index) };
            auto vec = StaticHelper::split(new_line, " ");
            std::string to_find{ "PRM_fctcf_CAL_" + file + "_" + StaticHelper::replaceAll(vec.at(vec.size() - 1), "m_", "") + "_INIT_VAR_defaults" };

            if (inits.count(to_find)) {
               std::string value = inits.at(to_find);
               new_content += new_line + " = " + value + ";\n";
            }
            else {
               Failable::getSingleton()->addWarning("'" + to_find + "' not found.");
               new_content += line + "\n";
            }
         }
      }

      StaticHelper::writeTextToFile(new_content, path + "prm_fctcf_cal_" + file + "_data.hpp");
   }

   termnate();
}

void vfm::test::aca4_1Run()
{
   StaticHelper::removeAllFilesSafe(std::filesystem::path("./tmp"), false);
   //convenienceArtifactRunHardcoded(
   //   MCExecutionType::parser, 
   //   "../src/examples/aca4/generated",
   //   "../src/examples/aca4/envmodel_config.json",
   //   "../src/templates/EnvModel.tpl",
   //   "../src/examples/aca4/vfm-includes.txt");
   //std::cin.get();
   convenienceArtifactRunHardcoded(
      MCExecutionType::mc,
      "../src/examples/aca4/generated_config_aca",
      "../src/examples/aca4/envmodel_config.json",
      "../src/templates/EnvModel.tpl",
      "../src/examples/aca4/vfm-includes.txt");
   termnate();
}

static const Vec2D dim3D{ 500, 120 };

std::shared_ptr<RoadGraph> vfm::test::paintExampleRoadGraphStrangeJunction(
   const bool write_to_files, 
   const std::shared_ptr<RoadGraph> ego_section)
{
   assert(!ego_section || ego_section->isRootedInZeroAndUnturned());
   assert(!ego_section || ego_section->getAllNodes().size() == 1);
   assert(!ego_section || ego_section->getMyRoad().getEgo());

   // Strange junction
   std::shared_ptr<HighwayTranslator> trans2{ std::make_shared<Plain2DTranslator>() };
   std::shared_ptr<HighwayTranslator> trans3{ std::make_shared<Plain3DTranslator>(false) };
   HighwayImage image2{ 3000, 2000, trans2, 4 };
   HighwayImage image3{ 1500, 500, trans3, 4 };
   image2.restartPDF();
   image2.fillImg(BLACK);
   image3.restartPDF();
   image3.fillImg(BLACK);
   //image3.paintEarthAndSky({ 1500, 500 });
   LaneSegment segment11{ 0, 2, 6 };
   LaneSegment segment12{ 20, 0, 4 };
   LaneSegment segment21{ 0, 0, 4 };
   LaneSegment segment22{ 30, 2, 6 };
   LaneSegment segment31{ 0, 2, 6 };
   LaneSegment segment32{ 25, 0, 4 };
   LaneSegment segment41{ 0, 2, 6 };
   LaneSegment segment42{ 35, 0, 4 };
   StraightRoadSection sectiona1{ 4, 50 };
   StraightRoadSection sectiona2{ 4, 60 };
   StraightRoadSection sectiona3{ 4, 100 };
   StraightRoadSection sectiona4{ 4, 55 };
   sectiona1.addLaneSegment(segment11);
   sectiona1.addLaneSegment(segment12);
   sectiona2.addLaneSegment(segment21);
   sectiona2.addLaneSegment(segment22);
   sectiona3.addLaneSegment(segment31);
   sectiona3.addLaneSegment(segment32);
   sectiona4.addLaneSegment(segment41);
   sectiona4.addLaneSegment(segment42);
   std::shared_ptr<CarPars> egoa = std::make_shared<CarPars>(2, 40, 13, HighwayImage::EGO_MOCK_ID);
   std::map<int, std::pair<float, float>> future_positions_of_others1{};
   std::map<int, std::pair<float, float>> future_positions_of_others2{};
   std::map<int, std::pair<float, float>> future_positions_of_others3{};
   std::map<int, std::pair<float, float>> future_positions_of_others4{};
   CarParsVec othersa1{ { 1, -20, 1, 0 }, { 2, -50, 11, 1 } };
   CarParsVec othersa2{ { 2, 4, 3, 2 }, { 1, 22, 11, 3 } };
   CarParsVec othersa3{ { 2, 40, 3, 4 }, { 1, 22, 11, 5 } };
   CarParsVec othersa4{ { 1, 50, 3, 6 }, { 1, 22, 11, 6 } };
   sectiona1.setEgo(egoa);
   sectiona1.setOthers(othersa1);
   sectiona1.setFuturePositionsOfOthers(future_positions_of_others1);
   sectiona2.setEgo(nullptr);
   sectiona2.setOthers(othersa2);
   sectiona2.setFuturePositionsOfOthers(future_positions_of_others2);
   sectiona3.setEgo(nullptr);
   sectiona3.setOthers(othersa3);
   sectiona3.setFuturePositionsOfOthers(future_positions_of_others3);
   sectiona4.setEgo(nullptr);
   sectiona4.setOthers(othersa4);
   sectiona4.setFuturePositionsOfOthers(future_positions_of_others4);

   auto ra1 = std::make_shared<RoadGraph>(1);
   auto ra2 = std::make_shared<RoadGraph>(2);
   auto ra3 = std::make_shared<RoadGraph>(3);
   auto ra4 = std::make_shared<RoadGraph>(4);
   ra1->setMyRoad(sectiona1);

   if (ego_section) {
      ego_section->getMyRoad().setSectionEnd(ra1->getMyRoad().getLength());
      ra1 = ego_section;
   }

   ra1->setOriginPoint({ 0, 6 });
   ra1->setAngle(0);
   ra2->setMyRoad(sectiona2);
   ra2->setOriginPoint({ 75, 5 * vfm::LANE_WIDTH });
   ra2->setAngle(3.1415 / 2.1);
   ra3->setMyRoad(sectiona3);
   ra3->setOriginPoint({ ra3->getMyRoad().getLength() + 85 + vfm::LANE_WIDTH, 6 });
   ra3->setAngle(-3.1415);
   ra4->setMyRoad(sectiona4);
   ra4->setOriginPoint({ 65, -2.5 * vfm::LANE_WIDTH });
   ra4->setAngle(-3.1415 / 1.9);
   ra1->addSuccessor(ra2);
   ra3->addSuccessor(ra2);
   ra1->addSuccessor(ra4);
   ra3->addSuccessor(ra4);

   if (write_to_files) {
      bool old2 = image2.PAINT_ROUNDABOUT_AROUND_EGO_SECTION_FOR_TESTING_;
      bool old3 = image3.PAINT_ROUNDABOUT_AROUND_EGO_SECTION_FOR_TESTING_;
      image2.PAINT_ROUNDABOUT_AROUND_EGO_SECTION_FOR_TESTING_ = false;
      image3.PAINT_ROUNDABOUT_AROUND_EGO_SECTION_FOR_TESTING_ = false;

      image2.paintRoadGraph(ra1, { 500, 60 }, {}, true);
      image2.store("../examples/junction", OutputType::pdf);
      image2.store("../examples/junction", OutputType::png);
      image3.paintRoadGraph(ra1, dim3D);
      image3.store("../examples/junction_3d", OutputType::pdf);
      image3.store("../examples/junction_3d", OutputType::png);

      image2.PAINT_ROUNDABOUT_AROUND_EGO_SECTION_FOR_TESTING_ = old2;
      image3.PAINT_ROUNDABOUT_AROUND_EGO_SECTION_FOR_TESTING_ = old3;
   }

   return ra1;
}

std::shared_ptr<RoadGraph> vfm::test::paintExampleRoadGraphRoundabout(const bool write_to_files, const std::shared_ptr<RoadGraph> ego_section)
{
   assert(!ego_section || ego_section->isRootedInZeroAndUnturned());
   assert(!ego_section || ego_section->getAllNodes().size() == 1);
   assert(!ego_section || ego_section->getMyRoad().getEgo());

   // Roundabout
   std::shared_ptr<HighwayTranslator> trans2{ std::make_shared<Plain2DTranslator>() };
   std::shared_ptr<HighwayTranslator> trans3{ std::make_shared<Plain3DTranslator>(false) };
   HighwayImage image2{ 3000, 2000, trans2, 4 };
   HighwayImage image3{ 1500, 500, trans3, 4 };
   image2.restartPDF();
   image2.fillImg(BLACK);
   image3.restartPDF();
   image3.fillImg(BLACK);
   //image3.paintEarthAndSky({ 1500, 500 });

   static constexpr int lanes0{ 3 };
   static constexpr int lanes1{ 3 };
   static constexpr int lanes2{ 3 };
   static constexpr int lanes3{ 3 };
   static constexpr int lanes4{ 3 };
   static constexpr int lanes5{ 3 };
   static constexpr int lanes6{ 3 };
   static constexpr int lanes7{ 3 };

   LaneSegment segment0{ 0, 0, (lanes0 - 1) * 2 };
   LaneSegment segment1{ 0, 0, (lanes1 - 1) * 2 };
   LaneSegment segment2{ 0, 0, (lanes2 - 1) * 2 };
   LaneSegment segment3{ 0, 0, (lanes3 - 1) * 2 };
   LaneSegment segment4{ 0, 0, (lanes4 - 1) * 2 };
   LaneSegment segment5{ 0, 0, (lanes5 - 1) * 2 };
   LaneSegment segment6{ 0, 0, (lanes6 - 1) * 2 };
   LaneSegment segment7{ 0, 0, (lanes7 - 1) * 2 };
   StraightRoadSection section0{ lanes0, 50 };
   StraightRoadSection section1{ lanes1, 50 };
   StraightRoadSection section2{ lanes2, 50 };
   StraightRoadSection section3{ lanes3, 50 };
   StraightRoadSection section4{ lanes4, 50 };
   StraightRoadSection section5{ lanes5, 50 };
   StraightRoadSection section6{ lanes6, 50 };
   StraightRoadSection section7{ lanes7, 50 };
   section0.addLaneSegment(segment0);
   section1.addLaneSegment(segment1);
   section2.addLaneSegment(segment2);
   section3.addLaneSegment(segment3);
   section4.addLaneSegment(segment4);
   section5.addLaneSegment(segment5);
   section6.addLaneSegment(segment6);
   section7.addLaneSegment(segment7);
   std::shared_ptr<CarPars> ego = std::make_shared<CarPars>(2, 0, 0, HighwayImage::EGO_MOCK_ID);
   std::map<int, std::pair<float, float>> future_positions_of_others{};
   CarParsVec others0{ { 0, 10, 0, 0 } };
   CarParsVec others1{ { 0, 40, 0, 1 } };
   CarParsVec others2{};
   CarParsVec others3{};
   CarParsVec others4{};
   CarParsVec others5{};
   CarParsVec others6{};
   CarParsVec others7{};
   section0.setEgo(ego);
   section0.setOthers(others0);
   section0.setFuturePositionsOfOthers(future_positions_of_others);
   section1.setEgo(nullptr);
   section1.setOthers(others1);
   section1.setFuturePositionsOfOthers(future_positions_of_others);
   section2.setEgo(nullptr);
   section2.setOthers(others2);
   section2.setFuturePositionsOfOthers(future_positions_of_others);
   section3.setEgo(nullptr);
   section3.setOthers(others3);
   section3.setFuturePositionsOfOthers(future_positions_of_others);
   section4.setEgo(nullptr);
   section4.setOthers(others4);
   section4.setFuturePositionsOfOthers(future_positions_of_others);
   section5.setEgo(nullptr);
   section5.setOthers(others5);
   section5.setFuturePositionsOfOthers(future_positions_of_others);
   section6.setEgo(nullptr);
   section6.setOthers(others6);
   section6.setFuturePositionsOfOthers(future_positions_of_others);
   section7.setEgo(nullptr);
   section7.setOthers(others7);
   section7.setFuturePositionsOfOthers(future_positions_of_others);

   auto r0 = std::make_shared<RoadGraph>(0);
   auto r1 = std::make_shared<RoadGraph>(1);
   auto r2 = std::make_shared<RoadGraph>(2);
   auto r3 = std::make_shared<RoadGraph>(3);
   auto r4 = std::make_shared<RoadGraph>(4);
   auto r5 = std::make_shared<RoadGraph>(5);
   auto r6 = std::make_shared<RoadGraph>(6);
   auto r7 = std::make_shared<RoadGraph>(7);

   if (ego_section) {
      ego_section->my_road_.section_end_ = section0.getLength();
      r0 = ego_section;
   }
   else {
      r0->setMyRoad(section0);
   }

   r1->setMyRoad(section1);
   r2->setMyRoad(section2);
   r3->setMyRoad(section3);
   r4->setMyRoad(section4);
   r5->setMyRoad(section5);
   r6->setMyRoad(section6);
   r7->setMyRoad(section7);
   constexpr float rad{ 60.0f };
   const Vec2D mid{ 25, rad };
   Vec2D p0{ 0, 0 };
   Vec2D p1{ 0, 0 };
   Vec2D p2{ 0, 0 };
   Vec2D p3{ 0, 0 };
   Vec2D p4{ 0, 0 };
   Vec2D p5{ 0, 0 };
   const float a0{ 0 };
   const float a1{ 3.1415 / 3 };
   const float a2{ 2 * 3.1415 / 3 };
   const float a3{ 3.1415 };
   const float a4{ 4 * 3.1415 / 3 };
   const float a5{ 5 * 3.1415 / 3 };
   p1.rotate(a1, mid);
   p2.rotate(a2, mid);
   p3.rotate(a3, mid);
   p4.rotate(a4, mid);
   p5.rotate(a5, mid);

   r0->setOriginPoint(p0);
   r0->setAngle(a0);
   r1->setOriginPoint(p1);
   r1->setAngle(a1);
   r2->setOriginPoint(p2);
   r2->setAngle(a2);
   r3->setOriginPoint(p3);
   r3->setAngle(a3);
   r4->setOriginPoint(p4);
   r4->setAngle(a4);
   r5->setOriginPoint(p5);
   r5->setAngle(a5);

   r6->setOriginPoint({ 120, 12.5 * 3.75 });
   r6->setAngle(0);
   r7->setOriginPoint({ 170, 19 * 3.75 });
   r7->setAngle(-3.1415);
   r0->addSuccessor(r1);
   r1->addSuccessor(r2);
   r2->addSuccessor(r3);
   r3->addSuccessor(r4);
   r4->addSuccessor(r5);
   r5->addSuccessor(r0);
   r7->addSuccessor(r2);
   r1->addSuccessor(r6);

   if (write_to_files) {
      bool old2 = image2.PAINT_ROUNDABOUT_AROUND_EGO_SECTION_FOR_TESTING_;
      bool old3 = image3.PAINT_ROUNDABOUT_AROUND_EGO_SECTION_FOR_TESTING_;
      image2.PAINT_ROUNDABOUT_AROUND_EGO_SECTION_FOR_TESTING_ = false;
      image3.PAINT_ROUNDABOUT_AROUND_EGO_SECTION_FOR_TESTING_ = false;

      image2.paintRoadGraph(r1, { 500, 60 }, {}, true, 60, (float)r0->getMyRoad().getNumLanes() / 2.0f);
      image2.store("../examples/roundabout", OutputType::pdf);
      image2.store("../examples/roundabout", OutputType::png);
      image3.paintRoadGraph(r1, dim3D, {}, true, 0, 0);
      image3.store("../examples/roundabout_3d", OutputType::pdf);
      image3.store("../examples/roundabout_3d", OutputType::png);

      image2.PAINT_ROUNDABOUT_AROUND_EGO_SECTION_FOR_TESTING_ = old2;
      image3.PAINT_ROUNDABOUT_AROUND_EGO_SECTION_FOR_TESTING_ = old3;
   }

   return r0;
}

// --- Remaining comments from main.cpp ---

   //   std::string program = R"(
   //@a = 4;
   //if (c == 0) {
   //   @c = 1;
   //   @b = (b + 1) % 15;
   //} else if (c == 1) {
   //   @c = 2;
   //   @b = max(b - 1, 0);
   //} else {
   //   @c = 0;
   //};
   //@b = (b * 2) % 15;
   //if (b == 2 || e > 3 || e < f) {
   //   @b = c * b % 15;
   //};
   //@d = c * b;
   //)";
   //
   //   program = StaticHelper::removeComments(program);
   //   StaticHelper::preprocessCppConvertAllSwitchsToIfs(program);
   //   StaticHelper::preprocessCppConvertElseIfToPlainElse(program);
   //   const auto formula = MathStruct::parseMathStruct(program)->toTermIfApplicable();
   //   auto env_model_dummy = std::make_shared<mc::TypeAbstractionLayer>();
   //   //env_model_dummy->addVariableWithEnumType("f_state.m_state", { "StateMachineState::k_init", "StateMachineState::k_passive", "StateMachineState::k_active" }, "StateMachineState::k_init");
   //   //env_model_dummy->addIntegerVariable("f_state.m_state_machine_resume_speed", 0, 1000, "0");
   //   //env_model_dummy->addIntegerVariable("f_state.m_state_machine_set_speed", 0, 1000, "0");
   //   //env_model_dummy->addIntegerVariable("f_state.m_time_gap_leveh___609___.v", -1000, 1000, "0");
   //   FSMs::testCreationFromFormula(100, formula, 0, 5, 15, 256, 256, 0.01, env_model_dummy, nullptr, true, false, FSMCreationFromFormulaType::quick_translation_to_independent_lines);
   //
   //   for (int i = 0;; i++) {
   //      std::cout << i << ")" << std::endl;
   //      if (!FSMs::testCreationFromFormula(100, nullptr, i, 15, 7, 256, 256, 0.01, nullptr, nullptr, false, false, FSMCreationFromFormulaType::quick_translation_to_independent_lines)) {
   //         std::cout << "FAILED";
   //         std::cin.get();
   //      }
   //   }
   //
   //   termnate();

   // TODO: This fails.
   //std::string fmla_str = "0 || (0 == 116 % 24 + ((id(156 % 117) <= min((0 > (11 || 206)), (147 >= 164) - 64 % 212)) <= rsqrt(55)))";
   //auto fmla = MathStruct::parseMathStruct(fmla_str);
   //MathStruct::testJitVsNojit(fmla);

   //termnate();

   //auto data = std::make_shared<DataPack>();
   //data->addArrayAndOrSetArrayVal(".A", 500, 4);
   //data->addArrayAndOrSetArrayVal(".H_f", 400, 3.4);
   //data->addOrSetSingleVal("a", 6);
   //data->setHeapLocation(10, 3.1234);

   //// This only fails for a 64-Bit-Release build on Windows (VS). Works fine für 32-Bit, Debug and on OSD.
   //MathStruct::testJitVsNojit("--(.A(45))", data, true);
   //termnate();


   //std::string path = "../examples/mc/yaaa/aca5/";
   //auto res = vfm::yaaaModelCheckerWrapper((path + "InlaneDriving.activity_graph.json").c_str(), path.c_str(), true);
   //std::cout << res << std::endl;
   //vfm::freeModelCheckerReturnValue();
   //termnate();

   //   const std::string grammar_str = R"(Rule-separator-symbol: $<block>
   //Inner-separator-symbol: '<block>
   //Transition-symbol: :=><block>
   //Nondeterminism-symbol: :|<block>
   //Pseudo-epsilon-symbol: <><block>
   //Start-symbol: ~<block>
   //Terminals: '!'(')'*'+','x'{'}'<block>
   //Non-terminals: '~'~1'~2'<block>
   //Production-rules:
   //~ :=> '~'+'~1' :| '~1'$
   //~1 :=> '~1'*'~2' :| '~2'$
   //~2 :=> '!'('~')' :| '('~')' :| '*'('~','~')' :| '+'('~','~')' :| 'x'$
   //<block>)";
   //
   //   auto grammar = std::make_shared<earley::Grammar>();
   //   grammar->parseProgram(grammar_str, nullptr);
   //
   //   MathStruct::testEarleyParsing({ { "+", "*", "!" } }, MathStruct::EarleyChoice::Parser, "!(x) + x * x", 0, 2, false, grammar, nullptr, nullptr, true, true);
   //
   //   termnate();

// --- EO remaining comments from main.cpp ---

extern "C"
char* morty(const char* input, char* result, size_t resultMaxLength)
{
   std::string input_str(input);
   auto cars = StaticHelper::split(input_str, ";");
   auto main_file = StaticHelper::readFile("./morty/main.tpl") + "\n";
   int ego_pos = 0;

   cars.erase(cars.end() - 1);

   for (int i = 0; i < cars.size(); i++) {
      auto car = cars[i];

      if (!car.empty()) {
         auto data = StaticHelper::split(car, ",");
         
         // std::cout << data[1] << std::endl;
         // std::cout << i << ": " << data[2] << std::endl;
         // std::cout << data[3] << std::endl;
         // std::cout << data[4] << std::endl;
         
         float x{ std::stof(data[1]) };
         float y{ std::stof(data[2]) };
         float vx{ std::stof(data[3]) };
         float vy{ std::stof(data[4]) };
         
         x = std::max(std::min(x, std::numeric_limits<float>::max()), -300.0f);
         vx = std::max(std::min(vx, 70.0f), 0.0f);

         main_file += "INIT env.veh___6" + std::to_string(i) + "9___.rel_pos = " + std::to_string((int) (x - ego_pos)) + ";\n";
         main_file += "INIT env.veh___6" + std::to_string(i) + "9___.v = " + std::to_string((int) (vx)) + ";\n";

         std::set<int> lanes{};

         if (y < 4) lanes.insert(3);
         if (y > 0 && y < 8) lanes.insert(2);
         if (y > 4 && y < 12) lanes.insert(1);
         if (y > 8) lanes.insert(0);

         for (int lane = 0; lane <= 3; lane++) {
            main_file += "INIT " + std::string(lanes.count(lane) ? "" : "!") + "env.veh___6" + std::to_string(i) + "9___.lane_b" + std::to_string(lane) + ";\n";
         }
      }
   }

   StaticHelper::writeTextToFile(main_file, "./morty/main.smv");
   test::convenienceArtifactRunHardcoded(test::MCExecutionType::mc, "./morty", "fake-json-config-path", "fake-template-path", "fake-includes-path", "fake-cache-path", "./external");
   MCTrace trace{ StaticHelper::extractMCTraceFromNusmvFile("./morty/debug_trace_array.txt")};

   VarValsFloat delta_ve{};
   VarValsFloat delta_ol{};

   if (trace.empty()) { // Return IDLE for now when no CEX found.
      for (int i = 0; i < cars.size(); i++) {
         delta_ol["veh___6" + std::to_string(i) + "9___.on_lane"] = 0;
         delta_ve["veh___6" + std::to_string(i) + "9___.v"] = 0;
      }
   }
   else {
      std::set<std::string> variables_ve{};
      std::set<std::string> variables_ol{};

      for (int i = 0; i < cars.size(); i++) {
         variables_ve.insert("veh___6" + std::to_string(i) + "9___.v");
      }

      for (int i = 0; i < cars.size(); i++) {
         variables_ol.insert("veh___6" + std::to_string(i) + "9___.on_lane");
      }

      delta_ve = trace.getDeltaFromTo(0, 2, variables_ve);
      delta_ol = trace.getDeltaFromTo(0, std::min((int) trace.size() - 1, 6), variables_ol);
   }

   std::string res{};

   for (int i = 0; i < cars.size(); i++) {
      auto d_ol = delta_ol.at("veh___6" + std::to_string(i) + "9___.on_lane");
      auto d_ve = delta_ve.at("veh___6" + std::to_string(i) + "9___.v");

      if (d_ol < 0) {
         res += "LANE_LEFT;";
      }
      else if (d_ol > 0) {
         res += "LANE_RIGHT;";
      }
      else if (d_ve > 0) {
         res += "FASTER;";
      }
      else if (d_ve < 0) {
         res += "SLOWER;";
      }
      else {
         res += "IDLE;";
      }
   }

   snprintf(result, resultMaxLength, "%s", res.c_str());

   return result;
}

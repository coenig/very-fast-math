//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "cpp_parsing/cpp_parser.h"
#include "cpp_parsing/options.h"

using namespace vfm;


vfm::Options::Options() : Failable("VFM-Options") {
   reset();
}

void vfm::Options::fetchOptions(
   const std::vector<std::pair<std::string, std::string>>& keys_values, 
   const std::shared_ptr<mc::TypeAbstractionLayer> type_abstraction_layer,
   const CppParser* caller)
{
   const std::string OPERATOR_ARGNUM_SEPARATOR = "#";
   const std::string ARGNUMS_SEPARATOR = "'";

   for (const auto& key_value : keys_values) {
      addNote("Processing key-value pair '" + key_value.first + " " + OPTION_EQUAL_SIGN + " " + key_value.second + "'.");
      if (key_value.first == VFM_OPTION_POINTERIZE_NAME) {
         auto op_argnums = StaticHelper::split(key_value.second, OPERATOR_ARGNUM_SEPARATOR);
         std::string op_name = op_argnums[0];

         addNote("Pointerizing operator '" + op_name + "' at argument number(s) '" + op_argnums[1] + "'.");

         pointerized_arguments_in_operators_[op_name] = {};
         for (const auto& arg_num_str : StaticHelper::split(op_argnums[1], ARGNUMS_SEPARATOR)) {
            pointerized_arguments_in_operators_.at(op_name).insert(std::stoi(arg_num_str));
         }
      }
      else if (key_value.first == VFM_OPTION_OPTIMIZE) {
         optimization_mode_ = OptimizationMode(key_value.second);
         addNote("Optimization mode set to '" + optimization_mode_.value().getEnumAsString() + "'.");
      }
      else if (key_value.first == VFM_OPTION_FUNCTION_INLINING) {
         function_inline_type_ = FunctionInlineType(key_value.second);
         addNote("Function inlining type set to '" + function_inline_type_.value().getEnumAsString() + "'.");
      }
      else if (key_value.first == vfm::mc::VFM_OPTION_TARGET_MODEL_CHECKER) {
         target_mc_ = vfm::mc::TargetMC(key_value.second);
         type_abstraction_layer->setTargetMCMode(target_mc_.value());
         addNote("Target mc set to '" + target_mc_.value().getEnumAsString() + "'.");
      }
      else if (key_value.first == VFM_OPTION_CREATE_ADDITIONAL_FILES) {
         create_additional_debug_files_ = AdditionalFileCreation(key_value.second);
         addNote("Additional file creation for debug set to '" + create_additional_debug_files_.value().getEnumAsString() + "'.");
      }
      else if (key_value.first == VFM_OPTION_MODE) {
         mode_ = Mode(key_value.second);
         addNote("General mode set to '" + mode_.value().getEnumAsString() + "'.");

         if (isDebug()) {
            setOutputLevels(ErrorLevelEnum::debug, ErrorLevelEnum::error);
            caller->setOutputLevels(ErrorLevelEnum::debug, ErrorLevelEnum::error);
         }
         else {
            setOutputLevels(ErrorLevelEnum::note, ErrorLevelEnum::error);
            caller->setOutputLevels(ErrorLevelEnum::note, ErrorLevelEnum::error);
         }
      }
      else {
         addError("Unknown vfm option: '" + key_value.first + "'.");
      }
   }
}


bool vfm::Options::isDebug() const
{
   return mode_ == ModeEnum::debug || mode_ == ModeEnum::gatekeeper_with_debug || mode_ == ModeEnum::live_debug;
}


bool vfm::Options::isGatekeeper() const
{
   return mode_ == ModeEnum::gatekeeper || mode_ == ModeEnum::gatekeeper_with_debug;
}

void vfm::Options::reset()
{
   pointerized_arguments_in_operators_.clear();
   function_inline_type_ = std::nullopt;
   optimization_mode_ = std::nullopt;
   target_mc_ = std::nullopt;
   create_additional_debug_files_ = std::nullopt;
   mode_ = std::nullopt;
}

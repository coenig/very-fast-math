//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "fsm.h"
#include "failable.h"
#include "enum_wrapper.h"
#include <optional>


namespace vfm {

const std::string VFM_OPTION_FUNCTION_INLINING = "inline_functions"; // Select method of inlining functions.
const std::string VFM_OPTION_FUNCTION_INLINING_ALWAYS = "always";    // Select method of inlining functions.
const std::string VFM_OPTION_FUNCTION_INLINING_NEVER = "never";      // Select method of inlining functions.

const std::string VFM_OPTION_OPTIMIZE = "optimization_mode";
const std::string VFM_OPTION_OPTIMIZE_ALL = "all";
const std::string VFM_OPTION_OPTIMIZE_INNER_ONLY = "inner_only";
const std::string VFM_OPTION_OPTIMIZE_NONE = "none";

const std::string VFM_OPTION_CREATE_ADDITIONAL_FILES = "create_additional_files";
const std::string VFM_OPTION_CREATE_ADDITIONAL_FILES_ALL = "all";
const std::string VFM_OPTION_CREATE_ADDITIONAL_FILES_NONE = "none";

const std::string VFM_OPTION_MODE = "general_mode";
const std::string VFM_OPTION_MODE_GK = "gatekeeper";
const std::string VFM_OPTION_MODE_GKD = "gatekeeper_with_debug";
const std::string VFM_OPTION_MODE_R = "regular";
const std::string VFM_OPTION_MODE_D = "debug";

const std::string VFM_OPTION_POINTERIZE_NAME = "pointerize";
const std::string OPTION_EQUAL_SIGN = "<<";

enum class AdditionalFileCreationEnum {
   all,
   none,
};

const std::map<AdditionalFileCreationEnum, std::string> additional_file_creation_map{
   { AdditionalFileCreationEnum::all, VFM_OPTION_CREATE_ADDITIONAL_FILES_ALL },
   { AdditionalFileCreationEnum::none, VFM_OPTION_CREATE_ADDITIONAL_FILES_NONE },
};

class AdditionalFileCreation : public fsm::EnumWrapper<AdditionalFileCreationEnum>
{
public:
   explicit AdditionalFileCreation(const AdditionalFileCreationEnum& enum_val) : EnumWrapper("AdditionalFileCreation", additional_file_creation_map, enum_val) {}
   explicit AdditionalFileCreation(const int enum_val_as_num) : EnumWrapper("AdditionalFileCreation", additional_file_creation_map, enum_val_as_num) {}
   explicit AdditionalFileCreation(const std::string& enum_val_as_string) : EnumWrapper("AdditionalFileCreation", additional_file_creation_map, enum_val_as_string) {}
   explicit AdditionalFileCreation() : EnumWrapper("AdditionalFileCreation", additional_file_creation_map, AdditionalFileCreationEnum::all) {}
};

enum class OptimizationModeEnum {
   all,
   inner_only,
   none,
};

const std::map<OptimizationModeEnum, std::string> optimization_mode_map{
   { OptimizationModeEnum::all, VFM_OPTION_OPTIMIZE_ALL },
   { OptimizationModeEnum::inner_only, VFM_OPTION_OPTIMIZE_INNER_ONLY },
   { OptimizationModeEnum::none, VFM_OPTION_OPTIMIZE_NONE },
};

class OptimizationMode : public fsm::EnumWrapper<OptimizationModeEnum>
{
public:
   explicit OptimizationMode(const OptimizationModeEnum& enum_val) : EnumWrapper("OptimizationMode", optimization_mode_map, enum_val) {}
   explicit OptimizationMode(const int enum_val_as_num) : EnumWrapper("OptimizationMode", optimization_mode_map, enum_val_as_num) {}
   explicit OptimizationMode(const std::string& enum_val_as_string) : EnumWrapper("OptimizationMode", optimization_mode_map, enum_val_as_string) {}
   explicit OptimizationMode() : EnumWrapper("OptimizationMode", optimization_mode_map, OptimizationModeEnum::all) {}
};

enum class FunctionInlineTypeEnum {
   Always,
   Never,
};

const std::map<FunctionInlineTypeEnum, std::string> function_inline_type_map{
   { FunctionInlineTypeEnum::Always, VFM_OPTION_FUNCTION_INLINING_ALWAYS },
   { FunctionInlineTypeEnum::Never, VFM_OPTION_FUNCTION_INLINING_NEVER },
};

class FunctionInlineType : public fsm::EnumWrapper<FunctionInlineTypeEnum>
{
public:
   explicit FunctionInlineType(const FunctionInlineTypeEnum& enum_val) : EnumWrapper("FunctionInlineType", function_inline_type_map, enum_val) {}
   explicit FunctionInlineType(const int enum_val_as_num) : EnumWrapper("FunctionInlineType", function_inline_type_map, enum_val_as_num) {}
   explicit FunctionInlineType(const std::string& enum_val_as_string) : EnumWrapper("FunctionInlineType", function_inline_type_map, enum_val_as_string) {}
   explicit FunctionInlineType() : EnumWrapper("FunctionInlineType", function_inline_type_map, FunctionInlineTypeEnum::Always) {}
};

enum class ModeEnum {
   gatekeeper,
   gatekeeper_with_debug,
   regular,
   debug,
   live_debug,
};

const std::map<ModeEnum, std::string> mode_map{
   { ModeEnum::gatekeeper, VFM_OPTION_MODE_GK },
   { ModeEnum::gatekeeper_with_debug, VFM_OPTION_MODE_GKD },
   { ModeEnum::regular, VFM_OPTION_MODE_R },
   { ModeEnum::debug, VFM_OPTION_MODE_D },
};

class CppParser;

class Mode : public fsm::EnumWrapper<ModeEnum>
{
public:
   explicit Mode(const ModeEnum& enum_val) : EnumWrapper("FunctionInlineType", mode_map, enum_val) {}
   explicit Mode(const int enum_val_as_num) : EnumWrapper("FunctionInlineType", mode_map, enum_val_as_num) {}
   explicit Mode(const std::string& enum_val_as_string) : EnumWrapper("FunctionInlineType", mode_map, enum_val_as_string) {}
   explicit Mode() : EnumWrapper("FunctionInlineType", mode_map, ModeEnum::regular) {}
};

class Options : public Failable
{
public:
   Options();

   void fetchOptions(
      const std::vector<std::pair<std::string, std::string>>& keys_values, 
      const std::shared_ptr<mc::TypeAbstractionLayer> type_abstraction_layer,
      const CppParser* caller);

   bool isDebug() const;
   bool isGatekeeper() const;

   void reset();

   std::map<std::string, std::set<int>> pointerized_arguments_in_operators_{};

   inline FunctionInlineType getFunctionInlineType() const 
   {
      if (function_inline_type_) {
         return function_inline_type_.value();
      }
      else {
         fail();
         return FunctionInlineType(); // Will never occur.
      }
   }

   inline OptimizationMode getOptimizationMode() const 
   {
      if (optimization_mode_) {
         return optimization_mode_.value();
      }
      else {
         fail();
         return OptimizationMode(); // Will never occur.
      }
   }

   inline vfm::mc::TargetMC getTargetMC() const 
   {
      if (target_mc_) {
         return target_mc_.value();
      }
      else {
         fail();
         return vfm::mc::TargetMC(); // Will never occur.
      }
   }

   inline AdditionalFileCreation getCreateAdditionalDebugFiles() const 
   {
      if (create_additional_debug_files_) {
         return create_additional_debug_files_.value();
      }
      else {
         fail();
         return AdditionalFileCreation(); // Will never occur.
      }
   }

   inline Mode getMode() const 
   {
      if (mode_) {
         return mode_.value();
      }
      else {
         fail();
         return Mode(); // Will never occur.
      }
   }

   inline std::string serialize() const
   {
      std::string s{};

      s += "function_inline_type_ = " + (function_inline_type_ ? function_inline_type_->getEnumAsString() : "NULL") + "\n";
      s += "optimization_mode_ = " + (optimization_mode_ ? optimization_mode_->getEnumAsString() : "NULL") + "\n";
      s += "target_mc_ = " + (target_mc_ ? target_mc_->getEnumAsString() : "NULL") + "\n";
      s += "create_additional_debug_files_ = " + (create_additional_debug_files_ ? create_additional_debug_files_->getEnumAsString() : "NULL") + "\n";
      s += "mode_ = " + (mode_ ? mode_->getEnumAsString() : "NULL") + "\n";

      return s;
   }

private:
   std::optional<FunctionInlineType> function_inline_type_{};
   std::optional<OptimizationMode> optimization_mode_{};
   std::optional<vfm::mc::TargetMC> target_mc_{};
   std::optional<AdditionalFileCreation> create_additional_debug_files_{};
   std::optional<Mode> mode_{};

   inline void fail() const { 
      addFatalError("No value for option."); 
   }
};
} // vfm
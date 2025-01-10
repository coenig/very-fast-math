//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "math_struct.h"
#include "failable.h"
#include "cpp_parsing/cpp_type_struct.h"
#include "cpp_parsing/cpp_type_enum.h"
#include "parser.h"
#include <optional>

namespace vfm {
namespace mc {

const std::string LTL_GLOBALLY = "G";
const std::string LTL_FINALLY = "F";
const std::string LTL_NEXT = "X";
const std::string LTL_UNTIL = "U";
const std::string NO_INIT_DENOTER = "noinit";

using MCInitialValue = std::string;
using MCValues = std::vector<std::string>;
using MCType = std::pair<std::string, MCValues>;
using MCValueGetter = std::function<std::string(const float)>;
using MCTypeWithValueGetter = std::pair<MCType, MCValueGetter>;

struct FullType
{
   CppType cpp_type_{};
   std::string mc_type_name_{};
   std::vector<std::string> mc_type_values_{};
   MCValueGetter value_getter_{};

   inline std::string serialize() const
   {
      std::string s{"FullType:\n"};

      s += "cpp_type_: " + cpp_type_->toString() + "\n";
      s += "mc_type_name_: " + mc_type_name_ + "\n";
      s += "mc_type_values_: ";

      for (const auto& el : mc_type_values_) {
         s += el + ",";
      }

      s += "\n";

      return s;
   }
};

const auto DEFAULT_VALUE_GETTER = [](const float f) {
   return std::to_string(static_cast<float>(f));
};

const std::string USE_RANDOM_INIT_VALUE_DENOTER = "$IDONTCARE$";
const std::string DUMMY_TYPE_NAME = "_dummy_type_";

const std::string TYPE_ABSTRACTION_LAYER_FAILABLE_NAME = "MC_Type_Abstraction_Layer";

constexpr long long MAX_NUM_OF_VALUES{ 10000000 };


enum class TargetMCEnum {
   nusmv,
   kratos,
};

const std::string VFM_OPTION_TARGET_MODEL_CHECKER = "target_mc";     // Option to select the overall translation mode C++ ==> MC.
const std::string VFM_OPTION_TARGET_MODEL_CHECKER_NUSMV = "nusmv";   // Classic nusmv mode.
const std::string VFM_OPTION_TARGET_MODEL_CHECKER_KRATOS = "kratos"; // New kratos mode.

const std::map<TargetMCEnum, std::string> target_mc_map{
   { TargetMCEnum::nusmv, VFM_OPTION_TARGET_MODEL_CHECKER_NUSMV },
   { TargetMCEnum::kratos, VFM_OPTION_TARGET_MODEL_CHECKER_KRATOS },
};

class TargetMC : public fsm::EnumWrapper<TargetMCEnum>
{
public:
   explicit TargetMC(const TargetMCEnum& enum_val) : EnumWrapper("TargetMC", target_mc_map, enum_val) {}
   explicit TargetMC(const int enum_val_as_num) : EnumWrapper("TargetMC", target_mc_map, enum_val_as_num) {}
   explicit TargetMC(const std::string& enum_val_as_string) : EnumWrapper("TargetMC", target_mc_map, enum_val_as_string) {}
   explicit TargetMC() : EnumWrapper("TargetMC", target_mc_map, TargetMCEnum::kratos) {}
};

class TypeAbstractionLayer : public Failable
{
public:
   inline static MCTypeWithValueGetter getMcTypeWithValueGetter(const FullType& f) { return { getMcType(f), f.value_getter_ }; }
   inline static CppType getCppType(const FullType& f) { return f.cpp_type_; }
   inline static MCType getMcType(const FullType& f) { return { f.mc_type_name_, f.mc_type_values_ }; }
   inline static MCValueGetter getMCValueGetter(const FullType& f) { return getMcTypeWithValueGetter(f).second; }
   inline static MCValues getMCValues(const FullType& f) { return getMcType(f).second; }
   inline static std::string getMCTypeName(const FullType& f) { return f.mc_type_name_; }

   TypeAbstractionLayer();

   MCTypeWithValueGetter getMCDummyType();
   MCTypeWithValueGetter getMCBoolType();
   MCTypeWithValueGetter getMCEnumType(const CppTypeEnum& cpp_type);
   MCTypeWithValueGetter getMCIntegerType(const float low, const float high, const std::string& var_name_for_dabugging);

   std::string mcEncodingBool(const bool value);

   /// <summary>
   /// Add a variable with a specific cpp type and explicitly defined mc type.
   /// </summary>
   /// <param name="var_name">Name of the variable.</param>
   /// <param name="type">Type as used to restrict the values, e.g., from float to a discrete subset of ranges incl. C++ type, if available.</param>
   /// <param name="init_value">The init value.</param>
   void addVariableWithType(const std::string& var_name, const FullType& type, const MCInitialValue& init_value);
   
   /// <summary>
   /// Add an enum variable by deriving the values based on an existing enum type.
   /// </summary>
   /// <param name="var_name">Name of the variable.</param>
   /// <param name="type">Enum type to derive from.</param>
   /// <param name="init_value"The initial value of this variable.</param>
   void addVariableWithEnumType(const std::string& var_name, const std::shared_ptr<CppTypeEnum> type, const MCInitialValue& init_value = USE_RANDOM_INIT_VALUE_DENOTER);

   /// <summary>
   /// Add an enum variable with explicit values.
   /// </summary>
   /// <param name="var_name">Name of the variable.</param>
   /// <param name="values">Names of the enum values.</param>
   /// <param name="init_value"The initial value of this variable.</param>
   void addVariableWithEnumType(const std::string& var_name, const std::vector<std::string> values, const MCInitialValue& init_value, const std::string& type_name = "DummyTypeMC");

   /// <summary>
   /// Add a boolean variable.
   /// </summary>
   /// <param name="var_name">Name of the variable.</param>
   void addBooleanVariable(const std::string& var_name, const MCInitialValue& init_value);

   /// <summary>
   /// Add a boolean variable.
   /// </summary>
   /// <param name="var_name">Name of the variable.</param>
   /// <param name="init_value">Init value as actual bool value.</param>
   void addBooleanVariable(const std::string& var_name, const bool init_value);

   /// <summary>
   /// Add an integer variable.
   /// </summary>
   /// <param name="var_name">Name of the variable.</param>
   void addIntegerVariable(const std::string& var_name, const float low, const float high, const MCInitialValue& init_value = USE_RANDOM_INIT_VALUE_DENOTER);

   /// <summary>
   /// Add an integer variable.
   /// </summary>
   /// <param name="var_name">Name of the variable.</param>
   void addIntegerVariable(const std::string& var_name, const MCTypeWithValueGetter& mc_type, const MCInitialValue& init_value);

   /// <summary>
   /// Add a variable with dummy type that can be used for purely C++ variables that cannot be translated towards MC.
   /// </summary>
   /// <param name="var_name">Name of the variable.</param>
   /// <param name="init_value"The initial value of this variable.</param>
   void addDummyTypeVariable(const std::string& var_name, const MCInitialValue& init_value, const std::string& cpp_type_name = "dummy", const DataPack::AssociationType association_type = DataPack::AssociationType::Float);

   /// <summary>
   /// Add a variable and try deriving the mc type from an arbitrary cpp type. This currently works for:
   /// (1) enum,
   /// (2) bool.
   /// </summary>
   /// <param name="var_name">Name of the variable.</param>
   /// <param name="type">Unrestricted cpp type to derive from.</param>
   /// <param name="init_value"The initial value of this variable.</param>
   void addVariableWithTypeByDerivingFrom(const std::string& var_name, const CppType type, const MCInitialValue& init_value);

   static std::string getValueOf(const MCTypeWithValueGetter& type, const float value);

   enum class MCTarget {
      kratos,
      nuxmv
   };

   std::string getTypeOf(const std::string var_name, const MCTarget target) const;
   std::map<std::string, std::pair<FullType, MCInitialValue>> getVariablesWithTypes() const;
   std::string getInitValueFor(const std::string& var_name) const;
   void setInitValueOf(const std::string& var_name, const std::string& init_value);
   std::string serialize(const bool full = false) const;
   std::shared_ptr<TypeAbstractionLayer> copy() const;
   void addAkaMapping(const std::string& planner_varname, const std::string& envmodel_varname);
   std::map<std::string, std::string> getPlannerToEnvModelVarnameMapping() const;
   std::pair<FullType, MCInitialValue> get(const std::string& var_name) const;
   bool contains(const std::string& var_name) const;
   bool hasRange(const std::string& var_name) const;
   std::pair<float, float> getRange(const std::string& var_name) const; // Returns min > max if no range available.

   /// \brief Sets the range to the union of the existing and the new range.
   /// If the variable never existed, the new range is set.
   void extendOrSetRangeOfIntVariable(const std::string& var_name, const float lower_bound, const float upper_bound);

   void setTargetMCMode(const TargetMC& kratos_mode);
   bool isKratosMode() const;

private:
   std::map<std::string, std::pair<FullType, MCInitialValue>> variable_mc_types_{};
   std::map<std::string, std::string> planner_to_envmodel_varname_mapping_{}; // The aka information.
   std::optional<TargetMC> target_mc_mode_{};
};


} // mc
} // vfm

//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "model_checking/type_abstraction_layer.h"
#include "static_helper.h"


using namespace vfm;
using namespace mc;

// Legacy types for G1 demo.
//mutable std::map<std::string, std::string> special_cases_{
//   {"hasyVehicleSpeedKMH", "0..200"},
//{"safeStopTimer", "0..10000"},
//{"vehicleStandStillTimer", "0..10000"},
//{"failureToleranceCycleCounter", "0..200"},
//{"indicatorLeverStateTimeCounter", "0..3000"},
//{"tolerateActuatorStandbyCounter", "0..10"},
//};

TypeAbstractionLayer::TypeAbstractionLayer() : Failable(TYPE_ABSTRACTION_LAYER_FAILABLE_NAME) {}

std::string vfm::mc::TypeAbstractionLayer::mcEncodingBool(const bool value)
{
   return isKratosMode()
      ? (value ? "true" : "false")
      : (value ? "TRUE" : "FALSE");
}

void vfm::mc::TypeAbstractionLayer::addVariableWithType(
   const std::string& var_name,
   const FullType& type, 
   const MCInitialValue& init_value)
{
   addDebug("Adding variable '" + var_name + "' with MC type '" + type.mc_type_name_ + "' (CPP type '" + StaticHelper::trimAndReturn(type.cpp_type_->toString()) + "') to TAL.");

   // TODO: Delete, not needed.
   //if (planner_to_envmodel_varname_mapping_.count(var_name)) {
   //   addDebug("Adding AKA variable '" + planner_to_envmodel_varname_mapping_.at(var_name) + "' with MC type '" + type.mc_type_name_ + "' (CPP type '" + StaticHelper::trimAndReturn(type.cpp_type_->toString()) + "') to TAL.");
   //}

   if (variable_mc_types_.count(var_name)) {
      addDebug("A variable named '" + var_name + "' has already been defined. The old type will be overwritten.");
   }

   addDebug("{{addVariableWithType}} Init value of variable '" + var_name + "' has been set to '" + init_value + "'.");

   variable_mc_types_[var_name] = { type, init_value };
}

void vfm::mc::TypeAbstractionLayer::addVariableWithEnumType(const std::string& var_name, const std::shared_ptr<CppTypeEnum> type, const MCInitialValue& init_value)
{
   auto enum_values = getMCEnumType(*type);
   auto init = init_value;

   if (init_value == USE_RANDOM_INIT_VALUE_DENOTER) {
      init = enum_values.first.second.front();
   }

   FullType full_type{
      type, enum_values.first.first, enum_values.first.second, enum_values.second
   };

   addVariableWithType(var_name, full_type, init);
}

void vfm::mc::TypeAbstractionLayer::addVariableWithEnumType(const std::string& var_name, const std::vector<std::string> values, const MCInitialValue& init_value, const std::string& type_name)
{
   auto cpp_type = std::make_shared<CppTypeEnum>(type_name, values);
   addVariableWithEnumType(var_name, cpp_type, init_value);
}

void vfm::mc::TypeAbstractionLayer::addBooleanVariable(const std::string& var_name, const MCInitialValue& init_value)
{
   auto bool_type = getMCBoolType();

   FullType full_type{
       std::make_shared<CppTypeAtomic>("bool", DataPack::AssociationType::Bool), 
       bool_type.first.first, 
       bool_type.first.second, 
       bool_type.second
   };

   addVariableWithType(var_name, full_type, init_value);
}

void vfm::mc::TypeAbstractionLayer::addBooleanVariable(const std::string& var_name, const bool init_value)
{
   addBooleanVariable(var_name, std::string(mcEncodingBool(init_value)));
}

void vfm::mc::TypeAbstractionLayer::addIntegerVariable(const std::string& var_name, const MCTypeWithValueGetter& mc_type, const MCInitialValue& init_value)
{
   std::string init = init_value;

   if (init == USE_RANDOM_INIT_VALUE_DENOTER) {
      init = mc_type.first.second.front();
   }

   FullType full_type{
      std::make_shared<CppTypeAtomic>("int", DataPack::AssociationType::None),
      mc_type.first.first,
      mc_type.first.second,
      mc_type.second
   };

   addVariableWithType(var_name, full_type, init); //TODO: there is no int in DataPack??
}

void vfm::mc::TypeAbstractionLayer::addIntegerVariable(const std::string& var_name, const float low, const float high, const MCInitialValue& init_value)
{
    addIntegerVariable(var_name, getMCIntegerType(low, high, var_name), init_value);
}

void vfm::mc::TypeAbstractionLayer::addDummyTypeVariable(const std::string& var_name, const MCInitialValue& init_value, const std::string& cpp_type_name, const DataPack::AssociationType association_type)
{
   auto mc_type{ getMCDummyType() };

   FullType full_type{
      std::make_shared<CppTypeAtomic>(cpp_type_name, association_type),
      mc_type.first.first,
      mc_type.first.second,
      mc_type.second
   };

   addVariableWithType(var_name, full_type, init_value);
}

void vfm::mc::TypeAbstractionLayer::addVariableWithTypeByDerivingFrom(const std::string& var_name, const CppType type, const MCInitialValue& init_value)
{
   if (type->toAtomicIfApplicable()) {
      if (type->toAtomicIfApplicable()->getBaseType() == DataPack::AssociationType::Bool) {
         addBooleanVariable(var_name, init_value);
         return;
      }

      if (type->toEnumIfApplicable()) {
         addVariableWithEnumType(var_name, type->toEnumIfApplicable(), init_value);
         return;
      }

      auto int_type{ type->toIntegerIfApplicable() };
      if (int_type) {
         auto low{ int_type->getRange().first };
         auto high{ int_type->getRange().second };
         if (low > high) {
            addWarning("Invalid range for integer variable.");
         }
         addIntegerVariable(var_name, low, high, init_value);
         return;
      }
   }

   addError("Cannot derive MC type automatically from cpp type '" + type->toString() + "'.");
}

std::string vfm::mc::TypeAbstractionLayer::getValueOf(const MCTypeWithValueGetter& type, const float value)
{
   return type.second(value);
}

std::string vfm::mc::TypeAbstractionLayer::getTypeOf(const std::string var_name, const MCTarget target) const
{
   if (!variable_mc_types_.count(var_name)) return "notype";

   std::string name{ getMCTypeName(variable_mc_types_.at(var_name).first) };

   if (target == MCTarget::nuxmv) {
      name = StaticHelper::replaceAll(StaticHelper::replaceAll(StaticHelper::replaceAll(name, "(", "{"), ")", "}"), "|", "");
   }

   return name;
}

std::map<std::string, std::pair<FullType, MCInitialValue>> vfm::mc::TypeAbstractionLayer::getVariablesWithTypes() const
{
   return variable_mc_types_;
}

std::string vfm::mc::TypeAbstractionLayer::getInitValueFor(const std::string& var_name) const
{
   if (!getVariablesWithTypes().count(var_name)) {
      return NO_INIT_DENOTER;
   }
   return getVariablesWithTypes().at(var_name).second; 
}

void vfm::mc::TypeAbstractionLayer::setInitValueOf(const std::string& var_name, const std::string& init_value)
{
   if (variable_mc_types_.count(var_name)) {
      addDebug("{{setInitValueOf}} Init value of variable '" + var_name + "' has been set to '" + init_value + "'.");

      variable_mc_types_.at(var_name).second = init_value;
   }
   else {
      addError("Variable '" + var_name + "' cannot be initialized to '" + init_value + "' because it is not present in the TAL.");
   }
}

MCTypeWithValueGetter vfm::mc::TypeAbstractionLayer::getMCDummyType()
{
   return MCTypeWithValueGetter({ { DUMMY_TYPE_NAME, {} }, [](const float) {
      Failable::getSingleton()->addFatalError("DUMMY TYPE -- Don't use for actual value generation.");
      return ""; } }
   );
}

MCTypeWithValueGetter TypeAbstractionLayer::getMCBoolType()
{
   const auto true_val = mcEncodingBool(true);
   const auto false_val = mcEncodingBool(false);

   return MCTypeWithValueGetter({ { "bool", { true_val, false_val } }, 
      [true_val, false_val](const float f) { return f ? true_val : false_val; } });
}

MCTypeWithValueGetter vfm::mc::TypeAbstractionLayer::getMCEnumType(const CppTypeEnum& cpp_type)
{
   std::string declaration;
   std::vector<std::string> values;
   int min_value = std::numeric_limits<int>::max();
   int max_value = std::numeric_limits<int>::min();
   bool first = true;

   for (const auto& val_pair : cpp_type.getPossibleValues()) {
      std::string comma = first ? "" : (isKratosMode() ? " " : ", ");
      std::string val_name = (isKratosMode() ? "|" : "") + cpp_type.getName() + "____" + val_pair.second + (isKratosMode() ? "|" : ""); // 
      declaration += comma + val_name;
      values.push_back(val_name);
      first = false;
   }

   return MCTypeWithValueGetter({ { std::string(isKratosMode() ? "(enum " : "{") + declaration + (isKratosMode() ? ")" : "}"), values},
      [&cpp_type](const float f) {
         assert(cpp_type.getPossibleValues().count(static_cast<int>(f)) && "There needs to be a mapping from the given number to an enum name.");
         return cpp_type.getPossibleValues().at(static_cast<int>(f));
      } });
}

MCTypeWithValueGetter vfm::mc::TypeAbstractionLayer::getMCIntegerType(const float low, const float high, const std::string& var_name_for_dabugging)
{
   std::vector<std::string> values{};
   double llow{ low };
   double lhigh{ high };

   if (lhigh - llow > MAX_NUM_OF_VALUES) {
      Failable::getSingleton()->addError("Variable '" + var_name_for_dabugging + "' has too high number of values (" + std::to_string(high - low) + ">" + std::to_string(MAX_NUM_OF_VALUES) + ") for MC value getter.");
      values = { "-123456789" };

      return { { std::to_string(low) + ".." + std::to_string(high), values }, [low, high](const float f) -> std::string {
         return "ERROR";
      }};
   }

   for (float i = low; i <= high; i++) {
      values.push_back(std::to_string(i));
   }

   if (!values.empty() && values.back() != std::to_string(high)) {
      values.push_back(std::to_string(high)); // In case of non-integer values make sure that at least low and high are present.
   }

   return MCTypeWithValueGetter({ { (isKratosMode() ? "int" : std::to_string(low) + ".." + std::to_string(high)), values},
      [low, high](const float f) -> std::string {
         if (f < low || f > high) {
            Failable::getSingleton()->addFatalError("Cannot derive MC value for '" + std::to_string(f) + "' in integer range [" + std::to_string(low) + ", " + std::to_string(high) + "].");
            return "";
         }

         return std::to_string(f);
      } });
}

std::string mc::TypeAbstractionLayer::serialize(const bool full) const
{
   std::string s{};

   for (const auto& entry : variable_mc_types_) {
      std::string var_name = entry.first;
      auto full_type = entry.second.first;
      std::string init_value = entry.second.second;

      s += var_name + " (" + (full ? full_type.serialize() : getMCTypeName(full_type)) + "; INIT: " + init_value + ")\n";
   }

   if (full) {
      s += "\nplanner_to_envmodel_varname_mapping_:\n";
      for (const auto& el : planner_to_envmodel_varname_mapping_) {
         s += el.first + " = " + el.second;
      }

      s += "target_mc_mode_: " + (target_mc_mode_ ? target_mc_mode_->getEnumAsString() : "NULL") + "\n";
   }

   return s;
}

std::shared_ptr<TypeAbstractionLayer> mc::TypeAbstractionLayer::copy() const
{
   auto tal_copy = std::make_shared<TypeAbstractionLayer>();

   tal_copy->target_mc_mode_ = target_mc_mode_;

   for (const auto& el : variable_mc_types_) {
      tal_copy->variable_mc_types_.insert({ el.first, { el.second.first, el.second.second } });
   }

   tal_copy->planner_to_envmodel_varname_mapping_ = planner_to_envmodel_varname_mapping_;

   return tal_copy;
}

void vfm::mc::TypeAbstractionLayer::addAkaMapping(const std::string& planner_varname, const std::string& envmodel_varname)
{
   addDebug("Adding mapping from (planner) variable '" + planner_varname + "' to its actual (env model) name '" + envmodel_varname + "'.");

   if (variable_mc_types_.count(planner_varname)) { // TODO: Is this former error message really relevant, even as a debug output?
      addDebug("Possible name clash - (environment) model already contains a variable named '" 
         + planner_varname 
         + "', but you're trying to add a mapping from a (planner) variable with the same name to '" 
         + envmodel_varname 
         + "'.");
   }

   if (!variable_mc_types_.count(envmodel_varname)) {
      std::vector<std::string> found_base_name_envmodel{};
      std::string found_base_name_envmodel_str{};
      std::vector<std::string> found_base_name_planner{};
      std::string found_base_name_planner_str{};

      for (const auto& var : variable_mc_types_) {
         if (StaticHelper::stringStartsWith(var.first, envmodel_varname + ".")) {
            found_base_name_envmodel.push_back(var.first);
            found_base_name_envmodel_str += var.first + "\n";
         }

         if (StaticHelper::stringStartsWith(var.first, planner_varname + ".")) {
            found_base_name_planner.push_back(var.first);
            found_base_name_planner_str += var.first + "\n";
         }
      }

      if (found_base_name_envmodel.empty() && found_base_name_planner.empty() && !planner_to_envmodel_varname_mapping_.count(planner_varname)) {
         addWarning("Trying AKA mapping from (planner) variable '" + planner_varname + "' to (environment model) variable '"
            + envmodel_varname + "' although the target variable name is unknown. Make sure to not blindly use the mapping, assuming the variable is there.");
      }
      else {
         addNote("AKA mapping from (planner) BASE variable '" + planner_varname + "' to (environment model) BASE variable '" + envmodel_varname + "'.");
         addNote("I have found these variables containing '" + envmodel_varname + "' or '" + planner_varname + "' as base name, though, and will aka them all\n" + found_base_name_envmodel_str + "\n" + found_base_name_planner_str);

         for (const auto& var_with_base_from_envmodel : found_base_name_envmodel) {
            std::string back_part_of_envmodel_var_name{ var_with_base_from_envmodel.substr(envmodel_varname.size() + 1) };
            std::string new_planner_var_name{planner_varname + "." + back_part_of_envmodel_var_name};

            addAkaMapping(new_planner_var_name, envmodel_varname + "." + back_part_of_envmodel_var_name);
         }

         for (const auto& var_with_base_from_planner : found_base_name_planner) {
            std::string back_part_of_planner_var_name{ var_with_base_from_planner.substr(planner_varname.size() + 1) };
            std::string new_planner_var_name{planner_varname + "." + back_part_of_planner_var_name};

            addAkaMapping(new_planner_var_name, envmodel_varname + "." + back_part_of_planner_var_name);
         }
      }
   }

   planner_to_envmodel_varname_mapping_.insert({ planner_varname, envmodel_varname });
}

std::map<std::string, std::string> vfm::mc::TypeAbstractionLayer::getPlannerToEnvModelVarnameMapping() const
{
   return planner_to_envmodel_varname_mapping_;
}

std::pair<FullType, MCInitialValue> vfm::mc::TypeAbstractionLayer::get(const std::string& var_name) const
{
   return variable_mc_types_.at(var_name);
}

bool vfm::mc::TypeAbstractionLayer::contains(const std::string& var_name) const
{
   return variable_mc_types_.count(var_name);
}

bool vfm::mc::TypeAbstractionLayer::hasRange(const std::string& var_name) const
{
   auto range = getRange(var_name);
   return range.first <= range.second;
}

std::pair<float, float> vfm::mc::TypeAbstractionLayer::getRange(const std::string& var_name) const
{
   float min = std::numeric_limits<float>::infinity();
   float max = -std::numeric_limits<float>::infinity();

   if (variable_mc_types_.count(var_name)) {
      auto full_type = variable_mc_types_.at(var_name).first;
      auto cpp_type = full_type.cpp_type_;

      if (cpp_type->toBoolIfApplicable()) {
         min = 0;
         max = 1;
      }
      else if (full_type.cpp_type_->toEnumIfApplicable()) {
         for (const auto& possible_value : cpp_type->toEnumIfApplicable()->getPossibleValues()) {
            min = std::min(min, possible_value.first);
            max = std::max(max, possible_value.first);
         }
      }
      else { // Assume integer type.
         auto vec = getMCValues(full_type);

         if (vec.empty()) {
            addDebug("Cannot derive range for '" + var_name + "' because it has no MCValues attached.");
            // Error case. Denoted by min > max in returned pair.
         } else {
            min = std::stof(vec.front());
            max = std::stof(vec.back());
         }
      }
   }
   else {
      addDebug("Cannot derive range for '" + var_name + "' because it is missing in TAL.");
      // Error case. Denoted by min > max in returned pair.
   }
   
   return { min, max };
}

void vfm::mc::TypeAbstractionLayer::extendOrSetRangeOfIntVariable(const std::string& var_name, const float lower_bound, const float upper_bound)
{
   if (!variable_mc_types_.count(var_name)) {
      addNote("Setting range of variable '" + var_name + "' to bound '" + std::to_string(lower_bound) + ".." + std::to_string(upper_bound) + "'.");
      addIntegerVariable(var_name, lower_bound, upper_bound);
   }
   else {
      addNote("Extending range of variable '" + var_name + "' to bound '" + std::to_string(lower_bound) + ".." + std::to_string(upper_bound) + "'.");

      auto& full_type = variable_mc_types_.at(var_name).first;
      auto values = getMCValues(full_type);

      float new_low = values.empty() ? lower_bound : std::min(std::stof(values.front()), lower_bound);
      float new_high = values.empty() ? upper_bound : std::max(std::stof(values.back()), upper_bound);
      auto mc_type_with_value_getter{ getMCIntegerType(new_low, new_high, var_name) };

      full_type.mc_type_name_ = mc_type_with_value_getter.first.first;
      full_type.mc_type_values_ = mc_type_with_value_getter.first.second;
      full_type.value_getter_ = mc_type_with_value_getter.second;
   }
}

void vfm::mc::TypeAbstractionLayer::setTargetMCMode(const TargetMC& mode)
{
   target_mc_mode_ = mode;
   addNote("Target MC mode set to '" + target_mc_mode_.value().getEnumAsString() + "'.");
}

bool vfm::mc::TypeAbstractionLayer::isKratosMode() const
{
   if (target_mc_mode_ == std::nullopt) {
      addFatalError("Target MC mode not set. Help: Use command-line option '// #vfm-option[[ target_mc << kratos ]]' or '// #vfm-option[[ target_mc << nusmv ]]'.");
   }

   return target_mc_mode_ == TargetMCEnum::kratos;
}

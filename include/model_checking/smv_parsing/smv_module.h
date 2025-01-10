//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "smv_module_elements.h"
#include "static_helper.h"
#include "parsable.h"
#include "cpp_parsing/cpp_type_struct.h"
#include "cpp_parsing/cpp_parser.h"
#include "cpp_parsing/cpp_type_enum.h"
#include "data_pack.h"
#include <memory>
#include <variant>

namespace vfm {
namespace mc {
namespace smv {

class SMVDatatypeBool;
class SMVDatatypeInt;
class SMVDatatypeBoundedInt;
class SMVDatatypeEnum;

static const std::string ENUM_QUALIFICATION_DENOTER = "____"; // The :: in C++.

class SMVDatatype : public Parsable, public std::enable_shared_from_this<SMVDatatype>
{
public:
   SMVDatatype(const std::string& cpp_type_name) : Parsable("SMVDatatype-" + cpp_type_name), cpp_type_name_(cpp_type_name) {};

   virtual std::string getSMVName() const = 0;
   virtual std::shared_ptr<CppTypeStruct> getCppType() const = 0;

   std::string getCppTypeName() const
   {
      return cpp_type_name_;
   }

   virtual std::shared_ptr<SMVDatatypeBool> toBoolIfApplicable() { return nullptr; };
   virtual std::shared_ptr<SMVDatatypeInt> toIntIfApplicable() { return nullptr; };
   virtual std::shared_ptr<SMVDatatypeBoundedInt> toBoundedIntIfApplicable() { return nullptr; };
   virtual std::shared_ptr<SMVDatatypeEnum> toEnumIfApplicable() { return nullptr; };

protected:
   void setCppTypeName(const std::string& name)
   {
      cpp_type_name_ = name;
   }

private:
   std::string cpp_type_name_;
};

class SMVDatatypeBool : public SMVDatatype 
{
public:
   SMVDatatypeBool() : SMVDatatype("bool") {}

   std::string getSMVName() const override
   {
      return "boolean";
   }

   std::shared_ptr<CppTypeStruct> getCppType() const override
   {
      return std::make_shared<CppTypeAtomic>("bool", DataPack::AssociationType::Bool);
   }

   bool parseProgram(const std::string& program) override { return true; }

   std::shared_ptr<SMVDatatypeBool> toBoolIfApplicable() override 
   { 
      return std::dynamic_pointer_cast<SMVDatatypeBool>(this->shared_from_this());
   };
};

class SMVDatatypeInt : public SMVDatatype
{
public:
   SMVDatatypeInt() : SMVDatatype("int") {}

   std::string getSMVName() const override 
   {
      return "integer";
   }

   std::shared_ptr<CppTypeStruct> getCppType() const override
   {
      return std::make_shared<CppTypeAtomic>("int", DataPack::AssociationType::None);
   }

   bool parseProgram(const std::string& program) override { return true; }

   std::shared_ptr<SMVDatatypeInt> toIntIfApplicable() override
   { 
      return std::dynamic_pointer_cast<SMVDatatypeInt>(this->shared_from_this());
   };
};

class SMVDatatypeBoundedInt : public SMVDatatype 
{
public:
   SMVDatatypeBoundedInt() : SMVDatatype("int") {}
   SMVDatatypeBoundedInt(const std::string& min, const std::string& max) : SMVDatatype("int"), min_{min}, max_{max} {}

   std::string getSMVName() const override
   {
      return min_ + ".." + max_;
   }

   std::shared_ptr<CppTypeStruct> getCppType() const override
   {
      return std::make_shared<CppTypeAtomic>("int", DataPack::AssociationType::None);
   }

   bool parseProgram(const std::string& program) override
   {
      auto split = StaticHelper::split(program, "..");

      if (split.size() != 2) {
         addError("Cannot parse integer range from input '" + program + "'.");

         return false;
      }

      min_ = StaticHelper::trimAndReturn(split[0]);
      max_ = StaticHelper::trimAndReturn(split[1]);

      return true;
   }

   std::shared_ptr<SMVDatatypeBoundedInt> toBoundedIntIfApplicable() override
   { 
      return std::dynamic_pointer_cast<SMVDatatypeBoundedInt>(this->shared_from_this());
   };

private:
   std::string min_{ "54321"};
   std::string max_{ "-54321"};
};

static const std::string DUMMY_ENUM_NAME = "DUMMY_NAME";

class SMVDatatypeEnum : public SMVDatatype
{
public:
   SMVDatatypeEnum() : SMVDatatype(DUMMY_ENUM_NAME) {}
   SMVDatatypeEnum(const std::vector<std::string>& possible_values) : SMVDatatype(deriveTypeNameAndPossibleValues(possible_values).first), possible_values_{ deriveTypeNameAndPossibleValues(possible_values).second } {}

   std::shared_ptr<CppTypeStruct> getCppType() const override
   {
      return std::make_shared<CppTypeEnum>(getCppTypeName(), possible_values_);
   }

   std::string getSMVName() const override
   {
      std::string s{"{"};

      for (const auto& possible_value : possible_values_) {
         s += getCppTypeName() + "::" + possible_value + ", ";
      }

      s += "}";

      return StaticHelper::replaceAll(s, ", }", "}");
   }

   bool parseProgram(const std::string& program) override {
      if (!StaticHelper::stringContains(program, "{") || !StaticHelper::stringContains(program, "}")) {
         addError("Cannot create enum type, input '" + program + "' is malformed.");

         return false;
      }

      auto split = StaticHelper::split(StaticHelper::removeWhiteSpace(StaticHelper::replaceAll(StaticHelper::replaceAll(program, "{", ""), "}", "")), ",");

      for (const auto& val : split) {
         if (!StaticHelper::isAlphaNumericOrOneOfThese(val, "_:")) {
            addError("Possible value '" + val + "' from enum definition '" + program + "' is malformed.");

            return false;
         }

         possible_values_.push_back(val);
      }

      const auto temp_pair = deriveTypeNameAndPossibleValues(possible_values_);
      setCppTypeName(temp_pair.first);
      possible_values_ = temp_pair.second;

      return true;
   }

   std::shared_ptr<SMVDatatypeEnum> toEnumIfApplicable() override
   { 
      return std::dynamic_pointer_cast<SMVDatatypeEnum>(this->shared_from_this());
   };

private:
   static std::pair<std::string, std::vector<std::string>> deriveTypeNameAndPossibleValues(const std::vector<std::string>& possible_values_)
   {
      std::string name{ DUMMY_ENUM_NAME };
      std::vector<std::string> possible_values{};

      auto func = [&name, &possible_values](const std::string& possible_value) {
         if (StaticHelper::stringContains(possible_value, ENUM_QUALIFICATION_DENOTER)) {
            auto split = StaticHelper::split(possible_value, ENUM_QUALIFICATION_DENOTER);

            if (split.size() != 2) {
               Failable::getSingleton()->addError("Malformed enum name from SMV '" + possible_value + "'.");
            }

            const auto temp_name = split[0];

            if (name != DUMMY_ENUM_NAME && name != temp_name) {
               Failable::getSingleton()->addError("Possible enum value from SMV '" + possible_value + "' is of type '" + temp_name + "' which is different from type '" + name + "' which I have derived earlier.");
            }

            name = temp_name;

            const auto possible_value = split[1];

            if (possible_values.size() != 1 || possible_values[0] != possible_value) { // Check for double insert because we look at the first element twice.
               possible_values.push_back(possible_value);
            }
         }
         else {
            Failable::getSingleton()->addError("Possible enum value from SMV '" + possible_value + "' does not contain full qualification via '" + ENUM_QUALIFICATION_DENOTER + "' and is therefore malformed.");
         }
      };

      if (possible_values_.empty()) {
         Failable::getSingleton()->addError("No possible values given for enum.");
      }
      else {
         func(possible_values_[0]);
      }

      for (const auto& val : possible_values_) {
         func(val);
      }

      return { name, possible_values };
   }

   std::vector<std::string> possible_values_{};
};

class Module : public Failable
{
public:
   Module();

   bool parseProgram(const std::string& program);
   std::map<std::string, std::shared_ptr<SMVDatatype>> getVariablesWithTypes() const;
   std::map<std::string, std::string> getAllVariableInits() const;
   std::map<std::string, std::string> getVariableDefines() const;
   std::set<std::string> getAllVariablesAndDefinesForOutsideUsage() const;
   std::string serialize() const;
   void clear();

private:
   std::map<std::string, ModuleElement> elements_singleton_{
      { NAME_NULL_ELEMENT, NullElement() },
      { NAME_VAR, VarSection() },
      { NAME_IVAR, IVarSection()},
      { NAME_FROZENVAR, FrozenVarSection() },
      { NAME_DEFINE, DefineSection() },
      { NAME_CONSTANTS, ConstantsSection() },
      { NAME_ASSIGN, AssignSection() },
      { NAME_TRANS, TransSection() },
      { NAME_INIT, InitSection() },
      { NAME_INVAR, InvarSection() },
      { NAME_FAIRNESS, FairnessSection() },
      { NAME_JUSTICE, JusticeSection() },
      { NAME_COMPASSION, CompassionSection() },
      { NAME_CTLSPEC, CtlspecSection() },
      { NAME_SPEC, SpecSection() },
      { NAME_CTLSPEC_NAME, CtlspecNameSection() },
      { NAME_SPEC, SpecNameSection() },
      { NAME_INVARSPEC, InvarspecSection() },
      { NAME_INVARSPEC_NAME, InvarspecNameSection() },
      { NAME_LTLSPEC, LtlspecSection() },
      { NAME_LTLSPEC_NAME, LtlspecNameSection() },
      { NAME_PSLSPEC, PslspecSection() },
      { NAME_PSLSPEC_NAME, PslspecNameSection() },
      { NAME_COMPUTE, ComputeSection() },
      { NAME_COMPUTE_NAME, ComputeNameSection() },
      { NAME_PARSYNTH, ParsynthSection() },
      { NAME_ISA, IsaSection() },
      { NAME_PRED, PredSection() },
      { NAME_MIRROR, MirrorSection() } };

   std::string module_name_{};

   std::vector<std::string> splitAlongLines(const std::string& raw_code) const;
};

} // smv
} // mc
} // vfm

//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "cpp_parsing/cpp_type_atomic.h"
#include "parser.h"
#include "static_helper.h"

using namespace vfm;

vfm::CppTypeAtomic::CppTypeAtomic(
   const std::string& type_name,
   const DataPack::AssociationType base_on_type,
   const AssociativitySuite& association_creator,
   const std::shared_ptr<Equation> in_mapping,
   const std::shared_ptr<Equation> out_mapping,
   const float range_low,
   const float range_high)
   : CppTypeStruct(type_name), 
   base_on_type_(base_on_type), 
   association_creator_(association_creator), 
   in_mapping_(in_mapping), 
   out_mapping_(out_mapping), 
   optional_range_{ range_low, range_high }
{ // TODO: Would be efficient-er to avoid resolving when in_mapping_ AND out_mapping_ are trivial.
   if (!in_mapping_) {
      in_mapping_ = _equation(_var(IN_VAR_NAME), _var(OUT_VAR_NAME));
   }

   if (!out_mapping_) {
      std::function<bool(std::shared_ptr<MathStruct>)> match_cond = [](std::shared_ptr<MathStruct> m) -> bool {
         return m->getOptor() == OUT_VAR_NAME;
      };

      out_mapping_ = in_mapping_->copy()->toEquationIfApplicable();
      out_mapping_->toEquationIfApplicable()->resolveTo(match_cond, false);
      out_mapping_->swap();
   }
}

vfm::CppTypeAtomic::CppTypeAtomic(const std::string& type_name, const DataPack::AssociationType base_on_type, const float range_low, const float range_high)
   : CppTypeAtomic(type_name, base_on_type, DEFAULT_ASSOCIATIVITY_SUITE, nullptr, nullptr, range_low, range_high)
{}

std::shared_ptr<CppTypeStruct> vfm::CppTypeAtomic::copy(const bool copy_constness) const
{
   auto in_mapping = std::make_shared<Equation>(in_mapping_->getOperands()[0], in_mapping_->getOperands()[1]);
   auto out_mapping = std::make_shared<Equation>(out_mapping_->getOperands()[0], out_mapping_->getOperands()[1]);

   return std::make_shared<CppTypeAtomic>(getName(), base_on_type_, association_creator_, in_mapping, out_mapping);
}

std::vector<std::pair<std::vector<std::string>, std::string>> vfm::CppTypeAtomic::getCppAssociations(const std::string& base_name, const bool is_const, const int id) const
{
   std::string function_name{};
   std::string base_type_name{};

   if (base_on_type_ == DataPack::AssociationType::Bool) {
      function_name = "initAndAssociateVariableToExternalBool";
      base_type_name = "bool";
   }
   else if (base_on_type_ == DataPack::AssociationType::Char) {
      function_name = "initAndAssociateVariableToExternalChar";
      base_type_name = "char";
   }
   else if (base_on_type_ == DataPack::AssociationType::Float) {
      function_name = "initAndAssociateVariableToExternalFloat";
      base_type_name = "float";
   }
   else { // No association possible due to missing type.
      return {};
   }

   auto generic_struct_name = "GenericMember" + std::to_string(id);
   return { { 
         association_creator_.first(getName(), generic_struct_name, base_type_name, "m_value", id == 0), 
         function_name + "(\"" + base_name + "\", " + association_creator_.second(base_name, base_type_name, generic_struct_name, getName(), is_const) + ")" 
      } };
}

std::shared_ptr<CppTypeAtomic> vfm::CppTypeAtomic::toIntegerIfApplicable()
{
   auto name{ StaticHelper::toLowerCase(getName()) };
   return (name == "int" || name == "integer") ? toAtomicIfApplicable() : nullptr;
}

std::shared_ptr<CppTypeAtomic> vfm::CppTypeAtomic::toAtomicIfApplicable()
{
   return std::dynamic_pointer_cast<CppTypeAtomic>(this->shared_from_this());
}

std::shared_ptr<CppTypeAtomic> vfm::CppTypeAtomic::toBoolIfApplicable()
{
   return getName() == "bool" ? std::dynamic_pointer_cast<CppTypeAtomic>(this->shared_from_this()) : nullptr;
}

std::shared_ptr<Equation> vfm::CppTypeAtomic::getInMapping() const
{
   return in_mapping_;
}

std::shared_ptr<Equation> vfm::CppTypeAtomic::getOutMapping() const
{
   return out_mapping_;
}

DataPack::AssociationType vfm::CppTypeAtomic::getBaseType() const
{
   return base_on_type_;
}

std::pair<float, float> vfm::CppTypeAtomic::getRange() const
{
   return optional_range_;
}

void vfm::CppTypeAtomic::setRange(const float low, const float high)
{
   if (!StaticHelper::isFloatInteger(low) && low != high) {
      Failable::getSingleton()->addError("Range of type '" + toString() + "' is '" + std::to_string(low) + ".." + std::to_string(high) + "', which is both non-integer and non-const. This is currently not supported.");
   }

   optional_range_ = { low, high };
}

void vfm::CppTypeAtomic::extendRange(const float low, const float high)
{
   setRange(std::min(optional_range_.first, low), std::max(optional_range_.second, high));
}

void vfm::CppTypeAtomic::extendRange(const std::pair<float, float>& range)
{
   extendRange(range.first, range.second);
}

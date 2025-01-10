//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_var.h"
#include "static_helper.h"
#include "math_struct.h"
#include "parser.h"
#include "string"

using namespace vfm;

float TermVar::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
   if (constant_ && getOptor()[0] != SYMB_REF[0]) { // If it's a reference, we're not interested in the value it's holding, but the place it's sitting.
      return constant_value_; // No JIT in this case since it gains little, and I'm not sure if it could break something.
   }

#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return varVals->getSingleVal(getOptor());
}

void vfm::TermVar::setOptor(const std::string& new_optor_name)
{
   MathStruct::setOptor(new_optor_name);
   this->my_struct_.op_name = new_optor_name;
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermVar::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   if (StaticHelper::stringStartsWith(getOptor(), SYMB_REF)) {
      std::string real_name = getOptor().substr(SYMB_REF.length());
      d->getAddressOfSingleVal(real_name);
      return _val(d->getAddressOf(real_name))->createSubAssembly(cc, d, p);
   }

   x86::Xmm x = cc.newXmm();

   float* addressToFloatVal = d->getAddressOfSingleVal(getOptor());
   bool* addressToBoolVal = d->getAddressOfExternalBool(getOptor());
   char* addressToCharVal = d->getAddressOfExternalChar(getOptor());

   if (addressToBoolVal) {
      setXmmVarToAddressLocation(cc, x, addressToBoolVal);
   }
   else if (addressToCharVal) {
      setXmmVarToAddressLocation(cc, x, addressToCharVal);
   }
   else if (addressToFloatVal) {
      setXmmVarToAddressLocation(cc, x, addressToFloatVal);
   }
   else {
      Failable::getSingleton()->addError("Assembly not created due to ill-defined reference for variable '" + getOptor() + "'.");
      return nullptr;
   }

   return std::make_shared<x86::Xmm>(x);
}
#endif

OperatorStructure vfm::TermVar::getOpStruct() const
{
   return my_struct_;
}

TermVar::TermVar(std::string varname):
   TermConst::TermConst(varname), my_struct_(OutputFormatEnum::plain, AssociativityTypeEnum::left, 0, 1000, varname, true, true)
{
}

bool TermVar::isOverallConstant(const std::shared_ptr<std::vector<int>> non_const_metas,
   const std::shared_ptr<std::set<std::shared_ptr<MathStruct>>> visited)
{
   return constant_;
}

bool TermVar::isTermVar() const
{
   return true;
}

bool vfm::TermVar::isTermVarOnLeftSideOfAssignment() const
{
   auto father{ getFather() };
   return father && father->thisPtrGoUpToCompound()->isTermSetVarOrAssignment() && father->getOperands()[0] == shared_from_this();
}

bool vfm::TermVar::isTermVarNotOnLeftSideOfAssignment() const
{
   return !isTermVarOnLeftSideOfAssignment();
}

std::shared_ptr<TermVar> vfm::TermVar::toVariableIfApplicable()
{
   return std::dynamic_pointer_cast<TermVar>(this->shared_from_this());
}

std::shared_ptr<Term> TermVar::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   auto copy_term = std::make_shared<TermVar>(getOptor());

   copy_term->private_variable_ = private_variable_;
   copy_term->constant_ = constant_;
   copy_term->constant_value_ = constant_value_;

   return copy_term;
}

std::string TermVar::getVariableName() const
{
   return my_struct_.op_name;
}

void vfm::TermVar::setVariableName(const std::string& name)
{
   setOptor(name);
}

void replacementsMC(std::string& reg_ser, const bool kratos, const std::map<std::string, std::string>& enum_values)
{
   std::string quotation = kratos ? "|" : "";
   reg_ser = StaticHelper::cleanVarNameOfPossibleRefSymbol(reg_ser);

   if (reg_ser == "true") {
      reg_ser = kratos ? "true" : "TRUE";
      quotation = "";
   }
   else if (reg_ser == "false") {
      reg_ser = kratos ? "false" : "FALSE";
      quotation = "";
   }
   else if (StaticHelper::stringContains(reg_ser, "::")) {
      //int pos = reg_ser.find("::");

      if (kratos && enum_values.count(reg_ser)) { // It's an enum.
         auto values = enum_values.at(reg_ser);
         reg_ser = "(const |" + reg_ser + "| (enum" + values + "))";
         quotation = "";
      } // TODO: What about static variables of some class or namespace denoters?
      else {
         //reg_ser = reg_ser.substr(pos + 2);
      }
   }

   reg_ser = StaticHelper::replaceAll(reg_ser, ":::", "\""); // ::: is denoter for quote which is needed for addressing planner variables.
   reg_ser = StaticHelper::replaceAll(reg_ser, "::", "____");
   reg_ser = quotation + reg_ser + quotation;
}

void vfm::TermVar::serialize(
   std::stringstream& os,
   const std::shared_ptr<MathStruct> highlight,
   const SerializationStyle style,
   const SerializationSpecial special,
   const int indent,
   const int indent_step,
   const int line_len,
   const std::shared_ptr<DataPack> data,
   std::set<std::shared_ptr<const MathStruct>>& visited) const
{
   if (possiblyHighlightAndGoOnSerializing(os, highlight, style, special, indent, indent_step, line_len, data, visited)) {
      return;
   }

   auto reg_ser = getVariableName();

   if (style == SerializationStyle::nusmv) {
      replacementsMC(reg_ser, false, {});
      os << reg_ser;
   } 
   else if (style == SerializationStyle::cpp && special == SerializationSpecial::remove_symb_ref) {
      os << (StaticHelper::replaceAll(reg_ser, SYMB_REF, ""));
   }
   else {
      MathStruct::serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);

      if (data) { // Creates a non-parsable output. TODO: This is probably not the desired default behavior. Maybe make dependent of variable in data.
         bool has_possibly_been_set_before = true;
         //auto currptr = const_cast<TermVar*>(this)->toTermIfApplicable();
         //auto father = currptr->getFather();
         //auto grandad = father ? father->getFather() : nullptr;

         //std::cout << serializeWithinSurroundingFormulaPlainOldVFMStyle() << std::endl;

         //while (grandad) {
         //   while (grandad && grandad->isTermSequence() && father != grandad->child0()) {
         //      if (grandad->child1()->isTermSetVarOrAssignment()) {
         //         std::cout << grandad->child1()->serializeWithinSurroundingFormulaPlainOldVFMStyle() << std::endl;
         //      }

         //      father = grandad;
         //      grandad = grandad->child0();
         //   }

         //   if (grandad->isTermSetVarOrAssignment()) {
         //      std::cout << grandad->serializeWithinSurroundingFormulaPlainOldVFMStyle() << std::endl;

         //      father = grandad;
         //      grandad = grandad->getFather();
         //      while (grandad && grandad->isTermSequence()) {
         //         father = grandad;
         //         grandad = grandad->getFather();
         //      }
         //   }
         //   
         //   if (grandad && (grandad->isTermIf() || grandad->isTermIfelse())) {
         //      father = grandad;
         //      grandad = grandad->getFather();
         //   }
         //   else {
         //      grandad = nullptr;
         //   }
         //}
         //std::cout << "\n";

         os << (has_possibly_been_set_before ? COLOR_PINK : COLOR_DARK_GREEN) << " /*" << data->getSingleVal(StaticHelper::cleanVarNameOfPossibleRefSymbol(reg_ser)) << "*/ " << RESET_COLOR;
      }
   }

   visited.insert(shared_from_this());
}

void vfm::TermVar::setPrivateVariable(const bool isPrivate)
{
   private_variable_ = isPrivate;
}

bool vfm::TermVar::isPrivateVariable() const
{
   return private_variable_;
}

void vfm::TermVar::setConstVariable(const float val)
{
   constant_ = true;
   constant_value_ = val;
}

void vfm::TermVar::makeNonConst()
{
   constant_ = false;
   constant_value_ = std::numeric_limits<float>::min();
}

bool vfm::TermVar::isConstVariable() const
{
   return constant_;
}

std::string vfm::TermVar::serializeK2(const std::map<std::string, std::string>& enum_values, std::pair<std::map<std::string, std::string>, std::string>& additional_functions, int& label_counter) const
{
   auto res = getVariableName();
   replacementsMC(res, true, enum_values);
   return res;
}

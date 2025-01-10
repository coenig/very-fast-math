//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_set_var.h"
#include "term_compound.h"
#include "static_helper.h"
#include "parser.h"

using namespace vfm;

const OperatorStructure TermSetVar::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_SET_VAR, PRIO_SET_VAR, SYMB_SET_VAR, false, false);

TermSetVar::TermSetVar(const std::vector<std::shared_ptr<Term>>& vt) : Term(vt) {}
TermSetVar::TermSetVar(const std::shared_ptr<Term> var, const std::shared_ptr<Term> val) : TermSetVar(std::vector<std::shared_ptr<Term>>{var, val}) {}
TermSetVar::TermSetVar(const std::string& var_name, const std::shared_ptr<Term> val, const std::shared_ptr<DataPack> d) : TermSetVar(getVarAddress(var_name, d), val) {}

bool vfm::TermSetVar::isTermSetVarOrAssignment() const
{
   return true;
}

std::shared_ptr<TermSetVar> vfm::TermSetVar::toSetVarIfApplicable()
{
   return std::dynamic_pointer_cast<TermSetVar>(this->shared_from_this());
}

OperatorStructure TermSetVar::getOpStruct() const
{
   return TermSetVar::my_struct;
}

float TermSetVar::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   int address = getOperands()[0]->eval(varVals, parser, suppress_sideeffects);
   float result = getOperands()[1]->eval(varVals, parser, suppress_sideeffects);

   if (!suppress_sideeffects) {
      varVals->setSingleVal(address, result);
   }

   return result;
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermSetVar::createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return nullptr;
}
#endif

std::shared_ptr<Term> TermSetVar::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   std::shared_ptr<Term> ptr(new TermSetVar(getOperands()[0]->copy(already_copied), getOperands()[1]->copy(already_copied)));
   ptr->setChildrensFathers(false, false);
   return ptr;
}

bool vfm::TermSetVar::hasSideeffectsThis() const
{
   if (isCompoundStructure()) { // This only works if all encapsulated setters have the set variable at p_(0).
      std::shared_ptr<TermCompound> curr_term = getCompoundStructureReferences()[0].lock();

      while (curr_term->isCompoundStructure()) {
         curr_term = curr_term->getCompoundStructureReferences()[0].lock();
      }

      return !curr_term->getOperands()[0]->isTermVar() || !curr_term->getOperands()[0]->toVariableIfApplicable()->isPrivateVariable(); // If the set variable isn't private, it's a side effect.
   }

   return !getOperandsConst()[0]->isTermVar() || !getOperandsConst()[0]->toVariableIfApplicable()->isPrivateVariable(); // If the set variable isn't private, it's a side effect.
}

bool vfm::TermSetVar::isOverallConstant(
   const std::shared_ptr<std::vector<int>> non_const_metas,
   const std::shared_ptr<std::set<std::shared_ptr<MathStruct>>> visited)
{
   return false;
}

std::string vfm::TermSetVar::getVarName(const std::shared_ptr<DataPack> d, const std::shared_ptr<FormulaParser> parser) const
{
   const auto& name = StaticHelper::cleanVarNameOfPossibleRefSymbol(
      getOperandsConst()[0]->toVariableIfApplicable() ? getOperandsConst()[0]->toVariableIfApplicable()->getVariableName() : CODE_FOR_ERROR_VAR_NAME);

   return name;
}

void vfm::TermSetVar::changeVarToSet(const std::string& var_name, const std::shared_ptr<DataPack> d)
{
   getOperands()[0] = _val(d->getAddressOf(var_name));
}

std::shared_ptr<Term> TermSetVar::getVarAddress(const std::string& var_name, const std::shared_ptr<DataPack> d)
{
   return _val(d->getAddressOf(var_name));
}
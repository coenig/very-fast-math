//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_set_arr.h"
#include "static_helper.h"
#include "parser.h"

using namespace vfm;

const OperatorStructure TermSetArr::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_SET_ARR, PRIO_SET_ARR, SYMB_SET_ARR, false, false);

TermSetArr::TermSetArr(const std::vector<std::shared_ptr<Term>>& vt) : Term(vt) {}
TermSetArr::TermSetArr(const std::shared_ptr<Term> var, const std::shared_ptr<Term> idx, const std::shared_ptr<Term> val) : TermSetArr(std::vector<std::shared_ptr<Term>>{var, idx, val}) {}
TermSetArr::TermSetArr(const std::string& var_name, const std::shared_ptr<Term> idx, const std::shared_ptr<Term> val, const std::shared_ptr<DataPack> d) : TermSetArr(getArrAddress(var_name, d), idx, val) {}

bool vfm::TermSetArr::isTermSetArr() const
{
   return true;
}

std::shared_ptr<TermSetArr> vfm::TermSetArr::toSetArrIfApplicable()
{
   return std::dynamic_pointer_cast<TermSetArr>(this->shared_from_this());
}

OperatorStructure TermSetArr::getOpStruct() const
{
   return TermSetArr::my_struct;
}

float TermSetArr::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   int address = getOperands()[0]->eval(varVals, parser, suppress_sideeffects);
   int index = getOperands()[1]->eval(varVals, parser, suppress_sideeffects);
   float result = getOperands()[2]->eval(varVals, parser, suppress_sideeffects);

   if (!suppress_sideeffects) {
      varVals->setArrayVal(address, index, result);
   }

   return result;
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermSetArr::createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return nullptr;
}
#endif

std::shared_ptr<Term> TermSetArr::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   std::shared_ptr<Term> ptr(new TermSetArr(getOperands()[0]->copy(already_copied), getOperands()[1]->copy(already_copied), getOperands()[2]->copy(already_copied)));
   ptr->setChildrensFathers(false, false);
   return ptr;
}

bool vfm::TermSetArr::hasSideeffectsThis() const
{
   return true; // d && !StaticHelper::isRenamedPrivateVar(getVarName(d)); // If the variable isn't private (meaning within compound structure and renamed), it's a side effect.
}

bool vfm::TermSetArr::isOverallConstant(
   const std::shared_ptr<std::vector<int>> non_const_metas,
   const std::shared_ptr<std::set<std::shared_ptr<MathStruct>>> visited)
{
   return false;
}

std::string vfm::TermSetArr::getArrName(const std::shared_ptr<DataPack> d, const std::shared_ptr<FormulaParser> parser) const
{
   return d->getNameOf(getOperandsConst().at(0)->eval(d, parser, true));
}

void vfm::TermSetArr::changeArrToSet(const std::string& var_name, const std::shared_ptr<DataPack> d)
{
   getOperands()[0] = _val(d->getAddressOf(var_name));
}

std::shared_ptr<Term> TermSetArr::getArrAddress(const std::string& var_name, const std::shared_ptr<DataPack> d)
{
   return _val(d->getAddressOf(var_name));
}
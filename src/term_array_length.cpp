//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_array_length.h"
#include "dat_src_arr.h"

using namespace vfm;

const OperatorStructure TermArrayLength::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_LENGTH, PRIO_LENGTH, SYMB_LENGTH, false, false);

TermArrayLength::TermArrayLength(const std::vector<std::shared_ptr<Term>>& t) : Term(t) {}
TermArrayLength::TermArrayLength(const std::shared_ptr<Term>& t) : TermArrayLength(std::vector<std::shared_ptr<Term>>{t}) {}

OperatorStructure TermArrayLength::getOpStruct() const
{
   return my_struct;
}

float TermArrayLength::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return varVals->getStaticArray(static_cast<int>(getOperands()[0]->eval(varVals, parser, suppress_sideeffects))).size();
}

std::shared_ptr<Term> TermArrayLength::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied)
{
   std::shared_ptr<Term> ptr(new TermArrayLength(getOperands()[0]->copy(already_copied)));
   ptr->setChildrensFathers(false, false);
   return ptr;
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermArrayLength::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   x86::Xmm x = cc.newXmm();
   setXmmVarToAddressLocation(cc, x, d->getStaticArray(getOperands()[0]->eval(d, p)).getAddressOfArraySize());
   return std::make_shared<x86::Xmm>(x);
}
#endif

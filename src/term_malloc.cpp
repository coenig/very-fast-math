//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_malloc.h"
#include "meta_rule.h"
#include "parser.h"

using namespace vfm;

const OperatorStructure TermMalloc::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_MALLOC, PRIO_MALLOC, SYMB_MALLOC, true, true);

TermMalloc::TermMalloc(const std::vector<std::shared_ptr<Term>>& t) : Term(t) {}
TermMalloc::TermMalloc(const std::shared_ptr<Term>& t) : TermMalloc(std::vector<std::shared_ptr<Term>>{t}) {}

OperatorStructure TermMalloc::getOpStruct() const
{
   return my_struct;
}

float TermMalloc::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   float cells_to_reserve = this->getOperands()[0]->eval(varVals, parser, suppress_sideeffects);
   float first_cell_address = varVals->getUsedHeapSize();
   varVals->reserveHeap(cells_to_reserve);
   return first_cell_address;
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermMalloc::createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return nullptr;
}
#endif

bool vfm::TermMalloc::hasSideeffectsThis() const
{
   return true;
}

std::shared_ptr<Term> TermMalloc::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   std::shared_ptr<Term> ptr(new TermMalloc(this->getOperands()[0]->copy(already_copied)));
   ptr->setChildrensFathers(false, false);
   return ptr;
}

std::set<MetaRule> vfm::TermMalloc::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);

   vec.insert(MetaRule(_malloc(_vara()), _val0(), SIMPLIFICATION_STAGE_MAIN, { {_vara(), ConditionType::is_constant_0 } }));

   return vec;
}

//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_delete.h"
#include "meta_rule.h"
#include "parser.h"

using namespace vfm;

const OperatorStructure TermDelete::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_DELETE, PRIO_DELETE, SYMB_DELETE, true, true);

TermDelete::TermDelete(const std::vector<std::shared_ptr<Term>>& t) : Term(t) {}
TermDelete::TermDelete(const std::shared_ptr<Term>& t) : TermDelete(std::vector<std::shared_ptr<Term>>{t}) {}

OperatorStructure TermDelete::getOpStruct() const
{
   return my_struct;
}

float TermDelete::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   float cells_to_delete = getOperands()[0]->eval(varVals, parser, suppress_sideeffects);

   if (cells_to_delete > 0) {
      return varVals->deleteHeapCells(cells_to_delete);
   }

   return 0;
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermDelete::createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return nullptr;
}
#endif

bool vfm::TermDelete::hasSideeffectsThis() const
{
   return true;
}

std::shared_ptr<Term> TermDelete::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   std::shared_ptr<Term> ptr(new TermDelete(getOperands()[0]->copy(already_copied)));
   ptr->setChildrensFathers(false, false);
   return ptr;
}

std::set<MetaRule> vfm::TermDelete::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);

   vec.insert(MetaRule(_delete(_vara()), _val0(), SIMPLIFICATION_STAGE_MAIN, { {_vara(), ConditionType::is_constant_0 } }));

   return vec;
}

//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_abs.h"
#include "meta_rule.h"
#include "parser.h"

using namespace vfm;

const OperatorStructure TermAbs::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_ABS, PRIO_ABS, SYMB_ABS, true, true);

TermAbs::TermAbs(const std::vector<std::shared_ptr<Term>>& t) : TermArith(t) {}
TermAbs::TermAbs(const std::shared_ptr<Term>& t) : TermAbs(std::vector<std::shared_ptr<Term>>{t}) {}

OperatorStructure TermAbs::getOpStruct() const
{
   return my_struct;
}

float TermAbs::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return std::abs(this->getOperands()[0]->eval(varVals, parser, suppress_sideeffects));
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermAbs::createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   lambda_so lambda = [](asmjit::x86::Compiler& cc, x86::Xmm& x) {
      x86::Xmm y = cc.newXmm();
      setXmmVar(cc, y, 0);
      cc.subss(y, x);
      cc.maxss(x, y);
   };

   return subAssemblySingleOp(cc, d, lambda);
}
#endif

std::shared_ptr<Term> TermAbs::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   std::shared_ptr<Term> ptr(new TermAbs(this->getOperands()[0]->copy(already_copied)));
   ptr->setChildrensFathers(false, false);
   return ptr;
}

std::set<MetaRule> vfm::TermAbs::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);

   vec.insert(MetaRule(_abs(_neg(_vara())), _abs(_vara()), SIMPLIFICATION_STAGE_MAIN)); // abs(-x) = abs(x)
   vec.insert(MetaRule(_abs(_abs(_vara())), _abs(_vara()), SIMPLIFICATION_STAGE_MAIN)); // abs(abs(x)) = abs(x)

   return vec;
}

std::string vfm::TermAbs::serializeK2(const std::map<std::string, std::string>& enum_values, std::pair<std::map<std::string, std::string>, std::string>& additional_functions, int& label_counter) const
{
   // (abs a) becomes (ite (ge a (const 0 int)) a (neg a))
   auto greq_part = _greq_plain(child0(), _val0())->serializeK2(enum_values, additional_functions, label_counter);
   auto pos_a = child0()->serializeK2(enum_values, additional_functions, label_counter);
   auto neg_a = _neg_plain(child0())->serializeK2(enum_values, additional_functions, label_counter);

   return "(ite " + greq_part + " " + pos_a + " " + neg_a + ")";
}

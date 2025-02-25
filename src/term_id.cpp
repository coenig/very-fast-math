//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_id.h"
#include "meta_rule.h"
#include "parser.h"

using namespace vfm;

const OperatorStructure TermID::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_ID, PRIO_ID, SYMB_ID, false, false);

TermID::TermID(const std::vector<std::shared_ptr<Term>>& t) : TermArith(t) {}
TermID::TermID(const std::shared_ptr<Term>& t) : TermID(std::vector<std::shared_ptr<Term>>{t}) {}

std::shared_ptr<Term> TermID::getInvPattern(int argNum)
{
   return MathStruct::parseMathStruct("p_(0)", false)->toTermIfApplicable();
}

OperatorStructure TermID::getOpStruct() const
{
   return my_struct;
}

float TermID::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return getOperands()[0]->eval(varVals, parser, suppress_sideeffects);
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermID::createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return getOperands()[0]->createSubAssembly(cc, d, p);
}
#endif

std::shared_ptr<Term> TermID::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   return copySimple<TermID>(already_copied);
}

bool vfm::TermID::isTermIdentity() const
{
   return true;
}

std::set<MetaRule> TermID::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec{ MathStruct::getSimplificationRules(parser) };

   vec.insert(MetaRule(_id(_vara()), _vara(), SIMPLIFICATION_STAGE_MAIN));

   return vec;
}

std::string TermID::getNuSMVOperator() const
{
   return "#UNSUPPORTED-OPERATOR[" + getOptor() + "]";
}

std::string TermID::getK2Operator() const
{
   return "#UNSUPPORTED-OPERATOR[" + getOptor() + "]";
}
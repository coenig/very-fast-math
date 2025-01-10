//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_logic_neq.h"
#include "meta_rule.h"
#include "parser.h"
#include <functional>

using namespace vfm;

const OperatorStructure TermLogicNeq::my_struct(OutputFormatEnum::infix, AssociativityTypeEnum::left, OPNUM_NEQ, PRIO_NEQ, SYMB_NEQ, false, true, AdditionalOptionEnum::always_print_brackets);

OperatorStructure TermLogicNeq::getOpStruct() const
{
   return my_struct;
}

float TermLogicNeq::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return evalSimple(varVals, std::not_equal_to<float>(), getOperands()[0]->eval(varVals, parser, suppress_sideeffects), 1, parser, suppress_sideeffects, true); // Invert if nan.
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermLogicNeq::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   lambda_vno lambda = [](asmjit::x86::Compiler& cc, x86::Xmm& x, x86::Xmm& y) {
      xmmCompareTo(cc, x, y, cmp_not_equal);
   };

   return subAssemblyVarnumOps(cc, d, lambda);
}
#endif

std::string vfm::TermLogicNeq::serializeK2(const std::map<std::string, std::string>& enum_values, std::pair<std::map<std::string, std::string>, std::string>& additional_functions, int& label_counter) const
{
   return _not_plain(_eq_plain(child0(), child1()))->serializeK2(enum_values, additional_functions, label_counter);
}

TermLogicNeq::TermLogicNeq(const std::vector<std::shared_ptr<Term>>& opds) : TermLogic(opds)
{
}

vfm::TermLogicNeq::TermLogicNeq(const std::shared_ptr<Term> opd1, const std::shared_ptr<Term> opd2) : TermLogicNeq(std::vector<std::shared_ptr<Term>>{opd1, opd2})
{
}

std::shared_ptr<Term> TermLogicNeq::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   return copySimple<TermLogicNeq>(already_copied);
}

std::set<MetaRule> vfm::TermLogicNeq::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);

   vec.insert(MetaRule(_neq(_vara(), _varb()), _boolify(_vara()), SIMPLIFICATION_STAGE_MAIN, {{ _varb(), ConditionType::is_constant_and_evaluates_to_false }, { _vara(), ConditionType::contains_no_meta } })); // "a != b", "bool(a)"
   vec.insert(MetaRule(_neq(_varb(), _vara()), _boolify(_vara()), SIMPLIFICATION_STAGE_MAIN, {{ _varb(), ConditionType::is_constant_and_evaluates_to_false }, { _vara(), ConditionType::contains_no_meta } })); // "b != a", "bool(a)"
   vec.insert(MetaRule(_neq(_vara(), _vara()), _val0(), SIMPLIFICATION_STAGE_MAIN, {}, ConditionType::has_no_sideeffects));                                   // "a != a", "0"
   vec.insert(MetaRule(_neq(_neg(_vara()), _neg(_varb())), _neq(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN));                                               // "--(a) != --(b)", "a != b"
   
   return vec;
}

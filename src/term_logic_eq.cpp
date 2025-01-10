//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_logic_eq.h"
#include "meta_rule.h"
#include "parser.h"
#include <functional>

using namespace vfm;

const OperatorStructure TermLogicEq::my_struct(OutputFormatEnum::infix, AssociativityTypeEnum::left, OPNUM_EQ, PRIO_EQ, SYMB_EQ, false, true, AdditionalOptionEnum::always_print_brackets);

OperatorStructure TermLogicEq::getOpStruct() const
{
   return my_struct;
}

float TermLogicEq::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return evalSimple(varVals, std::equal_to<float>(), getOperands()[0]->eval(varVals, parser, suppress_sideeffects), 1, parser, suppress_sideeffects, true); // Invert if nan.
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermLogicEq::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   lambda_vno lambda = [](asmjit::x86::Compiler& cc, x86::Xmm& x, x86::Xmm& y) {
      xmmCompareTo(cc, x, y);
   };

   return subAssemblyVarnumOps(cc, d, lambda);
}
#endif

TermLogicEq::TermLogicEq(const std::vector<std::shared_ptr<Term>>& opds) : TermLogic(opds)
{
}

vfm::TermLogicEq::TermLogicEq(const std::shared_ptr<Term> opd1, const std::shared_ptr<Term> opd2) : TermLogicEq(std::vector<std::shared_ptr<Term>>{opd1, opd2})
{
}

std::shared_ptr<Term> TermLogicEq::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   return copySimple<TermLogicEq>(already_copied);
}

std::set<MetaRule> vfm::TermLogicEq::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);

   vec.insert(MetaRule(_eq(_vara(), _varb()), _not(_vara()), SIMPLIFICATION_STAGE_MAIN, {{ _varb(), ConditionType::is_constant_and_evaluates_to_false }, { _vara(), ConditionType::contains_no_meta } })); // "a == b", "not(a)" [b is constant...]
   vec.insert(MetaRule(_eq(_varb(), _vara()), _not(_vara()), SIMPLIFICATION_STAGE_MAIN, {{ _varb(), ConditionType::is_constant_and_evaluates_to_false }, { _vara(), ConditionType::contains_no_meta } })); // "b == a", "not(a)" [b is constant...]
   vec.insert(MetaRule(_eq(_vara(), _vara()), _true(), SIMPLIFICATION_STAGE_MAIN, {}, ConditionType::has_no_sideeffects)); // "a == a", "1"
   vec.insert(MetaRule(_eq(_neg(_vara()), _neg(_varb())), _eq(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN));              // "--(a) == --(b)", "a == b"
   
   return vec;
}

std::string TermLogicEq::getNuSMVOperator() const
{
   return "=";
}

std::string vfm::TermLogicEq::getK2Operator() const
{
   return "eq";
}

std::string vfm::TermLogicEq::serializeK2(const std::map<std::string, std::string>& enum_values, std::pair<std::map<std::string, std::string>, std::string>& additional_functions, int& label_counter) const
{
   // TODO: Revert back to plain rem, once native operation available in Kratos.

   if (child0()->getOptor() == SYMB_MOD) {
      if (child1()->isAlwaysFalse()) {
         return _or(_eq(child0()->child0(), _val(0)), _or(_eq(child0()->child0(), _val(2)), _eq(child0()->child0(), _val(4))))->serializeK2(enum_values, additional_functions, label_counter);
      }
      else {
         return _or(_eq(child0()->child0(), _val(1)), _eq(child0()->child0(), _val(3)))->serializeK2(enum_values, additional_functions, label_counter);
      }
   }

   return MathStruct::serializeK2(enum_values, additional_functions, label_counter);
}

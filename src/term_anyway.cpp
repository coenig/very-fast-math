//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_anyway.h"
#include "meta_rule.h"
#include "parser.h"
#include <limits>

using namespace vfm;

const OperatorStructure TermAnyway::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, PARNUM_FLEXIBLE_MODE, PRIO_ANYWAY, SYMB_ANYWAY, false, false);
std::set<MetaRule> TermAnyway::simplifications_{};

OperatorStructure TermAnyway::getOpStruct() const
{
    return my_struct;
}

float TermAnyway::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return -std::numeric_limits<float>::infinity();
}

std::shared_ptr<Term> TermAnyway::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   std::vector<TermPtr> subterms;

   for (const auto& subterm : getOperands()) {
      subterms.push_back(subterm->copy());
   }

   std::shared_ptr<Term> ptr(new TermAnyway(subterms));
   ptr->setChildrensFathers(false, false);
   return ptr;
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermAnyway::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return nullptr;
}
#endif

std::shared_ptr<TermAnyway> vfm::TermAnyway::toTermAnywayIfApplicable()
{
   return std::dynamic_pointer_cast<TermAnyway>(this->shared_from_this());
}

std::set<MetaRule> vfm::TermAnyway::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   auto insert = [this, &parser](const std::string& rule) {
      MetaRule r0{ rule, SIMPLIFICATION_STAGE_PREPARATION, parser }; // Stage 0 since we generate a normal form here which we need for simplification.
      r0.convertVeriablesToMetas();
      simplifications_.insert(r0);

      MetaRule r1 = r0.copy(true, true, true);
      r1.setStage(SIMPLIFICATION_STAGE_MAIN); // Stage 10 since we need a normal form in the end, as well.
      simplifications_.insert(r1);
   };

   if (simplifications_.empty()) {
      // Normal form for all associative and commutative operators...
      // One desired side-effect of the normal form is to shift all constants into a sub-tree which, as a whole, gets replaced by a single number during simplification.

      // TODO: The below rule is not bad, per se, when combined with the two expansion rules in TermMult.
      // However, they srew up other intended simplifications, particularly the one shown as example in the readme. We need to balance these in future.
      //simplifications_.insert(MetaRule("(x ~ y) ~ z ==> x ~ (y ~ z) [[y ~ z: 'is_associative_operation']]", SIMPLIFICATION_STAGE_PREPARATION)); // ...first make right-associative: ... + (. + (. + (...)))
      simplifications_.insert(MetaRule("x ~ (y ~ z) ==> x ~ y ~ z [[y ~ z: 'is_associative_operation']]", SIMPLIFICATION_STAGE_MAIN));  // ...then  make left-associative:  (((...) + .) + .) + ...
      insert("x ~ y ==> y ~ x [[x: 'is_not_constant', y: 'is_constant', x ~ y: 'is_commutative_operation', x ~ y: 'is_associative_operation']] {{parent_is_not_get}}"); // ...put constants to the left: x + 2 ==> 2 + x.
      insert("(b ~ c) ~ a ==> (a ~ c) ~ b [[a: 'is_constant', b: 'is_not_constant', a ~ b: 'is_commutative_operation', a ~ b: 'is_associative_operation']]"); // ...send constants "downstream": (x + .) + 5 ==> (5 + .) + x.
      insert("(b ~ c) ~ a ==> (b ~ a) ~ c [[a: 'is_constant', c: 'is_not_constant', a ~ b: 'is_commutative_operation', a ~ b: 'is_associative_operation']]"); // ...same as before:              (. + y) + 5 ==> (. + 5) + y.

      // TODO: These are good, but screw up the statistics. Feel free to comment in if needed.
      //insert("a ~ b ==> a - b ~ 0 [[a ~ b: 'is_comparator_or_equality_operation', b: 'is_not_constant']]");
      //insert("a ~ b ==> a - b ~ 0 [[a ~ b: 'is_comparator_or_equality_operation', b: 'is_constant_and_evaluates_to_true']]");

      // Order operands of commutative operators alphabetically.
      // TODO: This no work! We need a mechanism to make the 0 the neutral operation of the respective operator.
      //insert("~(~(o_(a, 0), b), c) ==> ~(~(a, c), b) [[b: 'is_variable', c: 'is_variable', b ; c: 'of_first_two_operands_second_is_alphabetically_above', ~(b, c): 'is_commutative_operation']]");
   }

   return simplifications_;
}

void vfm::TermAnyway::serialize(
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
   const bool print_cmp{ special == SerializationSpecial::print_compound_structures || special == SerializationSpecial::enforce_postfix_and_print_compound_structures };

   if (possiblyHighlightAndGoOnSerializing(os, highlight, style, special, indent, indent_step, line_len, data, visited)) {
      return;
   }

   std::string anyway_symb{};
   int anyway_id{ static_cast<int>(const_cast<TermAnyway*>(this)->child0()->constEval()) };

   if (!print_cmp && anyway_id >= 1 && anyway_id < CREATE_SIMPLE_ANYWAYS_UP_TO) {
      for (int i = 0; i < anyway_id; i++) {
         anyway_symb += SYMB_ANYWAY_S;
      }

      os << anyway_symb << OPENING_BRACKET;
      bool first{ true };
      int cnt{ 0 };

      for (const auto& child : const_cast<TermAnyway*>(this)->getOperands()) {
         if (cnt > 0) {
            std::string comma{ first ? "" : ", " };
            os << comma << child->serialize(highlight, style, special, indent, indent_step, line_len, data);
            first = false;
         }

         cnt++;
      }

      os << CLOSING_BRACKET;
   }
   else {
      MathStruct::serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
   }

   visited.insert(shared_from_this());
}

TermAnyway::TermAnyway(const std::vector<std::shared_ptr<Term>> terms) : Term(terms) {}

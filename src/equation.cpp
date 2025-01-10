//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "equation.h"
#include "term.h"
#include "term_meta_compound.h"
#include "term_val.h"
#include "parser.h"
#include "simplification/simplification.h"
#include <vector>
#include <sstream>
#include <iostream>
#include <exception>

using namespace vfm;

const int Equation::default_mod_val = 256;
const OperatorStructure Equation::my_struct(OutputFormatEnum::infix, AssociativityTypeEnum::left, 2, PRIO_EQUATION, SYMB_EQUATION, true, true);

std::shared_ptr<Term> Equation::toTermIfApplicable()
{
   return nullptr; // An Equation is the only MathStruct that is not a Term. This might have been bad design after all, but how else to do it??
}

std::shared_ptr<Equation> Equation::toEquationIfApplicable()
{
   return std::dynamic_pointer_cast<Equation>(this->shared_from_this());
}

float Equation::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
   // An Equation cannot be evaluated.
   std::stringstream ss;
   ss << "An Equation ('" << *this << " (" << typeid(this).name() << ")" << "') cannot be evaluated.";
   throw std::runtime_error(ss.str());
}

float Equation::constEval(const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
   return eval(nullptr, parser, suppress_sideeffects);
}

std::shared_ptr<MathStruct> Equation::copy()
{
   return _equation(getOperands()[0]->copy(), getOperands()[1]->copy());
}

OperatorStructure Equation::getOpStruct() const
{
   return my_struct;
}

void Equation::serialize(
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
   if (special == SerializationSpecial::enforce_postfix || special == SerializationSpecial::enforce_postfix_and_print_compound_structures) {
      os << getOperandsConst()[0]->serialize(SerializationSpecial::enforce_postfix) << POSTFIX_DELIMITER << getOperandsConst()[1]->serialize(SerializationSpecial::enforce_postfix) << POSTFIX_DELIMITER << getOptor();
   } else {
      getOperandsConst()[0]->serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
      os << " " << getOptor() << " ";
      getOperandsConst()[1]->serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
   }

   visited.insert(shared_from_this());
}

bool find(const std::function<bool(std::shared_ptr<MathStruct>)>& match_cond, const std::shared_ptr<Term>& search_in, std::vector<std::shared_ptr<Term>>& trace, std::vector<int>& dirs) {
   if (match_cond(search_in)) {
      trace.push_back(search_in);
      dirs.push_back(-1);
      return true;
   }

   for (std::size_t i = 0; i < search_in->getOperands().size(); ++i) {
      auto t = search_in->getOperands()[i];
      if (find(match_cond, t, trace, dirs)) {
         trace.push_back(search_in);
         dirs.push_back(i);
         return true;
      }
   }

   return false;
}

void Equation::rearrange(const std::shared_ptr<Term>& inv_term, const std::shared_ptr<Term>& other_side, bool from_left) {
   std::vector<std::shared_ptr<Term>> trace{};
   std::vector<int> dirs{};
   bool doit{ true };

   while (doit) {
      trace.clear();
      dirs.clear();
      doit = find([](std::shared_ptr<MathStruct> m) {return m->isMetaCompound(); }, inv_term, trace, dirs);
      if (doit) {
         trace[1]->getOperands()[dirs[1]] = getOperands()[from_left]->copy();
      }
   }

   this->getOperands()[from_left] = inv_term;
   this->getOperands()[!from_left] = other_side;
}

bool Equation::resolveTo(const std::function<bool(std::shared_ptr<MathStruct>)>& match_cond, bool from_left)
{
   getOperands()[0] = simplification::simplifyFast(getOperands()[0]);
   getOperands()[1] = simplification::simplifyFast(getOperands()[1]);

   std::vector<std::shared_ptr<Term>> trace{};
   std::vector<int> dirs{};

   if (!find(match_cond, getOperands()[!from_left], trace, dirs)) {
      return false;
   }

   for (size_t i = dirs.size() - 1; i > 0; i--) {
      auto t{ trace[i] };
      auto d{ dirs[i] };
      try {
         auto inv{ t->getInvPattern(d) };
         rearrange(inv, t->getOperands()[d], from_left);
      }
      catch (const std::exception & ex) {
         setChildrensFathers();
         return false;
      }
   }

   getOperands()[0] = simplification::simplifyFast(getOperands()[0]);
   getOperands()[1] = simplification::simplifyFast(getOperands()[1]);
   setChildrensFathers();

   return true;
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> vfm::Equation::createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return nullptr;
}
#endif

Equation::Equation(const std::shared_ptr<Term>& t1, const std::shared_ptr<Term>& t2) : Equation(t1, t2, std::make_shared<TermVal>(default_mod_val))
{
}

Equation::Equation(const std::shared_ptr<Term>& t1, const std::shared_ptr<Term>& t2, const std::shared_ptr<Term>& mod) 
   : MathStruct(std::vector<std::shared_ptr<Term>>{t1, t2, mod})
{
}

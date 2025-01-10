//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

// #define COMPOUND_DEBUG // (Un-)comment to add/remove debug output.
// #define PRINT_ADDRESSES_IN_GRAPHVIZ // (Un-)comment to add/remove addresses from graph view.

#include "term_compound_operator.h"
#include "term_plus.h"
#include "static_helper.h"
#include "parser.h"
#include <iostream>
#include <map>

using namespace vfm;


OperatorStructure vfm::TermCompoundOperator::getOpStruct() const
{
   return my_struct_;
}

vfm::TermCompoundOperator::TermCompoundOperator(const std::vector<std::shared_ptr<Term>>& opds, const OperatorStructure& operator_structure)
   : Term{ opds }, my_struct_{ operator_structure }
{
}

float vfm::TermCompoundOperator::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
   Failable::getSingleton()->addError("TermCompoundOperator cannot be directly used for evaluation. Use only in scope of TermCompound.");

   return std::numeric_limits<float>::quiet_NaN();
}

float vfm::TermCompoundOperator::constEval(const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
   return eval(nullptr);
}

bool vfm::TermCompoundOperator::resolveTo(const std::function<bool(std::shared_ptr<MathStruct>)>& match_cond, bool from_left)
{
   constEval();
   return false;
}

bool vfm::TermCompoundOperator::isCompoundOperator() const
{
   return true;
}

std::shared_ptr<TermCompoundOperator> vfm::TermCompoundOperator::toCompoundOperatorIfApplicable()
{
   return std::dynamic_pointer_cast<TermCompoundOperator>(shared_from_this());
}

std::shared_ptr<x86::Xmm> vfm::TermCompoundOperator::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return nullptr;
}

std::shared_ptr<Term> vfm::TermCompoundOperator::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied)
{
   auto cp{ std::make_shared<TermCompoundOperator>(std::vector<std::shared_ptr<Term>>{}, my_struct_) };

   for (const auto& op : getOperands()) {
      cp->getOperands().push_back(op->copy());
   }

   return cp;
}

std::string vfm::TermCompoundOperator::getCPPOperator() const
{
   if (getFather()->isTermIfelse()) {
      return SYMB_IF;
   }

   return getOptor();
}

std::string vfm::TermCompoundOperator::getOptorOnCompoundLevel() const
{
   return SYMB_COMP_OP;
}

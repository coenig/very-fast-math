//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

// #define META_DEBUG // (Un-)comment to add/remove debug output.

#include "term_meta_compound.h"
#include "string"
#include <sstream>
#include <exception>

using namespace vfm;

const OperatorStructure TermMetaCompound::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_META_CMP, PRIO_META_CMP, SYMB_META_CMP, false, false);

bool TermMetaCompound::isOverallConstant(const std::shared_ptr<std::vector<int>> non_const_metas,
                                 const std::shared_ptr<std::set<std::shared_ptr<MathStruct>>> visited)
{
   if (!non_const_metas) { // If we don't know part of which compound structure the meta is, we assume non-const.
      return false;
   }

   int arg_num = getOperands()[0]->constEval();
   return std::find(non_const_metas->begin(), non_const_metas->end(), arg_num) == non_const_metas->end();
}

bool vfm::TermMetaCompound::hasSideeffectsThis() const
{
   return true;
}

OperatorStructure TermMetaCompound::getOpStruct() const
{
    return my_struct;
}

float TermMetaCompound::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   auto argnum{ getOperands()[0]->eval(varVals, parser, suppress_sideeffects) };
   auto terms{ varVals->getValuesForMeta() };
   auto term{ terms->at(argnum) };

#if defined(META_DEBUG)
   term->visited_stack_item_++;
   std::cout << "\n(m) ";
   varVals->printMetaStack();
   std::cout << std::endl << "(m) Evaluating argument " << argnum << " '" << *term << "'.";
   this->applyToMeAndMyParents([](std::shared_ptr<MathStruct> term)
   {
      std::cout << std::endl << "   (m) Argnum from: " << term->serializePlainOldVFMStyle() << ".";
   });
   
   if (term->isMeta()) {
      std::cout << std::endl << "ARG IS META: " << term->serializePlainOldVFMStyle() << ".";
   }
   std::cout << std::endl << std::endl << "DATA" << std::endl << *varVals;
#endif

   varVals->popValuesForMeta();

   auto evaluated = term->isTermVal() || term->isTermVar() // TODO: Not sure if this is really as harmless as I believe.
      ? term 
      : std::make_shared<TermVal>((term->eval(varVals, parser, suppress_sideeffects)));

   if (okToUpdateStack) {
      terms->at(argnum) = evaluated;
   }
   
   varVals->pushValuesForMeta(terms);

#if defined(META_DEBUG)
   std::cout << std::endl << "(m) Argument " << argnum << " '" << *term << "' evaluated to: " << evaluated << std::endl;
   std::cout << "\n(m) ";
   varVals->printMetaStack();
   std::cout << std::endl << std::endl << "DATA" << std::endl << *varVals;
#endif

   return evaluated->eval(varVals, parser, suppress_sideeffects);
}

bool TermMetaCompound::isMetaCompound() const {
    return true;
}

std::shared_ptr<Term> TermMetaCompound::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   std::shared_ptr<Term> ptr(new TermMetaCompound(getOperands()[0]->copy(already_copied)));
   ptr->setChildrensFathers(false, false);
   return ptr;
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermMetaCompound::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return nullptr;
}
#endif

std::shared_ptr<TermMetaCompound> TermMetaCompound::toMetaCompoundIfApplicable()
{
    return std::dynamic_pointer_cast<TermMetaCompound>(this->shared_from_this());
}

void TermMetaCompound::prohibitUpdateStack()
{
   okToUpdateStack = false;
}

void TermMetaCompound::allowUpdateStack()
{
   okToUpdateStack = true;
}

std::string vfm::TermMetaCompound::getK2Operator() const
{
   return "assume";
}

TermMetaCompound::TermMetaCompound() : TermMetaCompound(std::make_shared<TermVal>(0)) {}
TermMetaCompound::TermMetaCompound(const std::shared_ptr<Term>& t) : Term(std::vector<std::shared_ptr<Term>>{t}) {}
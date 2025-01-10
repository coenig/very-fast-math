//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

// #define META_DEBUG // (Un-)comment to add/remove debug output.

#include "term_meta_simplification.h"
#include "string"
#include <sstream>
#include <exception>

using namespace vfm;

const OperatorStructure TermMetaSimplification::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_META_SIMP, PRIO_META_SIMP, SYMB_META_SIMP, false, false);

bool TermMetaSimplification::isOverallConstant(const std::shared_ptr<std::vector<int>> non_const_metas,
                                 const std::shared_ptr<std::set<std::shared_ptr<MathStruct>>> visited)
{
   return false;
}

OperatorStructure TermMetaSimplification::getOpStruct() const
{
    return my_struct;
}

float TermMetaSimplification::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
   Failable::getSingleton()->addFatalError("Evaluation not available for 'TermMetaSimplification'.");
   return 0.0f;
}

bool TermMetaSimplification::isMetaSimplification() const {
    return true;
}

std::shared_ptr<Term> TermMetaSimplification::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   std::shared_ptr<Term> ptr(new TermMetaSimplification(getOperands()[0]->copy(already_copied)));
   ptr->setChildrensFathers(false, false);
   return ptr;
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermMetaSimplification::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   Failable::getSingleton()->addFatalError("ASMJIT not available for 'TermMetaSimplification'.");
   return nullptr;
}
#endif

std::shared_ptr<TermMetaSimplification> TermMetaSimplification::toMetaSimplificationIfApplicable()
{
    return std::dynamic_pointer_cast<TermMetaSimplification>(this->shared_from_this());
}

std::string vfm::TermMetaSimplification::getK2Operator() const
{
   Failable::getSingleton()->addFatalError("K2 operator not available for 'TermMetaSimplification'.");
   return "#ERROR";
}

TermMetaSimplification::TermMetaSimplification() : TermMetaSimplification(std::make_shared<TermVal>(0)) {}
TermMetaSimplification::TermMetaSimplification(const std::shared_ptr<Term>& t) : Term(std::vector<std::shared_ptr<Term>>{t}) {}
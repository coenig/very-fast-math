//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_literal.h"
#include "term_compound.h"

using namespace vfm;

const OperatorStructure TermLiteral::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_LITERAL, PRIO_LITERAL, SYMB_LITERAL, false, false);

TermLiteral::TermLiteral(const std::vector<std::shared_ptr<Term>>& ts) : Term(ts) {}
TermLiteral::TermLiteral(const std::shared_ptr<Term> t) : Term({ t }) {}

OperatorStructure TermLiteral::getOpStruct() const
{
   return my_struct;
}

float TermLiteral::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   auto formula{ getOperands()[0]->isMetaCompound()
      ? (*varVals->getValuesForMeta())[getOperands()[0]->child0()->constEval()]
      : getOperands()[0]->copy() };

   formula->applyToMeAndMyChildrenIterative([varVals, parser, suppress_sideeffects](const MathStructPtr m) {
      if (m->isTermIdentity()) { // Arrived at "id".
         m->replace(_val(m->eval(varVals, parser, suppress_sideeffects)));
      }
   }, TraverseCompoundsType::avoid_compound_structures);

   static constexpr int LEFT = 40;
   static constexpr int RIGHT = 5;

   std::string ser = getOperands()[0]->isMetaCompound()
      ? (formula->getFather() ? formula->getFather()->serializeWithinSurroundingFormula(LEFT, RIGHT) : formula->serializeWithinSurroundingFormula(LEFT, RIGHT))
      : formula->serialize();

   ser.push_back('\0');
   int address = varVals->reserveHeap(ser.size());

   for (int i = 0; i < ser.size(); i++) {
      varVals->setHeapLocation(address + i, (float) ser[i]);
   }

   return address;
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermLiteral::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return nullptr;
}
#endif

std::shared_ptr<Term> TermLiteral::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   return copySimple<TermLiteral>(already_copied);
}

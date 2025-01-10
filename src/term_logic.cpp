//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_logic.h"

using namespace vfm;

#if defined(ASMJIT_ENABLED)
void TermLogic::xmmCompareTo(asmjit::x86::Compiler& cc, x86::Xmm& x, const float& y, const CompFunction& comp_type, const float& true_val, const float& false_val)
{
   x86::Xmm val = cc.newXmm();
   setXmmVar(cc, val, y);
   xmmCompareTo(cc, x, val, comp_type);
}

void TermLogic::xmmCompareTo(asmjit::x86::Compiler & cc, x86::Xmm & x, x86::Xmm & y, const CompFunction& comp_type, const float& true_val, const float& false_val)
{
   asmjit::Label true_label = cc.newLabel();
   asmjit::Label exit_label = cc.newLabel();

   cc.ucomiss(x, y);
   comp_type(cc, true_label);     // Conditional jump to true.
   setXmmVar(cc, x, false_val);
   cc.jmp(exit_label);            // Jump to exit.

   cc.bind(true_label);           // LABEL TRUE.
   setXmmVar(cc, x, true_val);

   cc.bind(exit_label);           // LABEL EXIT.
}
#endif

bool vfm::TermLogic::hasBooleanResult() const
{
   return true;
}

bool vfm::TermLogic::isTermLogic() const
{
   return true;
}

TermLogic::TermLogic(const std::vector<std::shared_ptr<Term>>& opds) : Term(opds)
{
}

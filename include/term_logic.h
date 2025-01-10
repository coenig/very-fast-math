//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term.h"


namespace vfm {

// From: http://rayseyfarth.blogspot.com/2011/08/no-one-should-compare-floating-point.html

// For less than    : CF IF
// For greater than : IF
// For equal        : ZF

// jb     CF = 1              cmp_less
// jbe    CF = 1 or ZF = 1    cmp_less_equal
// ja     CF = 0 and ZF = 0   cmp_greater
// jae    CF = 0              cmp_greater_equal
// je     ZF = 1
// jne    ZF = 0

#if defined(ASMJIT_ENABLED)
typedef std::function<void(asmjit::x86::Compiler& cc, asmjit::Label)> CompFunction;
const CompFunction cmp_equal = [](asmjit::x86::Compiler& cc, asmjit::Label l)         {cc.je(l); };
const CompFunction cmp_not_equal = [](asmjit::x86::Compiler& cc, asmjit::Label l)     {cc.jne(l); };
const CompFunction cmp_less = [](asmjit::x86::Compiler& cc, asmjit::Label l)          {cc.jb(l); };
const CompFunction cmp_less_equal = [](asmjit::x86::Compiler& cc, asmjit::Label l)    {cc.jbe(l); };
const CompFunction cmp_greater = [](asmjit::x86::Compiler& cc, asmjit::Label l)       {cc.ja(l); };
const CompFunction cmp_greater_equal = [](asmjit::x86::Compiler& cc, asmjit::Label l) {cc.jae(l); };
#endif

class TermLogic :
   public Term
{
public:
#if defined(ASMJIT_ENABLED)
   static void xmmCompareTo(asmjit::x86::Compiler& cc, x86::Xmm& x, const float& y, const CompFunction& comp_type = cmp_equal, const float& true_val = 1, const float& false_val = 0);
   static void xmmCompareTo(asmjit::x86::Compiler& cc, x86::Xmm& x, x86::Xmm& y, const CompFunction& comp_type = cmp_equal, const float& true_val = 1, const float& false_val = 0);
#endif

   bool hasBooleanResult() const override;
   bool isTermLogic() const override;

   TermLogic(const std::vector<std::shared_ptr<Term>>& opds);
};

} // vfm
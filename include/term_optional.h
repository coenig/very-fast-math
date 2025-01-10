//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term.h"
#include "term_val.h"


namespace vfm {

/// TermOptional is currently used in combination with simplification metas only. It marks a meta
/// as being optional, and comes with a default value in case the meta is NOT there. 
/// For example the meta rule with optionals:
///                    "p_(0) * o_(p_(1), 1) + p_(0) * o_(p_(2), 1)" ==> "p_(0) * (p_(1) + p_(2))"
/// which represents   "a * b + a * c" ==> "a * (b + c)"
/// can be applied not only to
///   "x * y + x * z"
/// but also to
///   "x + x * z" ==> "x * (1 + z)"
///   "x * y + x" ==> "x * (y + 1)"
///   "x + x"     ==> "x * (1 + 1)" (==> "2 * x")
/// where the now missing metas "y"/"z" are treated as 1. This comes with some restrictions, and
/// has to be used carefully since it might not make sense in all circumstances.
/// Restrictions:
/// * The first operand "opnd[0]" has to be a meta (or a variable which is auto-translated into a meta later).
/// * Optionals are allowed only as first or second operand of a 2-operand term.
///   (Usually, the second operand should be optional only if the term is commutative;
///   or really think about it when doing something that unusual.)
/// * Only one of the children of a single term can be an optional.
/// Note that not all of these restrictions are enforced, but they need to be obeyed by the user.
class TermOptional :
   public Term
{
public:
   const static OperatorStructure my_struct;
   OperatorStructure getOpStruct() const override;

   float eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser = nullptr, const bool suppress_sideeffects = false) override;
   std::shared_ptr<Term> copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied = nullptr) override;
#if defined(ASMJIT_ENABLED)
   std::shared_ptr<x86::Xmm> createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p = nullptr) override;
#endif
   bool isTermOptional() const override;
   std::shared_ptr<TermOptional> toTermOptionalIfApplicable() override;

   TermOptional(const std::shared_ptr<Term>& meta, const std::shared_ptr<Term>& replacement_value);
};

} // vfm

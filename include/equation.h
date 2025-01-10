//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "math_struct.h"


namespace vfm {

class Equation :
   public MathStruct
{
private:
   static const int default_mod_val;
public:
   const static OperatorStructure my_struct;

   std::shared_ptr<Term> toTermIfApplicable() override;
   std::shared_ptr<Equation> toEquationIfApplicable() override;
   float eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser = nullptr, const bool suppress_sideeffects = false) override;
   float constEval(const std::shared_ptr<FormulaParser> parser = nullptr, const bool suppress_sideeffects = false) override;
   virtual std::shared_ptr<MathStruct> copy();
   virtual OperatorStructure getOpStruct() const;

   void serialize(
      std::stringstream& os,
      const std::shared_ptr<MathStruct> highlight, 
      const SerializationStyle style,
      const SerializationSpecial special,
      const int indent,
      const int indent_step,
      const int line_len,
      const std::shared_ptr<DataPack> data,
      std::set<std::shared_ptr<const MathStruct>>& visited) const override;

   void rearrange(const std::shared_ptr<Term>& inv_term, const std::shared_ptr<Term>& other_side, bool from_left);
   bool resolveTo(const std::function<bool(std::shared_ptr<MathStruct>)>& match_cond, bool from_left) override;
   
#if defined(ASMJIT_ENABLED)
   std::shared_ptr<x86::Xmm> createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p = nullptr) override;
#endif


   Equation(const std::shared_ptr<Term>& t1, const std::shared_ptr<Term>& t2);
   Equation(const std::shared_ptr<Term>& t1, const std::shared_ptr<Term>& t2, const std::shared_ptr<Term>& mod);
};

} // vfm
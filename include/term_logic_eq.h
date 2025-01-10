//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term_logic.h"


namespace vfm {

class TermLogicEq :
   public TermLogic
{
public:
   const static OperatorStructure my_struct;

   OperatorStructure getOpStruct() const override;
   float eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser = nullptr, const bool suppress_sideeffects = false) override;
#if defined(ASMJIT_ENABLED)
   std::shared_ptr<x86::Xmm> createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p = nullptr) override;
#endif
   std::shared_ptr<Term> copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied = nullptr) override;
   std::set<MetaRule> getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const override;
   std::string getNuSMVOperator() const override;
   std::string getK2Operator() const override;

   /// <summary>
   /// TODO: JUST FOR TESTING. HAS TO GO AWAY AGAIN!
   /// </summary>
   /// <param name="enum_values"></param>
   /// <param name="additional_functions"></param>
   /// <param name="label_counter"></param>
   /// <returns></returns>
   std::string serializeK2(const std::map<std::string, std::string>& enum_values, std::pair<std::map<std::string, std::string>, std::string>& additional_functions, int& label_counter) const override;

   TermLogicEq(const std::vector<std::shared_ptr<Term>>& opds);
   TermLogicEq(const std::shared_ptr<Term> opd1, const std::shared_ptr<Term> opd2);
};

} // vfm
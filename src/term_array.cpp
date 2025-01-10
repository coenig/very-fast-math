//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_array.h"
#include "data_pack.h"
#include <sstream>
#include <string>
#include <vector>

using namespace vfm;

bool TermArray::isOverallConstant(const std::shared_ptr<std::vector<int>> non_const_metas,
                                  const std::shared_ptr<std::set<std::shared_ptr<MathStruct>>> visited)
{
   return false;
}

OperatorStructure TermArray::getOpStruct() const
{
   return my_struct;
}

OperatorStructure TermArray::getOpStruct(const std::string& op_name) {
   return OperatorStructure(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_ARR, PRIO_ARRAY, op_name, true, true);
}

float TermArray::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return varVals->getValFromArray(getOptor(), static_cast<int>(getOperands()[0]->eval(varVals, parser, suppress_sideeffects)));
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermArray::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   MathStruct::parseMathStruct(getOptor() + OPENING_BRACKET + "0" + CLOSING_BRACKET, p)->eval(d, p, true); // Dummy access to force array registration. TODO: Add functions for that in DataPack.

   const char* char_array_address = d->getAddressOfArray(getOptor());
   const float* float_array_address = d->getAddressOfFloatArray(getOptor());

   // Note that there are currently no bool arrays in the "static array world".

   if (char_array_address) {
      lambda_so lambda = [char_array_address](asmjit::x86::Compiler& cc, x86::Xmm& x) {
         x86::Xmm y = cc.newXmm();
         cc.movss(y, x);
         setXmmVarToAddressLocation(
            cc,
            x,
            char_array_address,
            y,
            sizeof(char));
      };

      return subAssemblySingleOp(cc, d, lambda);
   }
   else if (float_array_address) {
      lambda_so lambda = [float_array_address](asmjit::x86::Compiler& cc, x86::Xmm& x) {
         x86::Xmm y = cc.newXmm();
         cc.movss(y, x);
         setXmmVarToAddressLocation(
            cc,
            x,
            float_array_address,
            y);
      };

      return subAssemblySingleOp(cc, d, lambda);
   }

   //addNote("Address not available for assembly generation of array '" + getOptor() + "'. Interpreter evaluation will be used for subtree '" + serializeWithinSurroundingFormula() + "'.");
   return nullptr;
}
#endif

TermArray::TermArray(const std::shared_ptr<Term>& opd, const std::string& opt)
   : Term(std::vector<std::shared_ptr<Term>>{opd}), my_struct(getOpStruct(opt))
{
}

std::shared_ptr<Term> TermArray::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   std::shared_ptr<Term> ptr(new TermArray(getOperands()[0]->copy(already_copied), this->getOptor()));
   ptr->setChildrensFathers(false, false);
   return ptr;
}

bool TermArray::isTermArray() const
{
   return true;
}

std::shared_ptr<TermArray> TermArray::toArrayIfApplicable()
{
   return std::dynamic_pointer_cast<TermArray>(this->shared_from_this());
}

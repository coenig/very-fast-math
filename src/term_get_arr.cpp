//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_get_arr.h"
#include "term_array.h"
#include "static_helper.h"
#include "parser.h"

using namespace vfm;

const OperatorStructure TermGetArr::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_GET_ARR, PRIO_GET_ARR, SYMB_GET_ARR, false, false);

TermGetArr::TermGetArr(const std::vector<std::shared_ptr<Term>>& vt) : Term(vt) {}
TermGetArr::TermGetArr(const std::shared_ptr<Term> var, const std::shared_ptr<Term> idx) : TermGetArr(std::vector<std::shared_ptr<Term>>{var, idx}) {}
TermGetArr::TermGetArr(const std::string& var_name, const std::shared_ptr<Term> idx, const std::shared_ptr<DataPack> d) : TermGetArr(getArrAddress(var_name, d), idx) {}

bool vfm::TermGetArr::isTermGetArr() const
{
   return true;
}

std::shared_ptr<TermGetArr> vfm::TermGetArr::toGetArrIfApplicable()
{
   return std::dynamic_pointer_cast<TermGetArr>(this->shared_from_this());
}

OperatorStructure TermGetArr::getOpStruct() const
{
   return TermGetArr::my_struct;
}

float TermGetArr::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   int address = getOperands()[0]->eval(varVals, parser, suppress_sideeffects);
   int index = getOperands()[1]->eval(varVals, parser, suppress_sideeffects);

   return varVals->getArrayVal(address, index);
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermGetArr::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   bool contains_meta = false;
   auto break_ptr = std::make_shared<bool>(false);

   applyToMeAndMyChildren([&contains_meta, break_ptr](const MathStructPtr m) {
      if (m->isMetaCompound()) {
         contains_meta = true;
         *break_ptr = true;
      }
   }, TraverseCompoundsType::avoid_compound_structures, nullptr, break_ptr);

   if (contains_meta) {
      return nullptr;
   }

   float optor = getOperands()[0]->eval(d, p, true);

   if (d->isStaticArray(optor)) {
      return TermArray(getOperands()[1], d->getNameOf(optor)).createSubAssembly(cc, d, p);
   }
#ifndef DISALLOW_EXTERNAL_ASSOCIATIONS
   else {
      d->precalculateExternalAddresses();
      auto references = (float**) d->getPrecalculatedExternalAddresses().data(); // Desired value is at address pointed to by references[opnds[0] + opnds[1]].

      lambda_vno lambda = [&references](asmjit::x86::Compiler& cc, x86::Xmm& x, x86::Xmm& y) {
         x86::Xmm index = cc.newXmm();
         cc.pxor(index, index);
         cc.addss(index, x);
         cc.addss(index, y);

         setXmmVarToValueAtAddressFromArray(
            cc,
            x,
            references,
            index);
      };

      return subAssemblyVarnumOps(cc, d, lambda);
   }
#endif

   return nullptr; // Only possible if no external associations.
}
#endif

std::shared_ptr<Term> TermGetArr::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   std::shared_ptr<Term> ptr(new TermGetArr(getOperands()[0]->copy(already_copied), getOperands()[1]->copy(already_copied)));
   ptr->setChildrensFathers(false, false);
   return ptr;
}

bool vfm::TermGetArr::hasSideeffectsThis() const
{
   return false;
}

bool vfm::TermGetArr::isOverallConstant(
   const std::shared_ptr<std::vector<int>> non_const_metas,
   const std::shared_ptr<std::set<std::shared_ptr<MathStruct>>> visited)
{
   return false;
}

std::string vfm::TermGetArr::getArrName(const std::shared_ptr<DataPack> d, const std::shared_ptr<FormulaParser> parser) const
{
   return d->getNameOf(getOperandsConst().at(0)->eval(d, parser, true));
}

void vfm::TermGetArr::changeArrToSet(const std::string& var_name, const std::shared_ptr<DataPack> d)
{
   getOperands()[0] = _val(d->getAddressOf(var_name));
}

std::shared_ptr<Term> TermGetArr::getArrAddress(const std::string& var_name, const std::shared_ptr<DataPack> d)
{
   return _val(d->getAddressOf(var_name));
}

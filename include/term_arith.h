//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term.h"
#include "term_meta_compound.h"
#include "term_val.h"
#include "data_pack.h"
#include <functional>


namespace vfm {

class TermArith :
   public Term
{
public:
   template<class T>
   std::shared_ptr<Term> getInvPatternSimplePlusStyle(int argNum);
   template<class T, class U>
   std::shared_ptr<Term> getInvPatternSimpleMinusStyle(int argNum);
   TermArith(const std::vector<std::shared_ptr<Term>>& opds);
};

template<class T>
std::shared_ptr<Term> TermArith::getInvPatternSimplePlusStyle(int argNum) {
   std::vector<std::shared_ptr<Term>> terms;
   terms.reserve(getOperands().size());
   terms.push_back(std::shared_ptr<Term>{new TermMetaCompound()});

   for (std::size_t i = 0; i < getOperands().size(); ++i) {
      if (i != argNum) {
         terms.push_back(getOperands()[i]);
      }
   }

   return std::shared_ptr<Term>(new T(terms));
}

template<class T, class U>
std::shared_ptr<Term> TermArith::getInvPatternSimpleMinusStyle(int argNum) {
   std::vector<std::shared_ptr<Term>> terms{};
   terms.reserve(getOperands().size());

   if (argNum != 0) {
      terms.push_back(getOperands()[0]);
   }

   terms.push_back(std::shared_ptr<Term>{new TermMetaCompound()});

   for (std::size_t i = 0; i < getOperands().size(); ++i) {
      if (i != argNum && (argNum == 0 || i > 0)) {
         terms.push_back(getOperands()[i]);
      }
   }

   if (argNum == 0) {
      return std::shared_ptr<Term>(new T(terms));
   }
   else {
      return std::shared_ptr<Term>(new U(terms));
   }
}

} // vfm
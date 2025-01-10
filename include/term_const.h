//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term.h"
#include "string"


namespace vfm {

class TermConst :
   public Term
{
public:
   virtual OperatorStructure getOpStruct() const override;
   TermConst(const std::string& opt);

private:
   const OperatorStructure my_struct;
};

} // vfm
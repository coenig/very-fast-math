//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_const.h"

using namespace vfm;

TermConst::TermConst(const std::string& opt) : 
   Term(std::vector<std::shared_ptr<Term>>()), my_struct(OutputFormatEnum::plain, AssociativityTypeEnum::left, 0, 1000, opt, true, true)
{
}

OperatorStructure TermConst::getOpStruct() const
{
   return my_struct;
}

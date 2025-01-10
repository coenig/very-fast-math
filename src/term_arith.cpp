//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_arith.h"
#include <vector>

using namespace vfm;

TermArith::TermArith(const std::vector<std::shared_ptr<Term>>& opds) : Term(opds)
{
}

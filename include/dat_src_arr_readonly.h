//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "dat_src_arr.h"


namespace vfm {

class DataSrcArrayReadonly :
   public DataSrcArray
{
public:
   void set(int index, float val);
   DataSrcArrayReadonly();
};

} // vfm
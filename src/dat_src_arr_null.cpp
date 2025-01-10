//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "dat_src_arr_null.h"

using namespace vfm;

void DataSrcArrayNull::set(int index, float val)
{
}

float DataSrcArrayNull::get(int index) const
{
   return 0.0;
}

float DataSrcArrayNull::size() const
{
   return 0;
}

const char * DataSrcArrayNull::getAddressOfArray()
{
   return nullptr;
}

float * DataSrcArrayNull::getAddressOfArraySize()
{
   return nullptr;
}

DataSrcArrayNull::DataSrcArrayNull()
{
}

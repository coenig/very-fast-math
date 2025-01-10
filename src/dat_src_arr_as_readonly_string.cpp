//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "dat_src_arr_as_readonly_string.h"
#include <exception>
#include <sstream>

using namespace vfm;

float DataSrcArrayAsReadonlyString::get(int index) const
{
   return data[index];
}

float DataSrcArrayAsReadonlyString::size() const
{
   return arr_size;
}

const char* DataSrcArrayAsReadonlyString::getAddressOfArray()
{
   return data.data();
}

float * DataSrcArrayAsReadonlyString::getAddressOfArraySize()
{
   return &arr_size;
}

DataSrcArrayAsReadonlyString::DataSrcArrayAsReadonlyString(const std::string& data): data(data), arr_size(data.size())
{
}

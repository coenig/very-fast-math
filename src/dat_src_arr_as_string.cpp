//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "dat_src_arr_as_string.h"
#include "data_pack.h"

using namespace vfm;

float DataSrcArrayAsString::get(int index) const
{
   if (index < 0 || index >= size()) {
      return 0.0; // TODO (AI)
   }

   return data[index];
}

void DataSrcArrayAsString::set(int index, float val)
{
   size_t ind = (size_t)index;
   if (index < 0) return;
   data.reserve(ind + 1);
   while (data.size() <= ind) data.push_back(0);
   data[index] = static_cast<int>(val) % 256;
   arr_size = data.size();
}

float DataSrcArrayAsString::size() const
{
   return arr_size;
}

const char* DataSrcArrayAsString::getAddressOfArray()
{
   return data.data();
}

float * DataSrcArrayAsString::getAddressOfArraySize()
{
   return &arr_size;
}

DataSrcArrayAsString::DataSrcArrayAsString(const int & intial_capacity): arr_size(0)
{
   data.reserve(intial_capacity);
}

DataSrcArrayAsString::DataSrcArrayAsString(const std::string & str): data(str.begin(), str.end()), arr_size(data.size())
{
   data.reserve(INITIAL_ARRAY_CAPACITY); // TODO: Find solution for avoiding reallocation issues on array expansion.
}

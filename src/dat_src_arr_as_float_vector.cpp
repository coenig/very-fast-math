//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "dat_src_arr_as_float_vector.h"
#include "data_pack.h"

using namespace vfm;

float DataSrcArrayAsFloatVector::get(int index) const
{
   if (index < 0 || index >= size()) {
      return 0.0; // TODO (AI)
   }

   return data[index];
}

void DataSrcArrayAsFloatVector::set(int index, float val)
{
   size_t ind = (size_t) index;
    data.reserve(ind + 1);
   while (data.size() <= ind) data.push_back(0);
   data[index] = val;
   arr_size = data.size();
}

float DataSrcArrayAsFloatVector::size() const
{
   return arr_size;
}

const char* DataSrcArrayAsFloatVector::getAddressOfArray()
{
    return nullptr; // We have no char array, so no pointer can be returned here.
}

const float* DataSrcArrayAsFloatVector::getAddressOfFloatArray()
{
   return data.data();
}

float* DataSrcArrayAsFloatVector::getAddressOfArraySize()
{
   return &arr_size;
}

DataSrcArrayAsFloatVector::DataSrcArrayAsFloatVector(const int initial_capacity): arr_size(0)
{
   data.reserve(initial_capacity);
}

DataSrcArrayAsFloatVector::DataSrcArrayAsFloatVector(const std::vector<float>& raw_data): data(raw_data.begin(), raw_data.end()), arr_size(data.size())
{
   data.reserve(INITIAL_ARRAY_CAPACITY); // TODO: Find solution for avoiding reallocation issues on array expansion.
}

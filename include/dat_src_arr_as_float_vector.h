//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "dat_src_arr.h"
#include <vector>


namespace vfm {

class DataSrcArrayAsFloatVector :
   public DataSrcArray
{
private:
   std::vector<float> data;
   float arr_size;

public:
   virtual float get(int index) const;
   virtual void set(int index, float val);
   virtual float size() const;
   virtual const float* getAddressOfFloatArray();
   virtual const char* getAddressOfArray();
   virtual float* getAddressOfArraySize();

   DataSrcArrayAsFloatVector(const std::vector<float>& raw_data);
   DataSrcArrayAsFloatVector(const int initial_capacity);
};

} // vfm
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

class DataSrcArrayAsString :
   public DataSrcArray
{
private:
   std::vector<char> data;
   float arr_size;

public:
   virtual float get(int index) const;
   virtual void set(int index, float val);
   virtual float size() const;
   const char* getAddressOfArray();
   virtual float* getAddressOfArraySize();

   DataSrcArrayAsString(const std::string&);
   DataSrcArrayAsString(const int& intial_capacity);
};

} // vfm
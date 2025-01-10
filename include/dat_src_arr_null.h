//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "dat_src_arr.h"


namespace vfm {

class DataSrcArrayNull :
   public DataSrcArray
{
public:
   virtual void set(int index, float val);
   virtual float get(int index) const;
   virtual float size() const;
   const char* getAddressOfArray();
   virtual float* getAddressOfArraySize();

   DataSrcArrayNull();
};

} // vfm
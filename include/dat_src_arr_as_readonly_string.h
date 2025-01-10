//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "dat_src_arr_readonly.h"
#include <string>
#include <vector>

namespace vfm {

class DataSrcArrayAsReadonlyString :
   public DataSrcArrayReadonly
{
private:
   const std::string data;
   float arr_size;

public:
   virtual float get(int index) const;
   virtual float size() const;
   const char* getAddressOfArray();
   virtual float* getAddressOfArraySize();

   DataSrcArrayAsReadonlyString(const std::string&);
};

} // vfm
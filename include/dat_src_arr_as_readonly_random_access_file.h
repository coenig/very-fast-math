//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "dat_src_arr_readonly.h"
#include <fstream>

namespace vfm {

class DataSrcArrayAsReadonlyRandomAccessFile :
   public DataSrcArrayReadonly
{
private:
   mutable std::ifstream file;
   float file_size;
public:
   virtual float get(int index) const;
   virtual float size() const;
   const char* getAddressOfArray();
   virtual float* getAddressOfArraySize();

   DataSrcArrayAsReadonlyRandomAccessFile(const std::string& in_path);
};

} // vfm
//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "dat_src_arr.h"
#include <fstream>

namespace vfm {

class DataSrcArrayAsRandomAccessFile :
   public DataSrcArray
{
private:
   mutable std::fstream file;
   float file_size;

public:
   virtual void set(int index, float val);
   virtual float get(int index) const;
   virtual float size() const;
   const char* getAddressOfArray();
   virtual float* getAddressOfArraySize();

   DataSrcArrayAsRandomAccessFile(const std::string& in_path, const bool& delete_file_contents);
};

} // vfm
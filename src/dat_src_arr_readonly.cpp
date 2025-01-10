//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "dat_src_arr_readonly.h"

using namespace vfm;

void DataSrcArrayReadonly::set(int index, float val)
{
   std::cout << std::endl << "Readonly data source cannot be altered at position '"
      << index
      << "' to value '"
      << val
      << "'. No change performed, and no harm done. (For input files use rw option "
      << " if you really must.)";
}

DataSrcArrayReadonly::DataSrcArrayReadonly()
{
}

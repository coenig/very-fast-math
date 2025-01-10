//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "dat_src_arr_as_readonly_random_access_file.h"
#include <exception>
#include <sstream>

using namespace vfm;

float DataSrcArrayAsReadonlyRandomAccessFile::get(int index) const
{
   if (index < 0 || index >= file_size) return 0;
   file.seekg(index, std::ios::beg);
   auto res{ file.get() };
   return res;
}

float DataSrcArrayAsReadonlyRandomAccessFile::size() const
{
   return file_size;
}

const char* DataSrcArrayAsReadonlyRandomAccessFile::getAddressOfArray()
{
   return nullptr; // TODO...
}

float* DataSrcArrayAsReadonlyRandomAccessFile::getAddressOfArraySize()
{
   return &file_size;
}

DataSrcArrayAsReadonlyRandomAccessFile::DataSrcArrayAsReadonlyRandomAccessFile(const std::string& in_path) : file(in_path, std::ifstream::binary)
{
   std::ifstream f(in_path, std::ios::binary | std::ios::ate);
   file_size = static_cast<int>(f.tellg());
}

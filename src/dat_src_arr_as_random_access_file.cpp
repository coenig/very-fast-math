//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "dat_src_arr_as_random_access_file.h"
#include <exception>
#include <sstream>
#include <sys/stat.h>

using namespace vfm;

void DataSrcArrayAsRandomAccessFile::set(int index, float val)
{
   if (index < 0) return;

   bool extended_file = index >= file_size;

   const char c = static_cast<int>(val);

   while (index >= file_size) {
      file.seekp(0, std::ios::end);
      file << 0;
      file_size++;
   }

   file.seekp(index, std::ios::beg);
   file << c;

   if (extended_file) {
      file_size = static_cast<int>(file.tellg());
   }
}

float DataSrcArrayAsRandomAccessFile::get(int index) const
{
   if (index < 0 || index >= file_size) return 0;
   file.seekg(index, std::ios::beg);
   return file.get();
}

float DataSrcArrayAsRandomAccessFile::size() const
{
   return file_size;
}

const char* DataSrcArrayAsRandomAccessFile::getAddressOfArray()
{
   return nullptr; // TODO...
}

float* DataSrcArrayAsRandomAccessFile::getAddressOfArraySize()
{
   return &file_size;
}

DataSrcArrayAsRandomAccessFile::DataSrcArrayAsRandomAccessFile(const std::string& in_path, const bool& delete_file_contents)
{
   struct stat buffer;
   if (stat(in_path.c_str(), &buffer) != 0) {
      std::fstream fs;
      fs.open(in_path, std::ios::out);
      fs.close();
   }

   if (delete_file_contents) file = std::fstream(in_path, std::ios::binary | std::ios::in | std::ios::out | std::ios_base::ate | std::ofstream::trunc);
   else file = std::fstream(in_path, std::ios::binary | std::ios::in | std::ios::out | std::ios_base::ate);
   file_size = static_cast<int>(file.tellg());
}

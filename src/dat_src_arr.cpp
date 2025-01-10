//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "dat_src_arr.h"
#include <sstream>
#include <cctype>

using namespace vfm;

std::string DataSrcArray::toPlainString()
{
   std::stringstream ss;
   for (int i = 0; i < size(); i++) {
      ss << (char) get(i);
   }
   return ss.str();
}


std::string DataSrcArray::to_string()
{
   std::stringstream ss;
   ss << *this;
   return ss.str();
}

const float* DataSrcArray::getAddressOfFloatArray()
{
    return nullptr;
}

DataSrcArray::DataSrcArray()
{
}

bool my_isprint(char ch)
{
    return std::isprint(static_cast<unsigned char>(ch));
}

std::ostream& operator<<(std::ostream &os, const DataSrcArray &m) {
/*   os << "\"";
    int unprintable = -1;

    for (int i = 0; i < m.size(); i++) {
        if (my_isprint((char) m.get(i))) {
            os << (char) m.get(i);
        }
        else {
            os << (char)(i % 10 + 48);
            unprintable = i;
        }
    }

   os << "\"";
    if (unprintable >= 0) os << " (contains unprintable charaters, last at index " << unprintable << ")";*/

   os << " [";
   if (m.size() > 0) os << m.get(0);
   for (int i = 1; i < m.size(); i++) os << ", " << m.get(i);
   os << "]";

   return os;
}

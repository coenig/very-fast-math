//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include <iostream>


namespace vfm { class DataSrcArray; }
std::ostream& operator<<(std::ostream &os, const vfm::DataSrcArray &m);

namespace vfm {

/// Usually a char array, but can be implemented as raw float as well.
/// Floats are used throughout the math_structs for calculation,
/// but the original array usage has been char due to the roots of the project
/// coming from cryptography. 
class DataSrcArray
{
public:
   friend std::ostream& ::operator<<(std::ostream &os, const DataSrcArray &m);

   std::string to_string();
   virtual std::string toPlainString(); /// Usually not necessary to override except for float arrays.
   virtual void set(int index, float val) = 0;
   virtual float get(int index) const = 0;
   virtual float size() const = 0;

   // Assembly stuff.
   virtual const float* getAddressOfFloatArray(); /// Only for actual float arrays, returns nullptr for char arrays.
   virtual const char* getAddressOfArray() = 0; /// Should return nullptr for float arrays.
   virtual float* getAddressOfArraySize() = 0;
   // EO Assembly stuff.

   DataSrcArray();
};

using ::operator<<;

} // vfm

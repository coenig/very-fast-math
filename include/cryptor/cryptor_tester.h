//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include <utility>
#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <memory>


namespace vfm {

class MathStruct;
class DataPack;

class CryptorTester
{
public:
   static void editor(int argc, char* argv[]);
   //static float testJitStatic();
   //static void encryptor(int argc, const char* argv[]);
   //static void testJitBenchmark(const std::shared_ptr<DataPack> d, std::shared_ptr<MathStruct>& m, int& num);
   //static void justATest();

   CryptorTester();
};

} // vfm

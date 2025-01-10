//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "failable.h"
#include "enum_wrapper.h"
#include <regex>
#include <vector>


namespace vfm { class CryptorStarter; }
std::ostream& operator<<(std::ostream &os, const vfm::CryptorStarter &m);

namespace vfm {

enum class AutoModeEnum { auto_create_always, auto_create_if_missing, auto_create_never };
const std::map<AutoModeEnum, std::string> autoModeEnumMap
{
   { AutoModeEnum::auto_create_always, "auto_create_always" },
   { AutoModeEnum::auto_create_if_missing, "auto_create_if_missing" },
   { AutoModeEnum::auto_create_never, "auto_create_never" },
};

class AutoMode : public vfm::fsm::EnumWrapper<AutoModeEnum>
{
public:
   explicit AutoMode(const AutoModeEnum& enum_val) : EnumWrapper("AutoMode", autoModeEnumMap, enum_val) {}
   explicit AutoMode(const int enum_val_as_num) : EnumWrapper("AutoMode", autoModeEnumMap, enum_val_as_num) {}
   explicit AutoMode(const std::string& enum_as_string) : EnumWrapper("AutoMode", autoModeEnumMap, enum_as_string) {}
   explicit AutoMode() : EnumWrapper("AutoMode", autoModeEnumMap, AutoModeEnum::auto_create_if_missing) {}
};

enum class CryptingModeEnum { enc, dec, tester, editor };
const std::map<CryptingModeEnum, std::string> cryptingModeEnumMap
{
   { CryptingModeEnum::enc, "enc" },
   { CryptingModeEnum::dec, "dec" },
   { CryptingModeEnum::tester, "tester" },
   { CryptingModeEnum::editor, "editor" },
};

class CryptingMode : public vfm::fsm::EnumWrapper<CryptingModeEnum>
{
public:
   explicit CryptingMode(const CryptingModeEnum& enum_val) : EnumWrapper("CryptingMode", cryptingModeEnumMap, enum_val) {}
   explicit CryptingMode(const int enum_val_as_num) : EnumWrapper("CryptingMode", cryptingModeEnumMap, enum_val_as_num) {}
   explicit CryptingMode(const std::string& enum_as_string) : EnumWrapper("CryptingMode", cryptingModeEnumMap, enum_as_string) {}
   explicit CryptingMode() : EnumWrapper("CryptingMode", cryptingModeEnumMap, CryptingModeEnum::editor) {}
};

enum class MediumTypeEnum { file, filerw, screen, nil };
const std::map<MediumTypeEnum, std::string> typeEnumMap
{
   { MediumTypeEnum::file, "file" },
   { MediumTypeEnum::filerw, "filerw" },
   { MediumTypeEnum::screen, "screen" },
   { MediumTypeEnum::nil, "nil" },
};

class MediumType : public vfm::fsm::EnumWrapper<MediumTypeEnum>
{
public:
   explicit MediumType(const MediumTypeEnum& enum_val) : EnumWrapper("MediumType", typeEnumMap, enum_val) {}
   explicit MediumType(const int enum_val_as_num) : EnumWrapper("MediumType", typeEnumMap, enum_val_as_num) {}
   explicit MediumType(const std::string& enum_as_string) : EnumWrapper("MediumType", typeEnumMap, enum_as_string) {}
   explicit MediumType() : EnumWrapper("MediumType", typeEnumMap, MediumTypeEnum::nil) {}
};

class CryptorEncoder;
class DataPack;
class DataSrcArray;

const char CPAR_DELIMITER = '=';
const std::string INVALID_VALUE = "$$$";

const std::string CPAR_ALG = "alg";                         // For inserting chunks of algorithm via command line in both or...
const std::string CPAR_ENC = "enc";                         // ...only enc or...
const std::string CPAR_DEC = "dec";                         // ...only dec.
const std::string CPAR_IN_TYPE = "intype";                  // Default: file.
const std::string CPAR_OUT_TYPE = "outtype";                // Default: filerw.
const std::string CPAR_IN_FILE_PATTERN = "inpattern";       // Default: ./* or ./*.cry [Resolved depending on mode.]
const std::string CPAR_IN_PATH = "inpath";                  // Default: ".".
const std::string CPAR_ENC_FILE_PATH = "encoder";           // Default: crypting.enc
const std::string CPAR_DEC_FILE_PATH = "decoder";           // Default: INVALID_VALUE
const std::string CPAR_AUTO_CREATE_ENC = "auto" + CPAR_ENC; // Default: auto_create_if_missing.
const std::string CPAR_AUTO_CREATE_DEC = "auto" + CPAR_DEC; // Default: auto_create_if_missing.
const std::string CPAR_MODE = "mode";                       // enc, dec, tester, editor.
const std::string CPAR_JIT = "jit";                         // Allow using JIT (bool), default: true.
const std::string CPAR_TEST_MIN_SIZE = "testmin";
const std::string CPAR_TEST_MAX_SIZE = "testmax";
const std::string CPAR_TEST_NUM = "testnum";
const std::string CPAR_ENC_FILE_EXTENSION = "encfileextension";

class Parameters
{
public:
   // Default values for variables.
   MediumType in_type;
   MediumType out_type;
   std::string enc_file_path = "./crypting.enc";
   std::string dec_file_path = INVALID_VALUE;
   std::string enc_file_extension = ".cry";
   std::string in_file_pattern = INVALID_VALUE;
   AutoMode auto_create_enc;
   AutoMode auto_create_dec;
   CryptingMode mode;
   std::string in_path = ".";
   bool jit = true;
   int test_min_size = 100;
   int test_max_size = 1000;
   int test_num = 4;

   Parameters() : 
      in_type(MediumTypeEnum::file), 
      out_type(MediumTypeEnum::filerw), 
      mode(CryptingModeEnum::tester), 
      auto_create_enc(AutoModeEnum::auto_create_if_missing), 
      auto_create_dec(AutoModeEnum::auto_create_if_missing) {}

   const std::string DEFAULT_IN_FILE_PATTERN_ENC = "*";
   const std::string DEFAULT_IN_FILE_PATTERN_DEC = "*" + enc_file_extension;
};

class CryptorStarter : public Failable {
public:
   friend std::ostream& ::operator<<(std::ostream &os, CryptorStarter const &m);

   void autoCreateEncoder();
   void autoCreateDecoder();

   bool test(const std::string& test_word, const bool& verbose);
   void testWithOutput(int & failed, const std::string& s, const bool& verbose);
   bool testManyRandom(const int & how_many_alpha_numeric, const int & how_many_arbitrary, const int& size_min, const int& size_max);

   std::shared_ptr<DataPack> get_data() const;

   void decrypt(const std::shared_ptr<DataSrcArray>& in_encrypted_array, const std::shared_ptr<DataSrcArray>& out_decrypted_array);
   void encrypt(const std::shared_ptr<DataSrcArray>& in_decrypted_array, const std::shared_ptr<DataSrcArray>& out_encrypted_array);

   std::string encrypt_easy(const std::string& word);
   std::string decrypt_easy(const std::string& word);

   void processCommandLineArgs(int argc, char* argv[]);

   std::shared_ptr<CryptorEncoder> getEncryptor();
   std::shared_ptr<CryptorEncoder> getDecryptor();

   std::string helper(const std::string in_name, const int& cnt);

   void run();

   CryptorStarter(int argc, char* argv[]);

private:
   Parameters pars_;

   std::shared_ptr<DataPack> data = std::make_shared<DataPack>();
   std::shared_ptr<CryptorEncoder> encryptor;
   std::shared_ptr<CryptorEncoder> decryptor;

   void processCommandLineArg(std::string& key, std::string& val);
   void resetCommandLineVars();

   void applyMode(const CryptingMode& val);
};

using ::operator<<;

} // vfm
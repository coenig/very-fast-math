//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "static_helper.h"
#include "failable.h"
#include "data_pack.h"
#include "parser.h"
#include "vfmacro/script.h"


namespace vfm {
namespace mc {

class EnvModel : public Failable {
public:
   explicit EnvModel(const std::shared_ptr<DataPack> data, const std::shared_ptr<FormulaParser> parser);
   explicit EnvModel(const std::string& env_model_template, const std::shared_ptr<DataPack> data, const std::shared_ptr<FormulaParser> parser);

   std::string getTemplate() const;
   void setTemplate(const std::string& env_model_template);
   void loadFromFile(const std::string& file_path);
   std::string generateEnvModel(const std::shared_ptr<Failable> father_failable = nullptr);

private:
   std::string template_{};
   std::shared_ptr<DataPack> data_;
   std::shared_ptr<FormulaParser> parser_;
};

} // mc
} // vfm

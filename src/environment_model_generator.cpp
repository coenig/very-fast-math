#include "model_checking/environment_model_generator.h"


using namespace vfm;
using namespace mc;

vfm::mc::EnvModel::EnvModel(const std::shared_ptr<DataPack> data, const std::shared_ptr<FormulaParser> parser) : EnvModel(std::string(), data, parser)
{}

vfm::mc::EnvModel::EnvModel(const std::string& env_model_template, const std::shared_ptr<DataPack> data, const std::shared_ptr<FormulaParser> parser) : Failable("EnvModelGenerator"), template_(env_model_template), data_(data), parser_(parser)
{}

std::string vfm::mc::EnvModel::getTemplate() const
{
   return template_;
}

void vfm::mc::EnvModel::setTemplate(const std::string& env_model_template)
{
   template_ = env_model_template;
}

void vfm::mc::EnvModel::loadFromFile(const std::string& file_path)
{
   setTemplate(StaticHelper::readFile(file_path));
   addDebug("Loaded environment model template from '" + file_path + "'.");
}

std::string vfm::mc::EnvModel::generateEnvModel(const std::shared_ptr<Failable> father_failable)
{
   return vfm::macro::Script::processScript(getTemplate(), macro::Script::DataPreparation::both, false, data_, parser_, father_failable);
}

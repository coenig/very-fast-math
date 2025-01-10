//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term.h"
#include "static_helper.h"
#include <memory>

namespace vfm {

class FormulaParser;
class DataPack;

template <class TClass>
class TFunctor
{
public:
   typedef void (TClass::*FctPtr)(const std::shared_ptr<DataPack> data, const std::string& currentStateVarName);
   
   TFunctor<TClass>(const std::string& name) : name_(name) {};

   virtual void call(const std::shared_ptr<DataPack> data, const std::shared_ptr<FormulaParser> parser, const std::string& currentStateVarName) = 0;
   virtual FctPtr getFpt() const = 0;
   virtual TClass* getpt2Object() const = 0;

   virtual std::shared_ptr<Term> getFormula() const
   {
      return nullptr;
   }

   virtual void setFormula(const std::shared_ptr<Term> formula) 
   {

   }

   std::string getName() const
   {
      return name_;
   }

protected:
   void setName(const std::string& name) {
      name_ = name;
   }

private:
   std::string name_;
};

/// Note that TInternalFunctor is not actually a functor, but needs the same interface as TSpecificFunctor
/// to be usable in a common way by the FSM. Therefore, a dummy function pointer is returned by the
/// getFpt method, while the call method actually just evaluates a formula.
template <class TClass> 
class TInternalFunctor : public TFunctor<TClass>
{
public:
   TInternalFunctor(const std::shared_ptr<Term> formula, const std::string& name)
      : TFunctor<TClass>(name), formula_(formula) {}

   TInternalFunctor() : TInternalFunctor(nullptr, "#UNNAMED-INTERNAL-CALLBACK") {}

   void call(const std::shared_ptr<DataPack> data, const std::shared_ptr<FormulaParser> parser, const std::string& currentStateVarName) override
   {
      formula_->eval(data, parser, false);
   }

   typename TFunctor<TClass>::FctPtr getFpt() const override 
   {
      return nullptr;
   }

   TClass* getpt2Object() const override 
   {
      return nullptr;
   }

   virtual std::shared_ptr<Term> getFormula() const
   {
      return formula_;
   }

   virtual void setFormula(const std::shared_ptr<Term> formula) {
      formula_ = formula;
      this->setName(std::string("[") + StaticHelper::replaceAll(formula->serializePlainOldVFMStyle(), " ", "") + std::string("]"));
   }

private:
   std::shared_ptr<Term> formula_;
};

template <class TClass> 
class TSpecificFunctor : public TFunctor<TClass>
{
public:
    TSpecificFunctor<TClass>(
       TClass* _pt2Object, 
       void(TClass::*_fpt)(const std::shared_ptr<DataPack> data, const std::string& currentStateVarName),
       const std::string& name) : TFunctor<TClass>(name)
    {
        pt2Object = _pt2Object;
        fpt=_fpt; 
    }

    virtual void call(const std::shared_ptr<DataPack> data, const std::shared_ptr<FormulaParser> parser, const std::string& currentStateVarName) override
    { 
        (*pt2Object.*fpt)(data, currentStateVarName);
    }

    typename TFunctor<TClass>::FctPtr getFpt() const override
    {
       return fpt;
    }

    TClass* getpt2Object() const override
    {
       return pt2Object;
    }

private:
    typename TFunctor<TClass>::FctPtr fpt;
    TClass* pt2Object;
};

} // vfm
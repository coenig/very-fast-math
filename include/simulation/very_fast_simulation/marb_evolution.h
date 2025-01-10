//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "environment_2d.h"
#include "failable.h"
#include <memory>
#include <thread>
#include <ctime>
#include <chrono>

using namespace vfm::fsm;

namespace vfm {

constexpr int POPULATION_SIZE = 100;
constexpr int GENERATIONS = 10000;
constexpr int PARALLEL_RUNS = 4;
constexpr int EVALUATION_STEPS = 5000;
constexpr int VISUALIZATION_STEPS = 100000;
constexpr size_t NUM_CARS = 8;

class MARBEvolution : Failable {
public:
   MARBEvolution();

   void evolve();
   void evaluate(); /// Note: Can be made a bit faster if the parallel chunks are not waiting until each one is finished.
   void mutateAll();
   void select();

private:
   std::shared_ptr<FSM<Environment2D<NUM_CARS>>> population_[POPULATION_SIZE];
   float fitness_[POPULATION_SIZE];
   Environment2D<NUM_CARS> environment_[PARALLEL_RUNS];
   int current_best_ = -1;
   std::shared_ptr<FSM<Environment2D<NUM_CARS>>> overallBest_;
   int overallBestFitness_ = -99999999;

   static int getEnvForController(const int controller_id);
   void runForSteps(const int env_id, const int steps);
};

vfm::MARBEvolution::MARBEvolution() : Failable("MARB-Evolution")
{
   std::srand(std::time(nullptr));

   std::cout << std::endl;
   for (int i = 0; i < POPULATION_SIZE; i++) {
      std::cout << ".";
   }

   std::cout << std::endl;
   auto parser = SingletonFormulaParser::getInstance();
   for (int i = 0; i < POPULATION_SIZE; i++) {
      population_[i] = std::make_shared<FSM<Environment2D<NUM_CARS>>>(
         std::make_shared<FSMResolverDefault>(), 
         NonDeterminismHandling::ignore, 
         nullptr, 
         parser, 
         FSMJitLevel::no_jit);
      population_[i]->setProhibitDoubleEdges(true);
      environment_[getEnvForController(i)].registerEgoController(population_[i]);
      std::cout << "|";
   }

   overallBest_ = std::make_shared<FSM<Environment2D<NUM_CARS>>>(
      std::make_shared<FSMResolverDefault>(), 
      NonDeterminismHandling::ignore, 
      nullptr, 
      parser, 
      FSMJitLevel::no_jit);
   overallBest_->setProhibitDoubleEdges(true);
   Environment2D<NUM_CARS>().registerEgoController(overallBest_);

   std::cout << std::endl;
}

inline void MARBEvolution::evolve()
{
   constexpr int muts = 10;

   std::cout << std::endl;
   for (int i = 0; i < muts; i++) {
      std::cout << ".";
   }

   std::cout << std::endl;
   for (int i = 0; i < muts; i++) {
      mutateAll();
      std::cout << "M";
   }
   std::cout << std::endl;

   float last_best_fit = -std::numeric_limits<float>::infinity();

   for (int i = 0; i < GENERATIONS; i++) {
      addNote("Current generation is " + std::to_string(i));
      addNote("Current best / overall best: " + std::to_string(last_best_fit) + " (" + std::to_string(current_best_) + ") / " + std::to_string(overallBestFitness_));
      addNote("Evaluating...");
      evaluate();
      last_best_fit = fitness_[current_best_];


      if (overallBest_) {
         std::cout << overallBest_->serializeToProgram() << std::endl;
         overallBest_->createGraficOfCurrentGraph("ControllerImageBEST" + std::to_string(i) + "_" + std::to_string(fitness_[current_best_]), true, "pdf", false, true, false);
         overallBest_->createGraficOfCurrentGraph("ControllerDataBEST" + std::to_string(i) + "_" + std::to_string(fitness_[current_best_]), true, "pdf", false, false, true);
      }

      if (i % 100 == 99) {
         auto curr_best_aut = population_[current_best_];
         addNote("Simulating current best for a bit...");
         std::cout << curr_best_aut->serializeToProgram() << std::endl;
         auto& env = environment_[getEnvForController(current_best_)];

         env.registerEgoController(curr_best_aut);
         env.resetAndRandomizeTraffic(true);

         int stps = 200;

         std::cout << std::endl;
         for (int i = 0; i < stps; i++) {
            if (i % 10 == 0) std::cout << ".";
         }

         std::cout << std::endl;
         for (int ii = 0; ii < stps; ii++) {
            env.step(true, true, true, true, true);
            env.getOutsideView().store("./HighwayScene" + std::to_string(0) + ".png");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (ii % 10 == 0) std::cout << "|";

            if (ii % 1 == 0) {
               curr_best_aut->createGraficOfCurrentGraph("ControllerImage" + std::to_string(0), true, "pdf", false, true, false);
               curr_best_aut->createGraficOfCurrentGraph("ControllerData" + std::to_string(0), true, "pdf", false, false, true);
               //std::cout 
               //   << _Get(_var(AGENTS_POS_X_NAME), _var(EGO_LEAD_CAR_NAME))->eval(env.getData()) 
               //   << " ("
               //   << _var(EGO_LEAD_CAR_NAME)->eval(env.getData())
               //   << ") "
               //   << " <- "
               //   << " lead_pos / ego_y_pos "
               //   << " -> "
               //   << _var(EGO_POS_Y_NAME)->eval(env.getData()) 
               //   << std::endl;

               //std::cout 
               //   << env.agents_pos_x_[env.agents_lead_cars_[NUM_CARS]]
               //   << " ("
               //   << (int) env.agents_lead_cars_[NUM_CARS]
               //   << ") "
               //   << " <- "
               //   << " lead_pos / ego_y_pos "
               //   << " -> "
               //   << env.ego_pos_y_
               //   << std::endl
               //   << std::endl;
            }
         }

         std::cout << std::endl;

         addNote("Simulation finished.");
      }

      addNote("Selecting...");
      select();
      addNote("Mutating...");
      mutateAll();
   }
}

inline void MARBEvolution::evaluate()
{
   std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

   for (int i = 0; i < POPULATION_SIZE; i = i + PARALLEL_RUNS) {
      std::shared_ptr<std::thread> threads[PARALLEL_RUNS];

      for (int curr_cont = i; curr_cont < i + PARALLEL_RUNS && curr_cont < POPULATION_SIZE; curr_cont++) {
         const auto env_id = getEnvForController(curr_cont);
         const auto fct = [this, env_id]() { this->runForSteps(env_id, EVALUATION_STEPS); };
         const auto cont = population_[curr_cont];
         auto& env = environment_[env_id];

         env.registerEgoController(cont);
         env.resetAndRandomizeTraffic(true);
         threads[env_id] = std::make_shared<std::thread>(fct);
         //addNote("Evaluation of controller " + std::to_string(curr_cont) + " started in environment " + std::to_string(env_id) + ". Simulating " + std::to_string(EVALUATION_STEPS) + " steps.");
      }

      for (const auto thread : threads) {
         if (thread) {
            thread->join();
         }
      }

      //addNote("Threads joined.");

      for (int curr_cont = i; curr_cont < i + PARALLEL_RUNS && curr_cont < POPULATION_SIZE; curr_cont++) {
         fitness_[curr_cont] = environment_[getEnvForController(curr_cont)].getEgoFitness();

         if (current_best_ < 0 || fitness_[curr_cont] > fitness_[current_best_]) {
            current_best_ = curr_cont;

            if (fitness_[current_best_] > overallBestFitness_) {
               overallBest_->takeOverTopologyFrom(population_[current_best_]);
               overallBestFitness_ = fitness_[current_best_];
            }
         }

         //addNote("Fitness " + std::to_string(curr_cont) + ": " + std::to_string(fitness_[curr_cont]));
      }

      //addNote("Current best is " + std::to_string(current_best_) + " with fitness: " + std::to_string(fitness_[current_best_]) + " (" + std::to_string(overallBestFitness_) + ")");
   }

   std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
   addNote("Time (eval) = " + std::to_string(std::chrono::duration_cast<std::chrono::seconds>(end - begin).count()) + " [s]");
}

inline void MARBEvolution::mutateAll()
{
   constexpr int mutations = 1;
   constexpr int num_callbacks = 3;

   for (const auto& cont : population_) {
      int temp_action_id = 0;

      if (cont->getStateCount() < num_callbacks + 1) {
         cont->associateStateIdToActionId(INITIAL_STATE_NUM, temp_action_id++);

         while (cont->getStateCount() < num_callbacks + 1) {
            const int state_id = cont->smallestFreeStateID();
            cont->addUnconnectedStateIfNotExisting(state_id);
            cont->associateStateIdToActionId(state_id, temp_action_id++);
            cont->addTransition(INITIAL_STATE_NUM, state_id, _false());
         }

         for (int i = INITIAL_STATE_NUM; i < cont->smallestFreeStateID(); i++) {
            for (int j = INITIAL_STATE_NUM; j < cont->smallestFreeStateID(); j++) {
               if (j != INITIAL_STATE_NUM) {
                  if (i == INITIAL_STATE_NUM && j == 2) {
                     cont->addTransition(
                        i, 
                        j, 
                        _and(_and(_sm(_Get(_var(AGENTS_POS_X_NAME), _var(EGO_LEAD_CAR_NAME)), _val(40)), _greq(_var(EGO_POS_Y_NAME), _val(0.5))), _greq(_Get(_var(AGENTS_POS_X_NAME), _var(EGO_LEAD_CAR_NAME)), _val(0.1))));
                  }
                  else if (i == INITIAL_STATE_NUM && j == 3) {
                     cont->addTransition(
                        i, 
                        j, 
                        _and(_and(_sm(_Get(_var(AGENTS_POS_X_NAME), _var(EGO_LEAD_CAR_NAME)), _val(40)), _sm(_var(EGO_POS_Y_NAME), _val(0.5))), _greq(_Get(_var(AGENTS_POS_X_NAME), _var(EGO_LEAD_CAR_NAME)), _val(0.1))));
                  }
                  else if (i == j && i != INITIAL_STATE_NUM) {
                     cont->addTransition(
                        i, 
                        j, 
                        _greq(_minus(_var(EGO_POS_Y_NAME), _trunc(_var(EGO_POS_Y_NAME))), _val(Environment2D<NUM_CARS>::EPSILON)));
                  }
                  else {
                     cont->addTransition(i, j, _false());
                  }
               }
            }
         }
      }
      else {
         for (int i = 0; i < mutations; i++) {
            cont->mutate();
         }
      }
   }
}

inline void MARBEvolution::select()
{
   constexpr int TOURNAMENT_SIZE = 5;

   std::shared_ptr<FSM<Environment2D<NUM_CARS>>> population[POPULATION_SIZE];
   float fitness[POPULATION_SIZE];

   population[0] = population_[0];
   population[0]->takeOverTopologyFrom(population_[current_best_]);

   for (int i = 1; i < POPULATION_SIZE; i++) {
      std::vector<int> tournament;
      int best_id = 0;
      int best = -std::numeric_limits<float>::infinity();

      for (int j = 0; j < TOURNAMENT_SIZE; j++) {
         int candidate_id = std::rand() % POPULATION_SIZE;

         if (best < fitness_[candidate_id]) {
            best = fitness_[candidate_id];
            best_id = candidate_id;
         }
      }

      population[i] = population_[i];
      population[i]->takeOverTopologyFrom(population_[best_id]);
      fitness[i] = best;
   }

   for (int i = 0; i < POPULATION_SIZE; i++) {
      population_[i] = population[i];
      fitness_[i] = -std::numeric_limits<float>::infinity();
   }
}

inline int MARBEvolution::getEnvForController(const int controller_id)
{
   return controller_id % PARALLEL_RUNS;
}

inline void MARBEvolution::runForSteps(const int env_id, const int steps)
{
   auto& env = environment_[env_id];
   const int div = steps / 10;

   for (int i = 0; i < steps; i++) {
      if (i % div == div - 1) {
         env.resetAndRandomizeTraffic(false);
      }

      if (i % VISUALIZATION_STEPS == VISUALIZATION_STEPS - 1) {
         env.getOutsideView().store("./HighwayScene" + std::to_string(env_id) + "_" + std::to_string(i) + ".png");
         env.getEgoController()->createGraficOfCurrentGraph("ControllerImage" + std::to_string(env_id) + "_" + std::to_string(i), true, "pdf", false, true, false);
         env.getEgoController()->createGraficOfCurrentGraph("ControllerData" + std::to_string(env_id) + "_" + std::to_string(i), true, "pdf", false, false, true);
         std::cout << env.getEgoFitness() << std::endl;
      }

      env.step(true, true, true, true, true);
   }
}

}
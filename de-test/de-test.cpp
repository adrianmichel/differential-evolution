#include "pch.h"
#include "CppUnitTest.h"

#include <differential_evolution.hpp>
#include <test_listener.h>
#include <test_processor_listener.h>
#include <test_functions.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace de = amichel::de;

enum Objective {
  minimize,
  maximize
};

de::individual_ptr runTest(de::constraints_ptr constraints, de::ObjectiveFunction of, de::selection_strategy_ptr selectionStrategy,
  de::mutation_strategy_ptr mutationStrategy, size_t processor_count, size_t max_generation, size_t argumentCount, size_t populationSize, Objective objective) {
  // get the constraints as defined on the command line
  assert(constraints);

  // get the objective function as selected on the command line
  assert(of);

  // instantiate a DE listener
  de::listener_ptr listener(std::make_shared<DETestListener>());

  // instantiate a Processors listener
  de::processor_listener_ptr processorListener(
    std::make_shared<DETestProcessorListener>());

  // instantiate the Processors, using the number of processors defined on the
  // command line, and the processors listener
  de::processors::processors_ptr processors(
    std::make_shared<de::processors>(
      processor_count, of, processorListener));

  // instantiate a basic termination strategy (just count the # of generations)
  de::termination_strategy_ptr terminationStrategy(
    std::make_shared<de::max_gen_termination_strategy>(
      max_generation));

  // create a differential_evolution object using all the parameters defined
  // above or on the command line
  amichel::de::differential_evolution de(
    argumentCount, populationSize, processors,
    constraints, objective == minimize, terminationStrategy, selectionStrategy,
    mutationStrategy, listener);

  // run the optimization process
  de.run();
  return de.best();
}

/**
 * Objective function to optimize is the "sphere function":
 *
 * f(x,y) = x^2 + y^2
 */

constexpr auto varsCount = 20;
constexpr auto populationSize = 200;
constexpr auto processorCount = 4;
constexpr auto maxGeneration = 1000;
constexpr auto weight = 0.5;
constexpr auto crossover = 0.9;

using TF = double(*)(de::DVectorPtr);
template< typename selection_strategy, typename mutation_strategy> double testCase(de::ObjectiveFunction test_function) {
  de::constraints_ptr constraints(std::make_shared<de::constraints>(varsCount, -1.0e6, 1.0e6));
  (*constraints)[0] = std::make_shared<de::real_constraint>(-10, 10);
  (*constraints)[1] = std::make_shared<de::real_constraint>(-100, 100);

  de::selection_strategy_ptr selectionStrategy(std::make_shared<selection_strategy>());
  de::mutation_strategy_arguments mutation_arguments(weight, crossover);
  de::mutation_strategy_ptr mutationStrategy(std::make_shared<mutation_strategy>(varsCount, mutation_arguments));


  de::individual_ptr best = runTest(constraints, test_function, selectionStrategy, mutationStrategy, processorCount, maxGeneration, varsCount, populationSize, minimize);
  return best->cost();
}

class SphereFunctionClass {
public:
  double operator()(amichel::de::DVectorPtr args) { return SphereFunction(args); }
};

namespace de_test
{
	TEST_CLASS(de_test)
	{
	public:
    TEST_METHOD(SphereFunctionTest0)
    {
      double bestCost = testCase< de::best_parent_child_selection_strategy, de::mutation_strategy_1>(SphereFunction);
      // best cost is 0, but the result can be a very small value
      Assert::IsTrue(1e-100 > bestCost);
    }

    TEST_METHOD(SpereFunctorTest) {
      double bestCost = testCase< de::best_parent_child_selection_strategy, de::mutation_strategy_1>(SphereFunctionClass{});
      Assert::IsTrue(1e-100 > bestCost);
    }

    TEST_METHOD(SphereLambdaTest) {
      auto sphere = [](de::DVectorPtr args)->double { return SphereFunction(args); };

      double bestCost = testCase< de::best_parent_child_selection_strategy, de::mutation_strategy_1>(sphere);
      Assert::IsTrue(1e-100 > bestCost);
    }

		TEST_METHOD(SphereFunctionTest1)
		{
      double bestCost = testCase< de::tournament_selection_strategy, de::mutation_strategy_1>(SphereFunction);
      Assert::IsTrue(1e-100 > bestCost);
		}

    TEST_METHOD(SquareTest)
    {
      double bestCost = testCase< de::tournament_selection_strategy, de::mutation_strategy_1>(x_sqr_min_function);
      Assert::IsTrue(1e-100 > bestCost);
    }

    TEST_METHOD(TestSquare1)
    {
      double bestCost = testCase< de::best_parent_child_selection_strategy, de::mutation_strategy_1>(x_sqr_min_function);
      Assert::IsTrue(1e-100 > bestCost);
    }

    TEST_METHOD(TestSixHumpCamel)
    {
      double bestCost = testCase< de::tournament_selection_strategy, de::mutation_strategy_1>(SixHumpCamelBackFunction);
      Assert::IsTrue(-1.031628 > bestCost && -1.031629 < bestCost);
    }

    TEST_METHOD(TestSixHumpCamel1)
    {
      double bestCost = testCase< de::best_parent_child_selection_strategy, de::mutation_strategy_1>(SixHumpCamelBackFunction);
      Assert::IsTrue(-1.031628 > bestCost && -1.031629 < bestCost);
    }
  };
};


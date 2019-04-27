[TOC]

# C++ Differential Evolution #
## Basic concepts ##
### Introduction ###

Differential Evolution (DE) is a population based stochastic function optimizer algorithm developed by Kenneth Price and Rainer Storn in the 1990s. For detailed information on DE consult any of the numerous online or in print resources listed at the DE homepage (http://www.icsi.berkeley.edu/~storn/code.html)

To quote from the Wikipedia page dedicated to DE (http://en.wikipedia.org/wiki/Differential_evolution): 

*In computer science, differential evolution (DE) is a method that optimizes a problem by iteratively trying to improve a candidate solution with regard to a given measure of quality. Such methods are commonly known as metaheuristics as they make few or no assumptions about the problem being optimized and can search very large spaces of candidate solutions. However, metaheuristics such as DE do not guarantee an optimal solution is ever found.*

*DE is used for multidimensional real-valued functions but does not use the gradient of the problem being optimized, which means DE does not require for the optimization problem to be differentiable as is required by classic optimization methods such as gradient descent and quasi-newton methods. DE can therefore also be used on optimization problems that are not even continuous, are noisy, change over time, etc.*

### Real use cases
#### Algorithmic trading system optimization
This library was initially developed to be used for trading system optimization, and this section shows why.

An algoritmic trading system (TS) can be defined as a set of rules that takes market data as input and generates trading signals as output. These rules are often based on various statistical functions that are applied to stock prices, such as moving averages, standard deviation, which in turn depend on variables like the moving average period etc. 

Some believe that a TS may perform better if they are optimized to adapt to current market conditions. To optimize a TS, it will be repeatedly applied to past data using different sets of variable values until the best hypothetical result is obtained for that period. The best parameters then are applied in real time. 

Due to the large variables space (total number of distinct variable value sets), exhaustive optimization (EO), which traverses all these sets is usually impractical. For example 4 variables taking 10 distinct values each would require 10000 trading system runs. Other optimization methods may also be hard or impossible to apply due to the nature of the problem and its data.

DE however works well with this type of problem, and could be applied to reduce the explored variable space by orders of magnitude. For example, a DE optimization session could be set for a population of 10 individuals and a maximum number of generations of 20 as termination criteria (please refer to the terminology section for information on these terms). This would require at most 200 system runs, and often less, if other termination criteria is used such as a performance target x% better than a benchmark (S&P500 or other index). One may doubt the fact that DE offers an advantage over EO as the same termination criteria could be used when running exhaustive optimization, which would also limit the number of runs. But the DE algorithm is designed to converge much faster toward an optimium than a method that simply traverses the variables space.

Please note that it is not proven whether this method actually helps improve a trading system performance, the main point being howerver that DE can be used to obtain a good enough value in a reasonable number of tries.
#### Other uses
A quick Google search will yield many practical application of DE. Here are a few references to such articles just to illustrate the variaty of domains DE is applicable to:

* [An evaluation of Differential Evolution in software test data generation](http://ieeexplore.ieee.org/xpl/freeabs_all.jsp?arnumber=4983300)
* [Application of Differential Evolution for a Single-Item Resource-Constrained Aggregate Production Planning Problem](https://docs.google.com/viewer?url=http%3A%2F%2Fwww.iaeng.org%2Fpublication%2FIMECS2008%2FIMECS2008_pp1773-1778.pdf)
* [Differential Evolution Algorithm with Application to Optimal Operation of Multipurpose Reservoir]([http://www.scirp.org/Journal/PaperInformation.aspx?paperID=2071)
* [Combined optimization using Cultural and Differential Evolution: application to crystal structure solution from powder diffraction data](http://www.mendeley.com/research/combined-optimization-using-cultural-and-differential-evolution-application-to-crystal-structure-solution-from-powder-diffraction-data/)
* [Grammatical Differential Evolution](https://docs.google.com/viewer?url=http%3A%2F%2Fncra.ucd.ie%2Fpapers%2FICAI06_GDE.pdf) used for automatic program generation
* [Application of Differential Evolution to Determine the HEPWM Angles of a Three Phase Voltage Source Inverter](http://digitalcommons.calpoly.edu/cgi/viewcontent.cgi?article=1217&context=eeng_fac)

### Target audience

DE can be useful to many involved in scientific or engineering areas such as: chemistry, electronics, physics, agriculture, telecommunications, software engineering etc.

Using the library doesn't require prior understanding of its or DE internals. Developers with average C++ skills should be able to use it "out of the box". The library provides default behavior or a selection of several pre-defined policy classes, so that those who are not interested in the DE internals will only have to plug-in their objective function, select the DE runtime parameters, build the code and run it.

Those who need more advanced customization of the DE algorithm or are interested in researching DE itself, can create their own implementations of the various classes that control the algorithm behavior.
Rationale

This component is intended to address the need for a standardized generic, portable and efficient C++ DE library that can be easily customized for a variety of domain specific problems. 

### Features of this implementation

* Generic - doesn't make assumptions about the OS, environment it's running in or the problem to optimize
* Written in 100% portable C++14, with stl as only dependence
* Single-header project
* Modular
* Robust and memory efficient due to extensive use of shared pointers
* Performant thanks to platform independent multi-processing
* Adjustable and extensible through classes that control its behavior
* Easy to integrate with existing objective functions
* Supports several types of variable constraints useful in real life applications

### Terminology

*Objective Function* - the function to optimize. 

*Cost* - numeric output of the objective function for a given set of input variables.

*Constraint* - defines the domain of acceptable values for a variable. For example variables can be of real, integer type with values limited to a specific range. Other types include set, which can only take one of the several values in a set, or Boolean type, which can be true or false (1 or 0).

*Variable* - a value limited by a constraint

*Individual* - member of a population containing a collection of variables (implemented as a vector), each defined by its own constraints, dictated by the nature of the objective function. These values are passed to the objective function during the optimization process in order to calculate the function cost, which is also stored by the individual. Note: DE requires a larger number of variables than used by the objective function. For example if we are optimizing a function in two variables f(x,y), DE may require a vector of 20 total variables for its internal processing.

*Population* - a collection of individuals (implemented as a vector). During the optimization process, the population changes according to mutation algorithms to attempt to generate the optimium individual.

*Mutation* - the process of modifying the variable values according to one of the several mutation algorithms. The mutation algorithms can be tuned with the help of two parameters: weight and crossover. Consult the detailed DE documentation online for more information on these parameters.

*Selection* - the process of selecting the individuals for the next generation, of the best individual for the current generation, and of the best individual for the entire process.

*Processor* - one objective function processing unit. Each processor runs in its own thread on the same computer, but the processor class can be modified to distribute work across multiple computers. Any number of processors can be created and used during an optimization for parallel processing.

*Generation* - one iteration during the optimization process, which transforms one population in a new one, by mutating some of its individuals.

*Termination* - the logic used to determine when the optimization has completed.

### The DE Algorithm

The algorithm has been modified somewhat in this implementation to allow for multi-processing and use of policy classes, and this its description.

1. DE starts by creating an initial population, with individuals set to random variable values within the limits defined by their constraints, and then calculates the cost (the objective function value) for those variables for each individual.
1. DE then proceeds to iterate through the current population and mutate variables for each individual according to the mutation policy. All the mutated individuals are then pushed into the processing queue.
1. The processors now start evaluating the objective function for variables associated with individuals in the processing queue. The result (cost value) is set into each corresponding individual. This continues until the queue is empty. All these individuals are now candidates for the next generation.
1. DE now applies the selection policy to choose the individuals that will make it into the next generation, and also of the best individual for the current iteration and for the entire optimization process. The new generation becomes the current generation for the next iteration. There are currently two selection policies for the new generation:
    1. Sort the combined populations and select the first N individuals (where N is the population size) for the new population.
    1. Compare individuals with the same index from the old and candidate generation and pick the best of each for the new generation.
    Practical tests have shown that 1 allows the optimization to converge to an optimum value faster than 2.
1. DE applies the termination policy to determine if it should stop the process. The termination policy can be as simple as comparing the generation number with a constant representing the maximum allowed number of generations. Or it can be more complex, for example by analyzing the trend of best values for each generation and stopping when the best value change is at or below a predefined threshold.
1. If the termination criterion has been met, the process stops and the best value returned to the user. Otherwise, the process restarts from 2. with the new generation as the current generation.

Note that the the DE efficiency and even ability to converge to an optimum value depend (heavily for some objective functions) on its parameters: number of variables (total, not just the ones required by the objective function), population size, weight and crossover factors, as well as the different policies (mutation, selection, termination). Consult the online resources on this topic to get the most out of DE, and experiment with your own values as well.

## Tutorial

The following functional sample shows how to set up and run a basic DE optimization session. Only default implementations of listeners and various policy classes have been used to avoid distracting the reader with irrelevant details. In real applications, some or all of the classes used in this sample can be replaced with more useful implementations.

A more realistic use is presented in the downloadable project which can be used to test DE with several functions that are commonly used to assess the efficiency of optimization algorithms.


```c++
#include <differential_evolution.hpp>
#include "objective_function.h"

using namespace de;

/** 
 * Objective function to optimize is the "sphere function": 
 *  
 * f(x,y) = x^2 + y^2 
*/
class sphere_function : public objective_function
{
public:
    sphere_function()
    : objective_function( "sphere function" )
    {
    }

    virtual double operator()( de::DVectorPtr args )
    {
        /**
         * The two function arguments are the elements index 0 and 1 in 
         * the argument vector, as defined by the constraints vector 
         * below 
         */
        double x = (*args)[ 0 ];
        double y = (*args)[ 1 ];

        return x*x + y*y;
    }
};

#define VARS_COUNT 20
#define POPULATION_SIZE 200

void simpleUsage()
{
    try
    {
        /**
         * Create and initialize the constraints object 
         *  
         * First create it with default constraints (double type, min 
         * -1.0e6, max 1.0e6) then set the first two elements to be of 
         *  type real with x between -10, 10 and y between -100, 100.
         */
        constraints_ptr constraints( boost::make_shared< constraints >( VARS_COUNT , -1.0e6, 1.0e6 ) );
        (*constraints)[ 0 ] = boost::make_shared< real_constraint >( -10, 10 );
        (*constraints)[ 1 ] = boost::make_shared< real_constraint >( -100, 100 );

        /**
         * Instantiate the objective function 
         *  
         * The objective function can be any function or functor that 
         * takes a de::DVectorPtr as argument and returns a double. It 
         * can be passed as a reference, pointer or shared pointer. 
         */
        objective_function_ptr of( boost::make_shared< sphere_function >() );

        /**
         * Instantiate two null listeners, one for the differential 
         * evolution, the other one for the processors 
         */
        listener_ptr listener( boost::make_shared< null_listener >() );
        processor_listener_ptr processor_listener( boost::make_shared< null_processor_listener >() );

        /**
         * Instantiate the collection of processors with the number of 
         * parallel processors (4), the objective function and the 
         * listener 
         */
        processors< objective_function_ptr >::processors_ptr _processors( boost::make_shared< processors< objective_function_ptr > >( 4, of, processor_listener ) );

        /**
         * Instantiate a simple termination strategy wich will stop the 
         * optimization process after 10000 generations 
         */
        termination_strategy_ptr terminationStrategy( boost::make_shared< max_gen_termination_strategy >( 10000 ) );

        /**
         * Instantiate the selection strategy - we'll use the best of 
         * parent/child strategy 
         */
        selection_strategy_ptr selectionStrategy( boost::make_shared< best_parent_child_selection_strategy >() );

        /**
         * Instantiate the mutation strategy - we'll use the mutation 
         * strategy 1 with the weight and crossover factors set to 0.5 
         * and 0.9 respectively 
         */
        mutation_strategy_arguments mutation_arguments( 0.5, 0.9 );
        mutation_strategy_ptr mutationStrategy( boost::make_shared< mutation_strategy_1 >( VARS_COUNT, mutation_arguments ) );

        /**
         * Instantiate the differential evolution using the previously 
         * defined constraints, processors, listener, and the various 
         * strategies 
         */
        differential_evolution< objective_function_ptr > de( VARS_COUNT, POPULATION_SIZE, _processors, constraints, true, terminationStrategy, selectionStrategy, mutationStrategy, listener );

        /**
         * Run the optimization process
         */
        de.run();

        /**
         * Get the best individual resulted from the optimization 
         * process 
         */
        individual_ptr best( de.best() );

        /**
         * Print out the result
         */
        std::cout << "minimum value for the " << of->name() << " is " << best->cost() << " for x=" << (*best->vars())[ 0 ] << ", y=" << (*best->vars())[ 1 ];
    }
    catch( const std::exception& e )
    {
        /**
         * Print out any errors that happened during the initialization 
         * or optimization phases
         */
        std::cout << "an error occurred: " << e.what();
    }
}

```

## The test project
### What is de_test

The de_test project included in the package is a command line application that is used to find the max or min values of several test functions commonly used to benchmark optimization algorithms. These functions, such as Ackley, De Jong etc, are generally hard to optimize because of properties like many local minimia or global minimia surrounded by a high gradient region etc. Here is a page describing several of these functions http://www.geatbx.com/docu/fcnindex-01.html#P247_13252 .

The purpose of this project is to illustrate the use of the DE C++ library in a real application, but de_test can be used as a starting point for more complex application or can even be used to solve real optimization problems by adding new objective functions.

### Building de_test

The current version is released as a Visual Studio 2015 project that compiles and runs on Windows. A Linux version is currently being worked on.

VS 2015 Community Edition is available free from microsoft.com.

#### Installing and building the Boost dependency
The DE project depends on the Boost C++ library which can be downloaded from http://www.boost.org.

Install the library in a directory named "boost" located at the same level as the DE project directory, so if, for example, the DE project is at "c:\dev\de", then boost will have to be installed at c:\dev\boost.

Once installed, open a console window and navigate to the boost directory, then run "boostrap.bat" - this prepares the library for build.

Create a file called build.bat in the boost directory with the following content:

```
b2 --toolset=msvc-14.0 --with-thread --with-regex --with-date_time --with-program_options link=static address-model=32 threading=multi --stagedir=stage\x86 -a
b2 --toolset=msvc-14.0 --with-thread --with-regex --with-date_time --with-program_options link=static address-model=64 threading=multi --stagedir=stage\x64 -a
```

then from the console window run "build.bat". This will build all the boost libraries required by the DE project, and once it's done, you can then build and run the DE project from the VS IDE.

### Running de_test

#### Command line arguments

The de_test command line application requires setting several command line arguments for a succesful run.

The arguments can be passed either on the command line or in a configuration file.

Each argument has a short form (e.g. -w) and a long form (--weight). Both forms are supported when they are passed on the command line, but only the long form (without the --) is supported in the configuration file.

Here is the list of supported arguments presented as **short_form, long_form (=default_value if any) - comments** 

* ?, help - shows the command line usage message
* w, weight - sets the weight factor
* c, crossover - sets the crossover factor
* p, populationsize - sets the number of individuals in the population
* g, generations - sets the masimum number of generations
* m, mutationstrategy (=1) - set to a value between 1-5, for one of the 5 available mutation strategies
* s, selectionstrategy (=1) - set to 1 for "best parent child selection" or 2 for "tournament selection".
* i, minimize - 0 to maximize, 1 to minimize
* f, configfile - the name of the configuration file
* r, processorcount - the number of parallel processors to run the objective function in
* n, constraint - one constraint, using the format "type;min;max" where min and max are floating point values and type is one of real, int (set and boolean constraint types are not exposed through the command line, although they are implemented). Several constraints can be set on the command line, one per variable.
* v, variablescount (=20)- number of variables per individual - it is usually much higher than the number of variables required by the objective function
* x, defconstraintmin (=-1e6) - default min constraint, used for variables that don't have constraints set explicitly on the command line
* y, defconsraintmax (=1e6) - default max constraint, used for variables that don't have constraints set explicitly on the command line.
* o, functiontooptimize - one of the following values:
    1. -(x^2) [max: 0]
    1. x^2 [min:0]
    1. x^6 - 10*x^3 [min: -25]
    1. sphere function (first De Jong function) [min: 0]
    1. Auckley function [min: 0]
    1. second De Jong function [min: 0]
    1. six hump camel back function [min -1.031628453...]

#### Configuration file

As mentioned before, the application arguments can be passed in a configuration file whose path and name are specified on the command line using the argument -f (or --configfile). Configurations files are more convenient if one is supposed to run the application repeatedly while experimenting with various argument values.

The configuration file contains one argument per line:
arg_long_name_1=value_1
arg_long_name_2=value_2
...

Only argument long names are allowed in the configuration file.

For example to specify that the weight factor is to be set to 0.9, we'll enter:
weight=0.9

The user is encouraged to expirement with different values for weight, crossover, number of variables, population size etc to see how the algorithm behaves in a variety of setups
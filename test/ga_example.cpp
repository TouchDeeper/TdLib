//=================================================================================================
//                    Copyright (C) 2017 Olivier Mallet - All Rights Reserved                      
//=================================================================================================

#include "TdLibrary/Galgo.hpp"

// objective class example
template <typename T>
class MyObjective
{
public:
   // objective function example : Rosenbrock function
   // minimizing f(x,y) = (1 - x)^2 + 100 * (y - x^2)^2
   static std::vector<T> Objective(const std::vector<T>& x)
   {
      T obj = -(pow(1-x[0],2)+100*pow(x[1]-x[0]*x[0],2));
      return {obj};
   }
   // NB: GALGO maximize by default so we will maximize -f(x,y)
};

// constraints example:
// 1) x * y + x - y + 1.5 <= 0
// 2) 10 - x * y <= 0
template <typename T>
std::vector<T> MyConstraint(const std::vector<T>& x)
{
   return {x[0]*x[1]+x[0]-x[1]+1.5,10-x[0]*x[1]};
}
// NB: a penalty will be applied if one of the constraints is > 0 
// using the default adaptation to constraint(s) method


// objective class example
template <typename T>
class MyObjective2
{
public:
    // objective function example : Rosenbrock function
    // minimizing f(x) = x^2
    static std::vector<T> Objective(const std::vector<T>& x)
    {
       T obj = -x[0]*x[0];
       return {obj};
    }
    // NB: GALGO maximize by default so we will maximize -f(x)
};
template <typename T>
class MyObjective3
{
public:
    // objective function example : Rosenbrock function
    // minimizing f(x) = x^2
    static std::vector<T> Objective(const std::vector<T>& x)
    {
        constexpr double pi=3.141592653589793238;
        T obj;
        for (int i = 0; i < x.size(); ++i) {
            obj += -(x[i]*x[i]-10.0*cos(2.0*pi*x[i]));
        }

        return {obj};
    }
    // NB: GALGO maximize by default so we will maximize -f(x)
};

int main()
{
//   //example1
//   // initializing parameters lower and upper bounds
//   // an initial value can be added inside the initializer list after the upper bound
//   galgo::Parameter<double> par1({0.0,1.0});
//   galgo::Parameter<double> par2({0.0,13.0});
//   // here both parameter will be encoded using 16 bits the default value inside the template declaration
//   // this value can be modified but has to remain between 1 and 64
//
//   // initiliazing genetic algorithm
//   galgo::GeneticAlgorithm<double> ga(MyObjective<double>::Objective,100,50,true,par1,par2);
//
//   // setting constraints
//   ga.Constraint = MyConstraint;
//
//   // running genetic algorithm
//   ga.run();

//   //example2
//   // initializing parameters lower and upper bounds
//   // an initial value can be added inside the initializer list after the upper bound
//   galgo::Parameter<double> par1({-10.0,10.0});
//   // here both parameter will be encoded using 16 bits the default value inside the template declaration
//   // this value can be modified but has to remain between 1 and 64
//
//   // initiliazing genetic algorithm
//   galgo::GeneticAlgorithm<double> ga(MyObjective2<double>::Objective,100,50,true,par1);
//
//   // setting constraints
////   ga.Constraint = MyConstraint;
//   // running genetic algorithm
//   ga.run();

   //example3
   galgo::Parameter<double,64> par1({-2.0,2.0});
   galgo::Parameter<double,64> par2({-2.0,2.0});
   galgo::Parameter<double,64> par3({-2.0,2.0});
   galgo::Parameter<double,64> par4({-2.0,2.0});
   galgo::Parameter<double,64> par5({-2.0,2.0});
   galgo::GeneticAlgorithm<double> ga(MyObjective3<double>::Objective, 10000, 1000,true, par1,par2,par3,par4,par5 );

   ga.run();
}

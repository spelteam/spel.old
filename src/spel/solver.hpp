#ifndef _SOLVER_HPP_
#define _SOLVER_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <string>
#include <vector>

#include "solvlet.hpp"
#include "sequence.hpp"

namespace SPEL
{
  using namespace std;

  class Solver
  {
  public:
    Solver(void);
    virtual ~Solver(void);
    virtual vector<Solvlet> solve(const Sequence& v);
    virtual vector<Solvlet> solve(const Sequence& v, map<string, float> params);
    ///get the solver name. Every class inheriting solver has its own Name
    virtual string getName(void);
    ///get the solver Id. Every class inheriting solver has its own ID
    virtual int getId(void);
  protected:
    int id;
    string name;
  };

}

#endif  // _SOLVER_HPP_


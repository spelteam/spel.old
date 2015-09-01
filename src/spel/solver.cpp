#include "solver.hpp"

namespace SPEL
{

  Solver::Solver(void)
  {
    id = -1;
    name = "BaseClass";
  }

  Solver::~Solver(void)
  {

  }

  string Solver::getName() //get the solver name. Every class inheriting solver has its own Name
  {
    return name;
  }

  int Solver::getId()
  {
    return id;
  }

  vector<Solvlet> Solver::solve(const Sequence& v)
  {
    map<string, float> params;
    return solve(v, params);
  }

  vector<Solvlet> Solver::solve(const Sequence& v, map<string, float> params)
  {
    vector<Solvlet> empty;

    return empty;
  }

}

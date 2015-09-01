#ifndef _SOLUTION_HPP_
#define _SOLUTION_HPP_

#include <string>
#include <vector>
#include "solvlet.hpp"
#include "solution.hpp"

using namespace std;

namespace SPEL
{

  class Solution
  {
  public:
    Solution(void);
    Solution(int id, int solverId, int seqId, vector<float> params, vector<Solvlet> solvlets);

    Solution &operator=(const Solution &s);
    bool operator==(const Solution &s) const;
    bool operator!=(const Solution &s) const;

    int getId(void);
    void setId(int _id);

    vector<Solvlet> getSolvlets(void);
    void setSolvlets(vector<Solvlet> _solvlets);

  private:
    int id;
    ///id of the solver solution was obtained from
    int solverId;
    int sequenceId;
    ///name of the solver
    string solverName;
    vector <float> solverParams;
    vector <Solvlet> solvlets;
  };

}

#endif  // _SOLUTION_HPP_
 

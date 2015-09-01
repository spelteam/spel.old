#ifndef _SOLVLET_HPP_
#define _SOLVLET_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <string>
#include <vector>

#include "limbLabel.hpp"
#include "skeleton.hpp"
#include "frame.hpp"

namespace SPEL
{
  using namespace std;
  class Solvlet
  {
  public:
    Solvlet(void);
    Solvlet(int id, vector<LimbLabel> labels);
    virtual ~Solvlet(void);

    virtual Solvlet &operator=(const Solvlet &s);
    virtual bool operator<(const Solvlet &s) const;
    virtual bool operator>(const Solvlet &s) const;
    // bool operator==(const Solvlet &s) const;
    // bool operator!=(const Solvlet &s) const;

    virtual int getFrameID(void) const;
    virtual void setFrameID(int _id);

    virtual vector<LimbLabel> getLabels(void) const;
    virtual const vector<LimbLabel>* getLabelsPtr(void) const;
    virtual void setLabels(vector<LimbLabel> _labels);

    virtual Skeleton toSkeleton(const Skeleton &example);
    virtual float evaluateSolution(Frame* frame, map<string, float> params);

  private:
    int frameId;
    vector<LimbLabel> labels;
  };

}

#endif  // _SOLVLET_HPP_

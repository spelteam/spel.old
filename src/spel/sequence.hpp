#ifndef _SEQUENCE_HPP_
#define _SEQUENCE_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <string>
#include <math.h>

// Eigen3
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "spelHelper.hpp"
#include "interpolation.hpp"
#include "frame.hpp"

namespace SPEL
{
  using namespace std;
  using namespace Eigen;

  /// Used to evaluate accuracy of a detection
  class Sequence
  {
  public:
    Sequence(void);
    Sequence(const Sequence& seq);
    ///constructor
    Sequence(int idx, string seqName, vector<Frame*> seq);
    virtual ~Sequence(void);
    virtual string getName(void) const;
    virtual void setName(const string& _name);
    virtual int getID(void) const;
    virtual void setID(const int& _id);
    virtual vector<Frame*> getFrames(void) const;
    virtual void setFrames(const vector<Frame*> _frames);
    ///compute (or re-compute) interpolation for all frames which are not a keyframe or a lockframe
    virtual void computeInterpolation(map<string, float> &params);
    virtual void estimateUniformScale(map<string, float> &params);

  protected:
    virtual vector<Frame*> interpolateSlice(vector<Frame*> slice, map<string, float> params);
    virtual vector<Frame*> interpolateSlice2D(vector<Frame*> slice, map<string, float> params);
  private:
    /// detection score
    vector<Frame*> frames;
    ///sequence name
    string name;
    int id;
  };

}

#endif  // _SEQUENCE_HPP_


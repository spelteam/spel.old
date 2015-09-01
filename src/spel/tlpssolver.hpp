#ifndef _TLPSSOLVER_HPP_
#define _TLPSSOLVER_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <vector>
#include <opencv2/opencv.hpp>
#include <math.h>

//OpenGM
#include <opengm/graphicalmodel/graphicalmodel.hxx>
#include <opengm/graphicalmodel/graphicalmodel_hdf5.hxx>
#include <opengm/graphicalmodel/space/discretespace.hxx>
#include <opengm/operations/adder.hxx>
#include <opengm/functions/explicit_function.hxx>
#include <opengm/inference/messagepassing/messagepassing.hxx>
#include <opengm/operations/minimizer.hxx>

// Eigen3
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "solver.hpp"
#include "frame.hpp"
#include "lockframe.hpp"
#include "colorHistDetector.hpp"
#include "hogDetector.hpp"
#include "surfDetector.hpp"
#include "interpolation.hpp"

namespace SPEL
{
  using namespace std;
  using namespace opengm;
  using namespace Eigen;

  ///define the space and the model
  class TLPSSolver : public Solver
  {
  public:
    TLPSSolver(void);
    ///inherited virtual
    virtual ~TLPSSolver(void);
    ///inherited virtual
    virtual vector<Solvlet> solve(Sequence& frames);
    ///inherited virtual
    virtual vector<Solvlet> solve(Sequence& frames, map<string, float> params);
    virtual vector<Solvlet> solve(Sequence& frames, map<string, float> params, vector<Solvlet> solvlets);
    //INHERITED
    //public:
    // string getName(); //get the solver name. Every class inheriting solver has its own Name
    // string getId(); //get the solver Id. Every class inheriting solver has is own ID
  protected:

    virtual vector<Solvlet> solveWindowed(Sequence &sequence, map<string, float> params); //inherited virtual
    virtual vector<Solvlet> solveGlobal(Sequence &sequence, map<string, float> params); //inherited virtual

    virtual float evaluateSolution(Frame* frame, vector<LimbLabel> labels, map<string, float> params);

    virtual int findFrameIndexById(int id, vector<Frame*> frames);
    virtual float computeScoreCost(const LimbLabel& label, map<string, float> params);
    virtual float computeJointCost(const LimbLabel& child, const LimbLabel& parent, map<string, float> params, bool toChild);
    virtual float computeNormJointCost(const LimbLabel& child, const LimbLabel& parent, map<string, float> params, float jointMax, bool toChild);
    virtual float computePriorCost(const LimbLabel& label, const BodyPart& prior, const Skeleton& skeleton, map<string, float> params);
    virtual float computeNormPriorCost(const LimbLabel& label, const BodyPart& prior, const Skeleton& skeleton, map<string, float> params, float max);

    virtual float computePastTempCost(const LimbLabel& thisLabel, const LimbLabel& pastLabel, map<string, float> params);
    virtual float computeNormPastTempCost(const LimbLabel& thisLabel, const LimbLabel& pastLabel, map<string, float> params, float jointMax);
    virtual float computeFutureTempCost(const LimbLabel& thisLabel, const LimbLabel& futureLabel, map<string, float> params);
    virtual float computeNormFutureTempCost(const LimbLabel& thisLabel, const LimbLabel& futureLabel, map<string, float> params, float max);
    virtual float computeAnchorCost(const LimbLabel& thisLabel, Frame* anchor, map<string, float> params);
    virtual float computeNormAnchorCost(const LimbLabel& thisLabel, Frame* anchor, map<string, float> params, float jointMax);

    ///separate the sequence into slices, for temporal solve
    virtual vector<vector<Frame*> > slice(const vector<Frame*>& frames);

    //INHERITED
    //private:
    //int id;
    //string name;
  };

}

#endif  // _TLPSSOLVER_HPP_


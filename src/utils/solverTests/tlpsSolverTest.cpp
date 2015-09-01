#include <iostream>
#include <tlpssolver.hpp>
#include "projectLoader.hpp"

using namespace std;
using namespace SPEL;

int main (int argc, char **argv)
{
  if (argc != 3) 
  {
    cout << "Usage tlpsSolverTest [project.xml] [out directory]" << endl;
    return -1;
  }
  string curFolder = argv[1];
  curFolder = curFolder.substr(0, curFolder.find_last_of("/"));
  if (curFolder.back() != '/')
  {
    curFolder += '/';
  }

  ProjectLoader projectLoader(curFolder);

  cout << "Loading project..." << endl;
 
  if (projectLoader.Load(argv[1]) == true)
  {  
    cout << "Project was successfully loaded" << endl;
  }
  else
  {
    cout << "Project was not loaded" << endl;
    return -1;
  }
  
  map <string, float> params; //use the default params
  //params.emplace("debugLevel", 3); //set the debug setting to highest (0,1,2,3)
  params.emplace("tlpsLockframeThreshold", 0.0); // 0.0 avoiding the rejection of some TLPS solutions
  //global settings
  params.emplace("imageCoeff", 1.0); //set solver detector infromation sensitivity
  params.emplace("jointCoeff", 0.6); //set solver body part connectivity sensitivity
  params.emplace("priorCoeff", 0.0); //set solver distance to prior sensitivity
  params.emplace("tempCoeff", 0.1);

  //detector settings
  params.emplace("useCSdet", 0.1); //determine if ColHist detector is used and with what coefficient
  params.emplace("useHoGdet", 9.0); //determine if HoG descriptor is used and with what coefficient
  params.emplace("useSURFdet", 0.0); //determine whether SURF detector is used and with what coefficient

  params.emplace("grayImages", 1); // use grayscale images for HoG?

  //solver settings
  params.emplace("badLabelThresh", 0.45); //set bad label threshold, which will force solution discard at 0.45
  params.emplace("partDepthRotationCoeff", 1.25); //search radius increase for each depth level in the part tree

  params.emplace("anchorBindDistance", 0); //restrict search regions if within bind distance of existing keyframe or lockframe (like a temporal link
  params.emplace("anchorBindCoeff", 0.3); //multiplier for narrowing the search range if close to an anchor (lockframe/keyframe)
  params.emplace("bindToLockframes", 0); //should binds be also used on lockframes?

  params.emplace("maxFrameHeight", 288); //scale to 288p - same size as trijump video seq, for detection
  params.emplace("maxPartCandidates", 0.1); //set the max number of part candidates to allow into the solver
  params.emplace("uniqueLocationCandidates", 360);
  params.emplace("debugLevel", 0);
  vector<Solvlet> solve;
  
  vector <Frame*> vFrames = projectLoader.getFrames();
  Sequence seq(0, "test", vFrames);

  //now test inrepolation for this sequence
  seq.estimateUniformScale(params);
  seq.computeInterpolation(params);

  vFrames.clear();

  //first, test the 3D locations
  for(uint32_t i=0; i<vFrames.size(); ++i)
  {
      if(vFrames[i]->getFrametype()==KEYFRAME)
        cout << vFrames[i]->getID() << " KEYFRAME " << endl;
      else
          cout << vFrames[i]->getID() << endl;

      tree <BodyJoint> jointTree = vFrames[i]->getSkeleton().getJointTree();
      tree <BodyJoint>::iterator iter;
      for(iter=jointTree.begin(); iter!=jointTree.end(); ++iter)
      {
          Point3f spaceLoc = iter->getSpaceLocation();
          cout << "\t id: " << iter->getLimbID() << " x: " << spaceLoc.x
               << " y: " << spaceLoc.y << " z: " << spaceLoc.z << endl;
      }
  }

  TLPSSolver tSolver;

  cout << "Solving using TLPSSolver..." << endl;
  //solve with some default params
  solve = tSolver.solve(seq, params);

  for(uint32_t i=0; i<solve.size();++i)
  {
    projectLoader.drawFrameSolvlets(solve[i], vFrames[solve[i].getFrameID()], argv[2], Scalar(0,0,255), 2);
  }

  return 0;
}


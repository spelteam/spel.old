#include <iostream>
#include <nskpsolver.hpp>
#include <tlpssolver.hpp>
#include "projectLoader.hpp"

using namespace std;
using namespace SPEL;

int main (int argc, char **argv)
{
    if (argc != 3)
    {
        cout << "Usage hybridSolverTest [project.xml] [out directory]" << endl;
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
    params.emplace("debugLevel", 1); //set the debug setting to highest (0,1,2,3)

    vector <Frame*> vFrames = projectLoader.getFrames();
    Sequence seq(0, "test", vFrames);

    //now test inrepolation for this sequence
    seq.estimateUniformScale(params);
    seq.computeInterpolation(params);

    vFrames.clear();
    vFrames = seq.getFrames();

    NSKPSolver nSolver;
    TLPSSolver tSolver;
    cout << "Solving using NSKPSolver..." << endl;
    //solve with some default params
    //ImageSimilarityMatrix ism(vFrames);
    ImageSimilarityMatrix ism;
    if(!ism.read("testISM.ism"))
    {
        ism.buildImageSimilarityMatrix(vFrames);
        ism.write(("testISM.ism"));
    }
    //global settings
    params.emplace("imageCoeff", 1.0); //set solver detector infromation sensitivity
    params.emplace("jointCoeff", 0.6); //set solver body part connectivity sensitivity
    params.emplace("priorCoeff", 0.0); //set solver distance to prior sensitivity

    //detector settings
    params.emplace("useCSdet", 0.1); //determine if ColHist detector is used and with what coefficient
    params.emplace("useHoGdet", 1.0); //determine if HoG descriptor is used and with what coefficient
    params.emplace("useSURFdet", 0.0); //determine whether SURF detector is used and with what coefficient

    params.emplace("grayImages", 1); // use grayscale images for HoG?

    //solver settings
    params.emplace("nskpIters", 0); //do as many NSKP iterations as is useful at each run
    params.emplace("acceptLockframeThreshold", 0.52); // 0.52 set the threshold for NSKP and TLPSSolvers, forcing TLPS to reject some solutions
    params.emplace("badLabelThresh", 0.45); //set bad label threshold, which will force solution discard at 0.45
    params.emplace("partDepthRotationCoeff", 1.25); //search radius increase for each depth level in the part tree

    params.emplace("anchorBindDistance", 0); //restrict search regions if within bind distance of existing keyframe or lockframe (like a temporal link
    params.emplace("anchorBindCoeff", 0.3); //multiplier for narrowing the search range if close to an anchor (lockframe/keyframe)
    params.emplace("bindToLockframes", 0); //should binds be also used on lockframes?

    vector<Solvlet> finalSolve;
    int prevSolveSize=0;
    do
    {
        prevSolveSize=finalSolve.size(); //set size of the final solve

        vector<Solvlet> nskpSolve, tlpsSolve;
        //do an iterative NSKP solve
        nskpSolve = nSolver.solve(seq, params, ism);

        //draw the solution
        for(uint32_t i=0; i<nskpSolve.size();++i)
        {
            Frame* frame = seq.getFrames()[nskpSolve[i].getFrameID()];
            Frame* parent = seq.getFrames()[frame->getParentFrameID()];

            projectLoader.drawLockframeSolvlets(ism, nskpSolve[i], frame, parent, argv[2], Scalar(0,0,255), 1);
        }

        for(vector<Solvlet>::iterator s=nskpSolve.begin(); s!=nskpSolve.end(); ++s)
            finalSolve.push_back(*s);

        //then, do a temporal solve
        seq.computeInterpolation(params); //recompute interpolation (does this improve results?)

        tlpsSolve = tSolver.solve(seq, params);

        for(uint32_t i=0; i<tlpsSolve.size();++i)
        {
            Frame* frame = seq.getFrames()[tlpsSolve[i].getFrameID()];
            Frame* parent = seq.getFrames()[frame->getParentFrameID()];

            projectLoader.drawLockframeSolvlets(ism, tlpsSolve[i], frame, parent, argv[2], Scalar(0,0,255), 1);
        }

        for(vector<Solvlet>::iterator s=tlpsSolve.begin(); s!=tlpsSolve.end(); ++s)
            finalSolve.push_back(*s);

    } while(finalSolve.size()>prevSolveSize);


//    //draw the solution
//    for(uint32_t i=0; i<finalSolve.size();++i)
//    {
//        Frame* frame = seq.getFrames()[finalSolve[i].getFrameID()];
//        Frame* parent = seq.getFrames()[frame->getParentFrameID()];

//        projectLoader.drawLockframeSolvlets(ism, finalSolve[i], frame, parent, argv[2], Scalar(0,0,255), 1);
//    }


//    for(uint32_t i=0; i<solve.size();++i)
//    {
//        Frame* frame = vFrames[solve[i].getFrameID()];

//        projectLoader.drawFrameSolvlets(solve[i], frame, argv[2], Scalar(0,0,255), 1);
//    }

    return 0;
}


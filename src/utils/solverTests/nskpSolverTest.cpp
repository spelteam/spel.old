#include <iostream>
#include <thread>
#include <nskpsolver.hpp>
#include <tlpssolver.hpp>
#include "projectLoader.hpp"

using namespace std;
using namespace SPEL;

int main (int argc, char **argv)
{
    ios_base::sync_with_stdio(false); //turn off syncing of stdio for async operation
    unsigned int n = std::thread::hardware_concurrency();
    std::cout << n << " concurrent threads are supported.\n";

    if (argc != 3)
    {
        cout << "Usage nskpSolverTest [project.xml] [out directory]" << endl;
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

    vector<Solvlet> solve;

    vector <Frame*> vFrames = projectLoader.getFrames();
    Sequence seq(0, "test", vFrames);

    //now test inrepolation for this sequence
    seq.estimateUniformScale(params);
    seq.computeInterpolation(params);

    vFrames.clear();

    vFrames = seq.getFrames();

    NSKPSolver nSolver;
    //TLPSSolver tSolver;

    //global settings
    params.emplace("imageCoeff", 1.0); //set solver detector infromation sensitivity
    params.emplace("jointCoeff", 0.6); //set solver body part connectivity sensitivity
    params.emplace("priorCoeff", 0.0); //set solver distance to prior sensitivity

    //detector settings
    params.emplace("useCSdet", 0.1); //determine if ColHist detector is used and with what coefficient
    params.emplace("useHoGdet", 9.0); //determine if HoG descriptor is used and with what coefficient
    params.emplace("useSURFdet", 0.0); //determine whether SURF detector is used and with what coefficient

    params.emplace("grayImages", 1); // use grayscale images for HoG?

    //solver settings
    params.emplace("nskpIters", 0); //do as many NSKP iterations as is useful at each run
    params.emplace("nskpLockframeThreshold", 0.52); // 0.52 set the threshold for NSKP and TLPSSolvers, forcing TLPS to reject some solutions
    params.emplace("badLabelThresh", 0.45); //set bad label threshold, which will force solution discard at 0.45
    params.emplace("partDepthRotationCoeff", 1.25); //search radius increase for each depth level in the part tree

    params.emplace("anchorBindDistance", 0); //restrict search regions if within bind distance of existing keyframe or lockframe (like a temporal link
    params.emplace("anchorBindCoeff", 0.3); //multiplier for narrowing the search range if close to an anchor (lockframe/keyframe)
    params.emplace("bindToLockframes", 0); //should binds be also used on lockframes?

    params.emplace("maxFrameHeight", 288); //scale to 288p - same size as trijump video seq, for detection
    params.emplace("maxPartCandidates", 0.1); //set the max number of part candidates to allow into the solver

    cout << "Solving using NSKPSolver..." << endl;
    //solve with some default params
    //ImageSimilarityMatrix ism(vFrames);
    ImageSimilarityMatrix ism;
    string ismFile(projectLoader.getProjectTitle()+".ism");
    if(!ism.read(ismFile))
    {
        ism.buildImageSimilarityMatrix(vFrames, 288);
        ism.write(ismFile);
        exit(0);
    }

    float mean = ism.mean();
    float sd = ism.stddev();
    float min = ism.min();

    cout << "ISM Mean " << mean << " sd " << sd << " min " << min << endl;

    cout << "The min is " << (mean-min)/sd << " deviations away from mean. " << endl;
    cout << "One sd is " << sd/min << " of min." << endl;

    float simThresh = 1.0+3.5*sd/min;

    cout << "Seeting mstThresh to " << simThresh << endl;

    params.emplace("mstThresh", 1.0+3.5*sd/min); //set similarity as multiple of minimum, MUST be >=1
    //params.emplace("treeSize", 5); //no size limit

    solve = nSolver.solve(seq, params, ism);

    //draw the solution
    for(uint32_t i=0; i<solve.size();++i)
    {
        Frame* frame = seq.getFrames()[solve[i].getFrameID()];
        Frame* parent = seq.getFrames()[frame->getParentFrameID()];

        projectLoader.drawLockframeSolvlets(ism, solve[i], frame, parent, argv[2], Scalar(0,0,255), 1);
    }

    return 0;
}


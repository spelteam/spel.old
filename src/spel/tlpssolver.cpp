#include "tlpssolver.hpp"

namespace SPEL
{
TLPSSolver::TLPSSolver(void)
{
    id = 0;
    name = "TLPS";
}

TLPSSolver::~TLPSSolver(void)
{

}

vector<Solvlet> TLPSSolver::solve(Sequence &frames) //inherited virtual
{
    map<string, float> params; //set the default parameters vector

    //set some parameter deefaults
    params.at("scoreIndex") = 0;

    //pass to next level solve function, for ISM computing

    return this->solve(frames, params);
}

vector<Solvlet> TLPSSolver::solve(Sequence &sequence, map<string, float> params, vector<Solvlet> solvlets) //inherited virtual
{
    vector<Solvlet> tlpsSolvlets = solve(sequence, params); //inherited virtual

    for(auto&& s:solvlets)
        tlpsSolvlets.push_back(s);

    sort(tlpsSolvlets.begin(), tlpsSolvlets.end());

    return tlpsSolvlets;

}

vector<Solvlet> TLPSSolver::solve(Sequence &sequence, map<string, float> params) //inherited virtual
{
    if (sequence.getFrames().size() == 0)
        return vector<Solvlet>();

    Mat image(sequence.getFrames()[0]->getImage());
    //the params vector should contain all necessary parameters, if a parameter is not present, default values should be used
    params.emplace("debugLevel", 1); //set up the lockframe accept threshold by mask coverage
    params.emplace("temporalWindowSize", 0); //0 for unlimited window size

    //detector enablers
    params.emplace("useCSdet", 1.0); //determine if ColHist detector is used and with what coefficient
    params.emplace("useHoGdet", 0.0); //determine if HoG descriptor is used and with what coefficient
    params.emplace("useSURFdet", 0.0); //determine whether SURF detector is used and with what coefficient
    params.emplace("maxPartCandidates", 1.0); //set the max number of part candidates to allow into the solver

    //detector search parameters
    params.emplace("baseRotationRange", 40); //search angle range of +/- 60 degrees
    float baseRotationRange = params.at("baseRotationRange");
    params.emplace("baseRotationStep", baseRotationRange / 4.0); //search with angle step of 10 degrees

    params.emplace("baseSearchRadius", image.rows / 30.0); //search a radius of 100 pixels
    int baseSearchRadius = params.at("baseSearchRadius");
    params.emplace("baseSearchStep", baseSearchRadius / 10.0); //search in a grid every 10 pixels

    params.emplace("partShiftCoeff", 1.5); //search radius multiplier of distance between part in current and prev frames
    params.emplace("partRotationCoeff", 1.5); //rotation radius multiplier of distance between part in current and prev frames

    //solver sensitivity parameters
    params.emplace("imageCoeff", 1.0); //set solver detector infromation sensitivity
    params.emplace("jointCoeff", 0.5); //set solver body part connectivity sensitivity
    params.emplace("jointLeeway", 0.05); //set solver lenience for body part disconnectedness, as a percentage of part length
    params.emplace("tempCoeff", 0.1); //set the temporal link coefficient
    //params.emplace("priorCoeff", 0.0); //set solver distance to prior sensitivity NOT CURRENTLY USED
    //params.emplace("anchorCoeff", 0.0); //set the anchor coefficient cost NOT CURRENTLY USED

    //solver eval parameters
    params.emplace("tlpsLockframeThreshold", 0.52); //set up the lockframe accept threshold by mask coverage

    sequence.estimateUniformScale(params);
    sequence.computeInterpolation(params);

    vector<Solvlet> solution;
    //call the new function
    if (params.at("temporalWindowSize") == 0)
        solution = solveGlobal(sequence, params);
    else
        solution = solveWindowed(sequence, params);

    return solution;
}

vector<Solvlet> TLPSSolver::solveGlobal(Sequence &sequence, map<string, float> params) //inherited virtual
{

    float baseRotationStep = params.at("baseRotationStep");

    float partShiftCoeff = params.at("partShiftCoeff");
    float partRotationCoeff = params.at("partRotationCoeff");

    float depthRotationCoeff = params.at("partDepthRotationCoeff");
    float baseRotationRange = params.at("baseRotationRange");
    float baseSearchRadius = params.at("baseSearchRadius");

    float useHoG = params.at("useHoGdet");
    float useCS = params.at("useCSdet");
    float useSURF = params.at("useSURFdet");
    uint32_t debugLevel = params.at("debugLevel");
    //first slice up the sequences
    if (debugLevel >= 1)
        cout << "TLPSSolver started, slicing sequence..." << endl;

    vector<vector<Frame*> > slices = slice(sequence.getFrames());

    if (debugLevel >= 1)
        cout << slices.size() << " sequence slices created." << endl;

    vector<vector<Solvlet> > sequenceSolvlets; //one vector of solvlets per slice

    if (debugLevel >= 1)
        cout << "Solving slices..." << endl;


    for (uint32_t sliceNumber = 0; sliceNumber < slices.size(); ++sliceNumber)
    {
        ///define the space
        typedef opengm::DiscreteSpace<> Space;
        ///define the model
        typedef opengm::GraphicalModel<float, opengm::Adder, opengm::ExplicitFunction<float>, Space> Model;

        ///define the update rules
        typedef BeliefPropagationUpdateRules<Model, opengm::Minimizer> UpdateRules;
        ///define the inference algorithm
        typedef MessagePassing<Model, opengm::Minimizer, UpdateRules, opengm::MaxDistance> BeliefPropagation;

        //for every slice, build a factor grph
        if (debugLevel >= 1)
            cout << "Interpolating slice " << sliceNumber << endl;
        vector<Frame*> seqSlice = slices[sliceNumber]; //the slice we are working with, interpolated


        vector<Detector*> detectors;
        if (useCS)
            detectors.push_back(new ColorHistDetector());
        if (useHoG)
            detectors.push_back(new HogDetector());
        if (useSURF)
            detectors.push_back(new SurfDetector());

        vector<Frame*> trainingFrames;
        trainingFrames.push_back(seqSlice.front()); //set training frame by index
        trainingFrames.push_back(seqSlice.back());

        for (uint32_t i = 0; i < detectors.size(); ++i)
        {
            detectors[i]->train(trainingFrames, params);
        }

        vector<map<uint32_t, vector<LimbLabel> > > detections; //numbers of labels per part, per frame, for this slice

        //initialise first level
        for (uint32_t i = 0; i < seqSlice.size(); ++i) // access by reference, the type of i is int&
        {
            //init detections to contain all detects for the sequence slice
            detections.push_back(map<uint32_t, vector<LimbLabel> >());
        }

        //first do the detections, and store them

        for (uint32_t currentFrame = 1; currentFrame < seqSlice.size() - 1; ++currentFrame) //for every frame but first and last
        {
            //current frame represents the current frame, previous frame is currentFrame-1, next frame is currentFrame+1

            cerr << "Detecting on frame " << seqSlice[currentFrame]->getID() << endl;

            map<uint32_t, vector<LimbLabel> > labels;

            Skeleton skeleton = seqSlice[currentFrame]->getSkeleton();

            //now set up skeleton params, such as search radius and angle search radius for every part
            //this should very depending on relative distance between frames
            //for each body part
            tree<BodyPart> partTree = skeleton.getPartTree();
            tree<BodyPart>::iterator partIter;

            for(partIter=partTree.begin(); partIter!=partTree.end(); ++partIter)
            {
                //for each bodypart, establish the angle variation and the search distance, based on distance from parent frame
                //and based on node depth (deeper nodes have a higher distance)
                //this should rely on parameters e.g.


                //else
                int depth = partTree.depth(partIter);

                float rotationRange = baseRotationRange;//*pow(depthRotationCoeff, depth);
                float searchRange = baseSearchRadius*pow(depthRotationCoeff, depth);

                if(partTree.number_of_children(partIter)==0)
                {
                    searchRange=searchRange*2;
                    rotationRange=rotationRange*depthRotationCoeff;
                }


                //                if(isBound) //if we're close to the anchor, restrict the rotation range
                //                    rotationRange = rotationRange*anchorBindCoeff;

                partIter->setRotationSearchRange(rotationRange);
                partIter->setSearchRadius(searchRange);

            }

            skeleton.setPartTree(partTree);
            seqSlice[currentFrame]->setSkeleton(skeleton);

            for (uint32_t i = 0; i < detectors.size(); ++i) //for every detector
            {
                labels = detectors[i]->detect(seqSlice[currentFrame], params, labels); //detect labels based on keyframe training
            }            

            float maxPartCandidates=params.at("maxPartCandidates");
            //now take the top percentage of all labels
            for (uint32_t i = 0; i < labels.size(); ++i) //for each part
            {
                vector<Score> scores = labels[i].at(0).getScores();

                uint32_t isWeak=0;
                for(uint32_t j=0; j<scores.size(); ++j)
                {
                    if(scores[j].getIsWeak())
                        isWeak++;
                }
                //if both scores are weak
//                if(isWeak!=scores.size() && !labels[i].at(0).getIsOccluded()) //if not all scores are weak, and the part is NOT occluded filter
//                {
                    vector<LimbLabel> tmp;
                    for(uint32_t j=0; j<labels[i].size()*maxPartCandidates;++j) //for each label that is within the threshold
                    {
                        tmp.push_back(labels[i].at(j)); //push back the label
                    }
                    labels[i] = tmp; //set this part's candidates to the new trimmed vector
//                } //otherwise keep all samples, but ignore later
            }

            detections[currentFrame] = labels; //store all detections into detections
        }

        vector<size_t> numbersOfLabels; //numbers of labels per part

        //the first an last frames of detections are always empty

        for (uint32_t i = 0; i < detections.size(); ++i)
        {
            for (uint32_t j = 0; j < detections[i].size(); ++j)
            {
                numbersOfLabels.push_back(detections[i][j].size());
            }
        } //numbers of labels now contains the numbers of labels, and their respective variations

        //use this to shape the space, this will be rather large
        Space space(numbersOfLabels.begin(), numbersOfLabels.end());
        Model gm(space);

        cerr << "Computing slice factors..." << endl;

        int suppFactors = 0, jointFactors = 0, anchorFactors = 0, tempFactors = 0;
        //now do the factors
        for (uint32_t currentFrame = 1; currentFrame < seqSlice.size() - 1; ++currentFrame) //for every frame but first and last
        {
            Skeleton tempSkel = seqSlice[currentFrame]->getSkeleton(); //this frame interpolated skeleton
            Skeleton pSkel = seqSlice[currentFrame - 1]->getSkeleton(); //previous frame interpolated skeleton
            tree<BodyPart> partTree = tempSkel.getPartTree();
            tree<BodyPart> prevPartTree = pSkel.getPartTree();
            tree<BodyPart>::iterator partIter, parentPartIter, prevPartIter;

            partIter = partTree.begin();
            prevPartIter = prevPartTree.begin();

            while (partIter != partTree.end() && prevPartIter != prevPartTree.end())
            {
                //compute prev part angle
                Point2f p0, p1, c0, c1;
                p0 = pSkel.getBodyJoint(prevPartIter->getParentJoint())->getImageLocation();
                p1 = pSkel.getBodyJoint(prevPartIter->getChildJoint())->getImageLocation();

                c0 = tempSkel.getBodyJoint(partIter->getParentJoint())->getImageLocation();
                c1 = tempSkel.getBodyJoint(partIter->getChildJoint())->getImageLocation();
                //compute prev part centre shift
                Point2f partCentre = 0.5*c0 + 0.5*c1;
                Point2f prevPartCentre = 0.5*p0 + 0.5*p1;

                Point2f partShift = partCentre - prevPartCentre;

                //set the search range based on distance body part centre has moved between previous frame and this frame
                float searchRange = sqrt(partShift.x*partShift.x + partShift.y*partShift.y)*partShiftCoeff;

                //bow compute the rotation angle change
                float partAngle = float(spelHelper::angle2D(1.0, 0, c1.x - c0.x, c1.y - c0.y) * (180.0 / M_PI));
                float prevAngle = float(spelHelper::angle2D(1.0, 0, p1.x - p0.x, p1.y - p0.y) * (180.0 / M_PI));

                //set search radius based on part rotation between previous frame and this frame
                float rotationRange = (partAngle - prevAngle)*partRotationCoeff;

                partIter->setRotationSearchRange(rotationRange);
                partIter->setSearchRadius(searchRange);

                partIter++;
                prevPartIter++;
            }

            tempSkel.setPartTree(partTree); //set the part tree for the skel
            seqSlice[currentFrame]->setSkeleton(tempSkel); //set the skeleton for the frame

            //construct the image score cost factors
            //label score cost
            cerr << "Computing Factors at Frame " << seqSlice[currentFrame]->getID() << endl;
            auto labels = detections[currentFrame];
            for (partIter = partTree.begin(); partIter != partTree.end(); ++partIter) //for each of the detected parts
            {
                vector<Score> scores = labels[partIter->getPartID()].at(0).getScores();
                uint32_t isWeakCount=0;
                for(uint32_t j=0; j<scores.size(); ++j)
                {
                    if(scores[j].getIsWeak())
                        isWeakCount++;
                }

                vector<int> varIndices; //create vector of indices of variables
                int varID = (currentFrame - 1)*partTree.size() + partIter->getPartID();
                varIndices.push_back(varID); //push first value in
                size_t scoreCostShape[] = { labels[partIter->getPartID()].size() }; //number of labels

//                if(isWeakCount!=scores.size() && !labels[partIter->getPartID()].at(0).getIsOccluded()) //if the support score for this part is not weak
//                {
                    ExplicitFunction<float> scoreCostFunc(scoreCostShape, scoreCostShape + 1); //explicit function declare
                    for (uint32_t i = 0; i < labels[partIter->getPartID()].size(); ++i) //for each label in for this part
                    {
                        float cost = computeScoreCost(labels[partIter->getPartID()].at(i), params); //compute the label score cost
                        scoreCostFunc(i) = cost;
                    }
                    Model::FunctionIdentifier scoreFid = gm.addFunction(scoreCostFunc); //explicit function add to graphical model
                    gm.addFactor(scoreFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
                    suppFactors++;
//                }

                if (currentFrame == 1) //anchor to first skeleton in sequence
                {
                    float anchorMin = FLT_MAX, anchorMax = 0;
                    for (uint32_t i = 0; i < labels[partIter->getPartID()].size(); ++i)
                    {
                        float val = computeAnchorCost(labels[partIter->getPartID()].at(i), seqSlice[0], params);

                        if (val < anchorMin)
                            anchorMin = val;
                        if (val > anchorMax && val != FLT_MAX)
                            anchorMax = val;
                    }

                    //we already know the shape from the previous functions
                    ExplicitFunction<float> anchorCostFunc(scoreCostShape, scoreCostShape + 1); //explicit function declare

                    for (uint32_t i = 0; i < labels[partIter->getPartID()].size(); ++i) //for each label in for this part
                    {
                        float cost = computeNormAnchorCost(labels[partIter->getPartID()].at(i), seqSlice[0], params, anchorMax);
                        anchorCostFunc(i) = cost;
                    }

                    Model::FunctionIdentifier anchorFid = gm.addFunction(anchorCostFunc); //explicit function add to graphical model
                    gm.addFactor(anchorFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
                    anchorFactors++;
                }

                //anchor to last frame of the slice (i.e. to frame=seqSlice.size()-1
                if (currentFrame == (seqSlice.size() - 2)) //anchor to
                {
                    float anchorMin = FLT_MAX, anchorMax = 0;
                    for (uint32_t i = 0; i < labels[partIter->getPartID()].size(); ++i)
                    {
                        float val = computeAnchorCost(labels[partIter->getPartID()].at(i), seqSlice[seqSlice.size() - 1], params);

                        if (val < anchorMin)
                            anchorMin = val;
                        if (val > anchorMax && val != FLT_MAX)
                            anchorMax = val;
                    }
                    //we already know the shape from the previous functions
                    ExplicitFunction<float> anchorCostFunc(scoreCostShape, scoreCostShape + 1); //explicit function declare

                    for (uint32_t i = 0; i < labels[partIter->getPartID()].size(); ++i) //for each label in for this part
                    {
                        float cost = computeNormAnchorCost(labels[partIter->getPartID()].at(i), seqSlice[seqSlice.size() - 1], params, anchorMax);
                        anchorCostFunc(i) = cost;
                    }

                    Model::FunctionIdentifier anchorFid = gm.addFunction(anchorCostFunc); //explicit function add to graphical model
                    gm.addFactor(anchorFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
                    anchorFactors++;
                }

                if (partIter != partTree.begin()) //if iterator is not on root node, there is always a parent body part
                {
                    //this factor needs to be in reverse order, for sorting purposes
                    varIndices.clear();
                    parentPartIter = partTree.parent(partIter); //find the parent of this part
                    varIndices.push_back((currentFrame - 1)*partTree.size() + parentPartIter->getPartID()); //push back parent partID as the second variable index
                    varIndices.push_back(varID); //push first value in (parent, this)

                    size_t jointCostShape[] = { labels[parentPartIter->getPartID()].size(), labels[partIter->getPartID()].size() }; //number of labels
                    ExplicitFunction<float> jointCostFunc(jointCostShape, jointCostShape + 2); //explicit function declare

                    //first figure out which of the current body part's joints should be in common with the parent body part

                    bool toChild = false;
                    int pj = parentPartIter->getChildJoint();
                    int cj = partIter->getParentJoint();
                    if (pj == cj)
                    {
                        //then current parent is connected to paren
                        toChild = true;
                    }

                    float jointMin = FLT_MAX, jointMax = 0;
                    for (uint32_t i = 0; i < labels[partIter->getPartID()].size(); ++i) //for each label in for this part
                    {
                        for (uint32_t j = 0; j < labels[parentPartIter->getPartID()].size(); ++j)
                        {
                            float val = computeJointCost(labels[partIter->getPartID()].at(i), labels[parentPartIter->getPartID()].at(j), params, toChild);

                            if (val < jointMin)
                                jointMin = val;
                            if (val > jointMax && val != FLT_MAX)
                                jointMax = val;
                        }
                    }

                    for (uint32_t i = 0; i < labels[partIter->getPartID()].size(); ++i) //for each label in for this part
                    {
                        for (uint32_t j = 0; j < labels[parentPartIter->getPartID()].size(); ++j)
                        {
                            //for every child/parent pair, compute score
                            float cost = computeNormJointCost(labels[partIter->getPartID()].at(i), labels[parentPartIter->getPartID()].at(j), params, jointMax, toChild);
                            jointCostFunc(j, i) = cost;
                        }
                    }

                    Model::FunctionIdentifier jointFid = gm.addFunction(jointCostFunc); //explicit function add to graphical model
                    gm.addFactor(jointFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
                    jointFactors++;
                }

                //futre temporal link (if not anchored)
                if (currentFrame < seqSlice.size() - 2)
                {

                    float tempMin = FLT_MAX, tempMax = 0;
                    for (uint32_t i = 0; i < labels[partIter->getPartID()].size(); ++i) //for each label in for this part
                    {
                        for (uint32_t j = 0; j < detections[currentFrame + 1][partIter->getPartID()].size(); ++j)
                        {
                            float val = computeFutureTempCost(labels[partIter->getPartID()].at(i), detections[currentFrame + 1][partIter->getPartID()].at(j), params);

                            if (val < tempMin)
                                tempMin = val;
                            if (val > tempMax && val != FLT_MAX)
                                tempMax = val;
                        }
                    }

                    varIndices.clear();
                    varIndices.push_back((currentFrame - 1)*partTree.size() + partIter->getPartID()); //push back parent partID as the second variable index
                    varIndices.push_back((currentFrame)*partTree.size() + partIter->getPartID());

                    size_t futureCostShape[] = { labels[partIter->getPartID()].size(), detections[currentFrame + 1][partIter->getPartID()].size() }; //number of labels
                    ExplicitFunction<float> futureTempCostFunc(futureCostShape, futureCostShape + 2); //explicit function declare

                    for (uint32_t i = 0; i < labels[partIter->getPartID()].size(); ++i) //for each label in for this part
                    {
                        for (uint32_t j = 0; j < detections[currentFrame + 1][partIter->getPartID()].size(); ++j)
                        {
                            float cost = computeNormFutureTempCost(labels[partIter->getPartID()].at(i), detections[currentFrame + 1][partIter->getPartID()].at(j), params, tempMax);
                            futureTempCostFunc(i, j) = cost;
                        }
                    }
                    Model::FunctionIdentifier futureTempCostFid = gm.addFunction(futureTempCostFunc); //explicit function add to graphical model
                    gm.addFactor(futureTempCostFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
                    tempFactors++;
                }
            }
        }

        if (debugLevel >= 0)
        {
            float n, k;
            n = seqSlice.size() - 2; //num non-anchor frames
            k = seqSlice.front()->getSkeleton().getPartTree().size(); //number of bones
            //float expectedSuppFactors = n*k; //n*k
            float expectedJointFactors = n*(k - 1); //n*(k-1)
            float expectedAnchorFactors = 2 * k; //2*k
            float expectedTempFactors = (n - 1)*k; //(n-1)*k

            //assert(expectedSuppFactors == suppFactors);
            assert(expectedJointFactors == jointFactors);
            assert(expectedAnchorFactors == anchorFactors);
            assert(expectedTempFactors == tempFactors);
        }

        cerr << "Solving slice factor graph" << endl;

        //now solve the slice

        if (debugLevel >= 2)
            opengm::hdf5::save(gm, "gm.h5", "tlps-gm");

        const size_t maxNumberOfIterations = 100;
        const double convergenceBound = 1e-7;
        const double damping = 0.5;
        BeliefPropagation::Parameter parameter(maxNumberOfIterations, convergenceBound, damping);
        BeliefPropagation bp(gm, parameter);

        // optimize (approximately)
        BeliefPropagation::VerboseVisitorType visitor;
        //bp.infer(visitor);
        bp.infer();
        //pass to solve function

        // obtain the (approximate) argmin
        vector<size_t> labeling((detections.size() - 2)*detections[1].size()); //number of non-anchor frames * number of bodyparts
        bp.arg(labeling);

        vector<vector<LimbLabel> > solutionLabels;
        for (uint32_t i = 0; i < seqSlice.size(); ++i)
        {
            solutionLabels.push_back(vector<LimbLabel>());
            // for(uint32_t j=0; j<detections[i].size(); ++j)
            // {
            // 	solutionLabels[i].push_back(
            // }
        }

        for (uint32_t i = 1; i < seqSlice.size() - 1; ++i) //frames
        {
            for (uint32_t j = 0; j < detections[i].size(); ++j) //parts
            {
                int solveId = (i - 1)*detections[1].size() + j; //where to find the solution?
                solutionLabels[i].push_back(detections[i][j][labeling[solveId]]); //pupulate solution vector
            }
        }

        vector<Solvlet> solvlets;
        //now set up a solvlet for every frame
        for (uint32_t i = 1; i < seqSlice.size() - 1; ++i) //push all frames but keyframes into the solvlets
        {
            solvlets.push_back(Solvlet(seqSlice[i]->getID(), solutionLabels[i]));
        }

        sequenceSolvlets.push_back(solvlets);

        //do detector cleanup
        for(auto i=0; i<detectors.size(); ++i)
            delete detectors[i];
        detectors.clear();
    }


    if (debugLevel >= 1)
        cout << sequenceSolvlets.size() << " slices solved." << endl;
    //sequence solvlets now contain all the solvlets for the entire sequence

    //rearrange this into one vector of solvlets for evaluation
    vector<Solvlet> retSolve;
    for (uint32_t i = 0; i < sequenceSolvlets.size(); ++i)
    {
        for (uint32_t j = 0; j < sequenceSolvlets[i].size(); ++j)
        {
            retSolve.push_back(sequenceSolvlets[i][j]);
        }
    }

    vector<Frame*> frames = sequence.getFrames();

    float acceptLockframeThreshold = params.at("tlpsLockframeThreshold");
    if (acceptLockframeThreshold > 0) //if it's greater than zero, we want to evaluate solves before returning them
    {
        vector<Solvlet> passedSolves;
        for (uint32_t i = 0; i < retSolve.size(); ++i)
        {
            Solvlet solvlet = retSolve[i];
            float score = evaluateSolution(frames[retSolve[i].getFrameID()],
                    solvlet.getLabels(), params);

            if (score >= acceptLockframeThreshold) //if the frame passed the test
            {
                passedSolves.push_back(solvlet);
                //generate lockframe

                //set lockframe to frame
                int thisFrameID = solvlet.getFrameID();
                int parentFrameID = thisFrameID;//this frame is it's own parent frame, since this is a temporal solve

                Lockframe * lockframe = new Lockframe();
                lockframe->setImage(frames[thisFrameID]->getImage());
                lockframe->setMask(frames[thisFrameID]->getMask());
                lockframe->setID(thisFrameID);
                Skeleton parentSkel = frames[parentFrameID]->getSkeleton(); //use skeleton to mimic the structure of
                Skeleton skel = solvlet.toSkeleton(parentSkel);
                //parent skeleton is the basis, this will copy scale and other params, but have different locations
                lockframe->setSkeleton(skel);
                lockframe->setParentFrameID(parentFrameID);

                delete frames[thisFrameID]; //remove the existing frame
                frames[thisFrameID] = lockframe;
            }
        }
        retSolve = passedSolves; //return these solves
    }

    sequence.setFrames(frames);
    return retSolve;
}


vector<Solvlet> TLPSSolver::solveWindowed(Sequence &sequence, map<string, float> params) //inherited virtual
{
    ///define the space
    typedef opengm::DiscreteSpace<> Space;
    ///define the model
    typedef opengm::GraphicalModel<float, opengm::Adder, opengm::ExplicitFunction<float>, Space> Model;

    ///define the update rules
    typedef BeliefPropagationUpdateRules<Model, opengm::Minimizer> UpdateRules;
    ///define the inference algorithm
    typedef MessagePassing<Model, opengm::Minimizer, UpdateRules, opengm::MaxDistance> BeliefPropagation;

    int tempWindowSize = params.at("temporalWindowSize");
    float baseRotationStep = params.at("baseRotationStep");

    float partShiftCoeff = params.at("partShiftCoeff");
    float partRotationCoeff = params.at("partRotationCoeff");

    float useHoG = params.at("useHoGdet");
    float useCS = params.at("useCSdet");
    float useSURF = params.at("useSURFdet");
    uint32_t debugLevel = params.at("debugLevel");
    //first slice up the sequences

    vector<Solvlet> solution;

    //set solution

    return solution;
}

float TLPSSolver::evaluateSolution(Frame* frame, vector<LimbLabel> labels, map<string, float> params)
{
    // /*
    //   There should clearly be several factors that affect the outcome of an evaluation:
    //   1) Mask coverage
    //   2) Parts falling outside mask range
    //   3) ?

    //   */

    //engaged pixles - the total number of pixels that are either within a limb label of within the mask
    //correct pixels - those pixles that are black and outside a label, and white and inside a label
    //incorrect pixels - those pixels that are black and inside a label, and white and outside a label
    //score = correct/(correct+incorrect)

    //emplace defaults
    params.emplace("badLabelThresh", 0.52); //if less than 52% of the pixels are in the mask, label this label bad
    params.emplace("debugLevel", 1);
    params.emplace("maxFrameHeight", 288);  //emplace if not defined

    int maxFrameHeight = params.at("maxFrameHeight");
    int debugLevel = params.at("debugLevel");

    Mat mask = frame->getMask().clone();

    float factor = 1;
    //compute the scaling factor
    if (maxFrameHeight != 0)
    {
        factor = (float)maxFrameHeight / (float)mask.rows;

        resize(mask, mask, cvSize(mask.cols * factor, mask.rows * factor));
    }
    for (vector<LimbLabel>::iterator label = labels.begin(); label != labels.end(); ++label)
    {
        label->Resize(factor);
    }

    int correctPixels = 0, incorrectPixels = 0;
    int pixelsInMask = 0;
    int coveredPixelsInMask = 0;
    int incorrectlyCoveredPixels = 0;
    int missedPixels = 0;

    for (int i = 0; i < mask.cols; ++i) //at every col - x
    {
        for (int j = 0; j < mask.rows; ++j) //and every row - y
        {
            //int test = labels[0].containsPoint(Point2f(480,100));
            //check whether pixel hit a label from solution
            bool labelHit = false;
            for (vector<LimbLabel>::iterator label = labels.begin(); label != labels.end(); ++label)
            {
                if (label->containsPoint(Point2f(i, j))) //this is done in x,y coords
                {
                    labelHit = true;
                    //break;
                }
            }

            //check pixel colour
            int intensity = mask.at<uchar>(j, i); //this is done with reve
            bool blackPixel = (intensity < 10);

            if (!blackPixel)
                pixelsInMask++;

            if (blackPixel && labelHit) //if black in label, incorrect
            {
                incorrectPixels++;
                incorrectlyCoveredPixels++;
            }
            else if (!blackPixel && !labelHit) //if white not in label, incorret
            {
                incorrectPixels++;
                missedPixels++;
            }
            else if (!blackPixel && labelHit)//otherwise correct
            {
                correctPixels++;
                coveredPixelsInMask++;
            }
            //            else //black pixel and not label hit
            //                correctPixels++; //don't count these at all?
        }
    }


    double solutionEval = (float)correctPixels / ((float)correctPixels + (float)incorrectPixels);

    //now check for critical part failures - label mostly outside of mask

    vector<Point2f> badLabelScores;
    float badLabelThresh = params.at("badLabelThresh");

    for (vector<LimbLabel>::iterator label = labels.begin(); label != labels.end(); ++label)
    {
        vector<Point2f> poly = label->getPolygon(); //get the label polygon
        //compute min and max x and y
        //float xMin, xMax, yMin, yMax;
        vector<float> xS = { poly[0].x, poly[1].x, poly[2].x, poly[3].x };
        vector<float> yS = { poly[0].y, poly[1].y, poly[2].y, poly[3].y };
        auto xMin = min_element(xS.begin(), xS.end());
        auto xMax = max_element(xS.begin(), xS.end());

        auto yMin = min_element(yS.begin(), yS.end());
        auto yMax = max_element(yS.begin(), yS.end());

        int labelPixels = 0;
        int badLabelPixels = 0;

        for (int x = *xMin; x < *xMax; ++x)
        {
            for (int y = *yMin; y < *yMax; ++y)
            {
                if (label->containsPoint(Point2f(x, y)))
                {
                    int intensity = mask.at<uchar>(y, x); //this is done with reverse y,x
                    bool blackPixel = (intensity < 10);
                    labelPixels++;
                    if (blackPixel)
                        ++badLabelPixels;
                }
            }
        }

        float labelRatio = 1.0 - (float)badLabelPixels / (float)labelPixels; //high is good

        if (labelRatio < badLabelThresh /*&& !label->getIsWeak()*/ && !label->getIsOccluded()) //not weak, not occluded, badly localised
            badLabelScores.push_back(Point2f(label->getLimbID(), labelRatio));
    }

    if (debugLevel >= 1)
    {
        for (vector<Point2f>::iterator badL = badLabelScores.begin(); badL != badLabelScores.end(); ++badL)
        {
            cerr << "Part " << badL->x << " is badly localised, with score " << badL->y << endl;
        }
    }

    if (badLabelScores.size() != 0) //make the solution eval fail if a part is badly localised
        solutionEval = solutionEval - 1.0;

    if (debugLevel >= 1)
        cerr << "Solution evaluation score - " << solutionEval << " for frame " << frame->getID() << " solve from " << frame->getParentFrameID() << endl;

    return solutionEval;
}

int TLPSSolver::findFrameIndexById(int id, vector<Frame*> frames)
{
    for (uint32_t i = 0; i < frames.size(); ++i)
    {
        if (frames[i]->getID() == id)
            return i;
    }
    return -1;
}

//compute label score
float TLPSSolver::computeScoreCost(const LimbLabel& label, map<string, float> params)
{
    //    if(label.getIsOccluded() )// || label.getIsWeak()) //if it's occluded, return zero
    //        return 0;

    string hogName = "18500";
    string csName = "4409412";
    string surfName = "21316";

    params.emplace("imageCoeff", 1.0);
    params.emplace("useCSdet", 1.0);
    params.emplace("useHoGdet", 0.0);
    params.emplace("useSURFdet", 0.0);

    float lambda = params.at("imageCoeff");

    //@FIX
    float useHoG = params.at("useHoGdet");
    float useCS = params.at("useCSdet");
    float useSURF = params.at("useSURFdet");

    //TODO: Fix score combinations
    vector<Score> scores = label.getScores();

    //compute the weighted sum of scores
    float finalScore = 0;
    bool hogFound = false;
    bool csFound = false;
    bool surfFound = false;
    for (uint32_t i = 0; i < scores.size(); ++i)
    {
        float score = scores[i].getScore();
        if (scores[i].getScore() == -1)//if score is -1, set it to 1
        {
            score = 1.0; //set a high cost for invalid scores
        }
        if (scores[i].getDetName() == hogName)
        {
            finalScore = finalScore + score*useHoG;
            hogFound = true;
        }
        else if (scores[i].getDetName() == csName)
        {
            finalScore = finalScore + score*useCS;
            csFound = true;
        }
        else if (scores[i].getDetName() == surfName)
        {
            finalScore = finalScore + score*useSURF;
            surfFound = true;
        }
    }

    //now add 1.0*coeff for each not found score in this label, that should have been there (i.e., assume the worst)
    finalScore += 1.0*useHoG*(!hogFound) + 1.0*useCS*(!csFound) + 1.0*useSURF*(!surfFound);

    return finalScore;
}

//compute distance to parent limb label
float TLPSSolver::computeJointCost(const LimbLabel& child, const LimbLabel& parent, map<string, float> params, bool toChild)
{
    //emplace default
    //params.emplace("jointCoeff", 0.5);
    //float lambda = params.at("jointCoeff");
    Point2f p0, p1, c0, c1;

    //@FIX this is really too simplistic, connecting these points
    child.getEndpoints(c0, c1);
    parent.getEndpoints(p0, p1);

    //normalise this?

    //return the squared distance from the lower parent joint p1, to the upper child joint c0
    if (toChild)
        return sqrt(pow((c0.x - p1.x), 2) + pow((c0.y - p1.y), 2));
    else //otherwise we're connected to the parent's parent body part
        return sqrt(pow((c0.x - p0.x), 2) + pow((c0.y - p0.y), 2));
}

float TLPSSolver::computeNormJointCost(const LimbLabel& child, const LimbLabel& parent, map<string, float> params, float max, bool toChild)
{
    //emplace default
    params.emplace("jointCoeff", 0.5);
    params.emplace("jointLeeway", 0.05);
    params.emplace("debugLevel", 1);

    //read params
    float lambda = params.at("jointCoeff");
    int debugLevel = params.at("debugLevel");

    //float leeway = params.at("jointLeeway");
    Point2f p0, p1, c0, c1;

    //@FIX this is really too simplistic, connecting these points
    child.getEndpoints(c0, c1);
    parent.getEndpoints(p0, p1);

    //child length
    //float clen = sqrt(pow(c0.x-c1.x, 2)+pow(c0.y-c1.y, 2));

    //normalise this?
    //give some leeway
    float score = 0;
    if (toChild) //connected to parent's child joint
        score = sqrt(pow(c0.x - p1.x, 2) + pow(c0.y - p1.y, 2)) / max;
    else //else connected to parent's parent joint
        score = sqrt(pow(c0.x - p0.x, 2) + pow(c0.y - p0.y, 2)) / max;
    //    if(score<(clen*leeway)/max) //any distnace below leeway is zero
    //        score=0;
    //return the squared distance from the lower parent joint p1, to the upper child joint c0

    //output a sentence about who connected to whom and what the score was
    if (debugLevel >= 1 && (child.getLimbID() == 7 || child.getLimbID() == 6) && !toChild)
        cerr << "Part " << child.getLimbID() << " is connecting to part " << parent.getLimbID() << " PARENT joint" << endl;

    return lambda*score;
}

float TLPSSolver::computePriorCost(const LimbLabel& label, const BodyPart& prior, const Skeleton& skeleton, map<string, float> params)
{
    //  params.emplace("priorCoeff", 0.0);
    //	float lambda = params.at("priorCoeff");
    Point2f p0, p1, pp0, pp1;
    label.getEndpoints(p0, p1);
    pp0 = skeleton.getBodyJoint(prior.getParentJoint())->getImageLocation();
    pp1 = skeleton.getBodyJoint(prior.getChildJoint())->getImageLocation();

    //normalise this cost?

    //return the sum of squared distances between the corresponding joints, prior to label
    return pow((p0.x - pp0.x), 2) + pow((p0.y - pp0.y), 2) + pow((p1.x - pp1.x), 2) + pow((p1.y - pp1.y), 2);
}

float TLPSSolver::computeNormPriorCost(const LimbLabel& label, const BodyPart& prior, const Skeleton& skeleton, map<string, float> params, float max)
{
    params.emplace("priorCoeff", 0.0);
    float lambda = params.at("priorCoeff");
    Point2f p0, p1, pp0, pp1;
    label.getEndpoints(p0, p1);
    pp0 = skeleton.getBodyJoint(prior.getParentJoint())->getImageLocation();
    pp1 = skeleton.getBodyJoint(prior.getChildJoint())->getImageLocation();

    //normalise this cost?

    //return the sum of squared distances between the corresponding joints, prior to label
    return lambda*((pow((p0.x - pp0.x), 2) + pow((p0.y - pp0.y), 2) + pow((p1.x - pp1.x), 2) + pow((p1.y - pp1.y), 2)) / max);
}

float TLPSSolver::computePastTempCost(const LimbLabel& thisLabel, const LimbLabel& pastLabel, map<string, float> params)
{
    //emplace default
    params.emplace("tempCoeff", 0.5);
    float lambda = params.at("tempCoeff");
    //compute the temporal connection cost to label in the past

    //@PARAM there should be a parameter that sets the temporal cost constant (beta)
    //compute the joint joining cost for the skeleton
    Point2f p0, p1, c0, c1;

    //@FIX this is really too simplistic, connecting these points
    thisLabel.getEndpoints(c0, c1);
    pastLabel.getEndpoints(p0, p1);

    //return the squared distance from the lower parent joint p1, to the upper child joint c0
    return (pow((c0.x - p0.x), 2) + pow((c0.y - p0.y), 2) + pow((c1.x - p1.x), 2) + pow((c1.y - p1.y), 2));
}

float TLPSSolver::computeNormPastTempCost(const LimbLabel& thisLabel, const LimbLabel& pastLabel, map<string, float> params, float max)
{
    //emplace default
    params.emplace("tempCoeff", 0.5);
    float lambda = params.at("tempCoeff");
    //compute the temporal connection cost to label in the past

    //@PARAM there should be a parameter that sets the temporal cost constant (beta)
    //compute the joint joining cost for the skeleton
    Point2f p0, p1, c0, c1;

    //@FIX this is really too simplistic, connecting these points
    thisLabel.getEndpoints(c0, c1);
    pastLabel.getEndpoints(p0, p1);

    //return the squared distance from the lower parent joint p1, to the upper child joint c0
    return lambda*((pow((c0.x - p0.x), 2) + pow((c0.y - p0.y), 2) + pow((c1.x - p1.x), 2) + pow((c1.y - p1.y), 2)) / max);
}

float TLPSSolver::computeFutureTempCost(const LimbLabel& thisLabel, const LimbLabel& futureLabel, map<string, float> params)
{
    //emplace default
    params.emplace("tempCoeff", 0.5);
    float lambda = params.at("tempCoeff");
    //compute temporal connection cost to label in the future
    //@PARAM there needs to be a beta param here as well
    Point2f p0, p1, c0, c1;

    //@FIX this is really too simplistic, connecting these points
    thisLabel.getEndpoints(c0, c1);
    futureLabel.getEndpoints(p0, p1);

    float score = (pow((c0.x - p0.x), 2) + pow((c0.y - p0.y), 2) + pow((c1.x - p1.x), 2) + pow((c1.y - p1.y), 2));
    return score;
}

float TLPSSolver::computeNormFutureTempCost(const LimbLabel& thisLabel, const LimbLabel& futureLabel, map<string, float> params, float max)
{
    //emplace default
    params.emplace("tempCoeff", 0.5);
    float lambda = params.at("tempCoeff");
    //compute temporal connection cost to label in the future
    //@PARAM there needs to be a beta param here as well
    Point2f p0, p1, c0, c1;

    //@FIX this is really too simplistic, connecting these points
    thisLabel.getEndpoints(c0, c1);
    futureLabel.getEndpoints(p0, p1);

    float score = lambda*((pow((c0.x - p0.x), 2) + pow((c0.y - p0.y), 2) + pow((c1.x - p1.x), 2) + pow((c1.y - p1.y), 2)) / max);
    //return the squared distance from the lower parent joint p1, to the upper child joint c0

    return score;
}

float TLPSSolver::computeAnchorCost(const LimbLabel& thisLabel, Frame* anchor, map<string, float> params)
{
    //emploace default
    params.emplace("anchorCoeff", 1.0);
    float lambda = params.at("anchorCoeff");

    int limbId = thisLabel.getLimbID();

    vector<Point2f> partPolygon = anchor->getPartPolygon(limbId);
    vector<Point2f> labelPolygon = thisLabel.getPolygon();

    assert(partPolygon.size() == labelPolygon.size());

    //return the error between these'
    float score = 0;
    for (size_t i = 0; i < partPolygon.size(); ++i)
        score += pow(partPolygon[i].x - labelPolygon[i].x, 2) + pow(partPolygon[i].y - labelPolygon[i].y, 2);

    return score;
    //compute the cost of anchoring this label
}

float TLPSSolver::computeNormAnchorCost(const LimbLabel& thisLabel, Frame* anchor, map<string, float> params, float max)
{
    //emploace default
    params.emplace("anchorCoeff", 1.0);
    float lambda = params.at("anchorCoeff");

    int limbId = thisLabel.getLimbID();

    vector<Point2f> partPolygon = anchor->getPartPolygon(limbId);
    vector<Point2f> labelPolygon = thisLabel.getPolygon();

    assert(partPolygon.size() == labelPolygon.size());

    //return the error between these'
    float score = 0;
    for (size_t i = 0; i < partPolygon.size(); ++i)
        score += pow(partPolygon[i].x - labelPolygon[i].x, 2) + pow(partPolygon[i].y - labelPolygon[i].y, 2);

    return lambda*score / max;
    //compute the cost of anchoring this label
}

vector<vector<Frame*> > TLPSSolver::slice(const vector<Frame*>& frames) //separate the sequence into slices, for temporal solve
{
    //frames should be sliced into frame sets, where every non Keyframe non Lockframe frame should belong to a BOUNDED set
    //unbounded sets are not included in the solve
    vector<vector<Frame*> > aux;
    vector<vector<Frame*> > slices;

    vector<Frame*> currentSet;
    //bool isOpen;
    for (uint32_t i = 0; i < frames.size(); ++i)
    {
        currentSet.push_back(frames[i]); //push the frame to current set
        if (frames[i]->getFrametype() == KEYFRAME || frames[i]->getFrametype() == LOCKFRAME)
        {
            aux.push_back(currentSet);
            currentSet.clear();
            currentSet.push_back(frames[i]);
        }
    }
    //now go through every set, and eliminate it if:
    //1) it contains 2 or less elements
    //2) it doesn't end with a LOCKFRAME or a KEYFRAME
    //3) it doesn't begin with a LOCKFRAME or a KEYFRAME
    for (uint32_t i = 0; i < aux.size(); ++i)
    {
        if (aux[i].at(0)->getFrametype() == LOCKFRAME || aux[i].at(0)->getFrametype() == KEYFRAME) //if the set STARTS with a keyframe or a lockframe
        {
            if (aux[i].back()->getFrametype() == LOCKFRAME || aux[i].back()->getFrametype() == KEYFRAME) //if the set ENDS with a keyframe or a lockframe
            {
                if (aux[i].size()>2) //if size is greater than two elements
                    slices.push_back(aux[i]); //push back slice
            }
        }
    }

    return slices;
}

}

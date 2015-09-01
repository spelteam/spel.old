#include "surfDetector.hpp"

#define ERROR_HEADER __FILE__ << ":" << __LINE__ << ": "

namespace SPEL
{

  SurfDetector::SurfDetector(void)
  {
    id = 0x5344;
#if OpenCV_VERSION_MAJOR == 2 && OpenCV_VERSION_MINOR == 4 && OpenCV_VERSION_PATCH >= 9
    initModule_nonfree();
#endif
  }

  SurfDetector::~SurfDetector(void)
  {
    for (auto i : partModels)
      for (auto j : i.second)
        j.second.descriptors.release();

    for (auto i : labelModels)
      for (auto j : i.second)
        for (auto k : j.second)
          k.descriptors.release();
  }

  int SurfDetector::getID(void) const
  {
    return id;
  }

  void SurfDetector::setID(int _id)
  {
    id = _id;
  }

  //TODO (Vitaliy Koshura): Write real implementation here
  void SurfDetector::train(vector <Frame*> _frames, map <string, float> params)
  {
    frames = _frames;

    partModels.clear();
    labelModels.clear();

#ifdef DEBUG
    const uint8_t debugLevel = 5;
#else
    const uint8_t debugLevel = 1;
#endif // DEBUG
    const string sDebugLevel = "debugLevel";
    const uint32_t minHessian = 500;
    const string sMinHessian = "minHessian";

    params.emplace(sDebugLevel, debugLevel);
    params.emplace(sMinHessian, minHessian);

    const string sMaxFrameHeight = "maxFrameHeight";

    params.emplace(sMaxFrameHeight, frames.at(0)->getFrameSize().height);

    maxFrameHeight = params.at(sMaxFrameHeight);
    //maxFrameHeight=frames.at(0)->getFrameSize().height; //@TODO fix this later

    debugLevelParam = static_cast <uint8_t> (params.at(sDebugLevel));

    for (vector <Frame*>::iterator frameNum = frames.begin(); frameNum != frames.end(); ++frameNum)
    {

      if ((*frameNum)->getFrametype() != KEYFRAME && (*frameNum)->getFrametype() != LOCKFRAME)
      {
        continue;
      }

      int originalSize = (*frameNum)->getFrameSize().height;

      Frame *workFrame = 0;
      if ((*frameNum)->getFrametype() == KEYFRAME)
        workFrame = new Keyframe();
      else if ((*frameNum)->getFrametype() == LOCKFRAME)
        workFrame = new Lockframe();
      else if ((*frameNum)->getFrametype() == INTERPOLATIONFRAME)
        workFrame = new Interpolation();

      workFrame = (*frameNum)->clone(workFrame);

      workFrame->Resize(params.at(sMaxFrameHeight));

      if (debugLevelParam >= 2)
        cerr << "Training on frame " << workFrame->getID() << endl;

      try
      {
        partModels.insert(pair <uint32_t, map <uint32_t, PartModel>>(workFrame->getID(), computeDescriptors(workFrame, minHessian)));
      }
      catch (...)
      {
        break;
      }

      delete workFrame;
    }

  }

  //TODO (Vitaliy Koshura): Write real implementation here
  map <uint32_t, vector <LimbLabel> > SurfDetector::detect(Frame *frame, map <string, float> params, map <uint32_t, vector <LimbLabel>> limbLabels)
  {
    const string sMinHessian = "minHessian";
    const string sUseSURFdet = "useSURFdet";
    const string sKnnMatchCoeff = "knnMathCoeff";

    // first we need to check all used params
    params.emplace(sMinHessian, minHessian);
    params.emplace(sUseSURFdet, useSURFdet);
    params.emplace(sKnnMatchCoeff, knnMatchCoeff);

    //now set actual param values
    minHessian = params.at(sMinHessian);
    useSURFdet = params.at(sUseSURFdet);
    knnMatchCoeff = params.at(sKnnMatchCoeff);

    auto imgMat = frame->getImage();

#if OpenCV_VERSION_MAJOR == 3
    Ptr <SurfFeatureDetector> detector = SurfFeatureDetector::create(minHessian);
    detector->detect(imgMat, keyPoints);
    if (keyPoints.empty())
    {
      stringstream ss;
      ss << ERROR_HEADER << "Couldn't detect keypoints for frame " << frame->getID();
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
#else
    SurfFeatureDetector detector(minHessian);
    detector.detect(imgMat, keyPoints);
    if (keyPoints.empty())
    {
      stringstream ss;
      ss << ERROR_HEADER << "Couldn't detect keypoints for frame " << frame->getID();
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
#endif

    auto result = Detector::detect(frame, params, limbLabels);

    keyPoints.clear();

    return result;
  }

  map <uint32_t, SurfDetector::PartModel> SurfDetector::computeDescriptors(Frame *frame, uint32_t minHessian)
  {
    map <uint32_t, PartModel> parts;
    Skeleton skeleton = frame->getSkeleton();
    tree <BodyPart> partTree = skeleton.getPartTree();
    Mat imgMat = frame->getImage();
    vector <KeyPoint> keyPoints;

#if OpenCV_VERSION_MAJOR == 3
    Ptr <SurfFeatureDetector> detector = SurfFeatureDetector::create(minHessian);
    detector->detect(imgMat, keyPoints);
    if (keyPoints.empty())
    {
      stringstream ss;
      ss << ERROR_HEADER << "Couldn't detect keypoints for frame " << frame->getID();
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
#else
    SurfFeatureDetector detector(minHessian);
    detector.detect(imgMat, keyPoints);
    if (keyPoints.empty())
    {
      stringstream ss;
      ss << ERROR_HEADER << "Couldn't detect keypoints for frame " << frame->getID();
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
#endif

    for (tree <BodyPart>::iterator part = partTree.begin(); part != partTree.end(); ++part)
    {
      Point2f j0, j1;
      BodyJoint *joint = skeleton.getBodyJoint(part->getParentJoint());
      if (joint == 0)
      {
        stringstream ss;
        ss << "Invalid parent joint";
        if (debugLevelParam >= 1)
          cerr << ERROR_HEADER << ss.str() << endl;
        throw logic_error(ss.str());
      }
      j0 = joint->getImageLocation();
      joint = 0;
      joint = skeleton.getBodyJoint(part->getChildJoint());
      if (joint == 0)
      {
        stringstream ss;
        ss << "Invalid child joint";
        if (debugLevelParam >= 1)
          cerr << ERROR_HEADER << ss.str() << endl;
        throw logic_error(ss.str());
      }
      j1 = joint->getImageLocation();
      Point2f direction = j1 - j0; // used as estimation of the vector's direction
      float rotationAngle = float(spelHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI)); //bodypart tilt angle
      part->setRotationSearchRange(rotationAngle);
      try
      {
        parts.insert(pair <uint32_t, PartModel>(part->getPartID(), computeDescriptors(*part, j0, j1, imgMat, minHessian, keyPoints)));
      }
      catch (logic_error err)
      {
        stringstream ss;
        ss << "Can't compute descriptors for the frame " << frame->getID() << " for the part " << part->getPartID() << endl;
        ss << "\t" << err.what();
        if (debugLevelParam >= 1)
          cerr << ERROR_HEADER << ss.str() << endl;
        throw logic_error(ss.str());
      }
    }
    skeleton.setPartTree(partTree);
    frame->setSkeleton(skeleton);
    return parts;
  }

  SurfDetector::PartModel SurfDetector::computeDescriptors(BodyPart bodyPart, Point2f j0, Point2f j1, Mat imgMat, uint32_t minHessian, vector <KeyPoint> keyPoints)
  {
    float boneLength = getBoneLength(j0, j1);
    float boneWidth = getBoneWidth(boneLength, bodyPart);
    Size originalSize = Size(static_cast <uint32_t> (boneLength), static_cast <uint32_t> (boneWidth));
    POSERECT <Point2f> rect = getBodyPartRect(bodyPart, j0, j1, originalSize);

    PartModel partModel;
    partModel.partModelRect = rect;

    for (vector <KeyPoint>::iterator kp = keyPoints.begin(); kp != keyPoints.end(); ++kp)
    {
      if (rect.containsPoint(kp->pt) > 0)
      {
        partModel.keyPoints.push_back(*kp);
      }
    }

#if OpenCV_VERSION_MAJOR == 3
    if (partModel.keyPoints.empty())
    {
      if (debugLevelParam >= 2)
        cerr << ERROR_HEADER << "Couldn't detect keypoints of body part " << bodyPart.getPartID() << endl;
    }
    else
    {
      Ptr <SurfDescriptorExtractor> extractor = SurfDescriptorExtractor::create();
      extractor->compute(imgMat, partModel.keyPoints, partModel.descriptors);
      if (partModel.descriptors.empty() && debugLevelParam >= 2)
      {
        cerr << ERROR_HEADER << "Couldn't compute descriptors of body part " << bodyPart.getPartID() << endl;
      }
    }
#else
    if (partModel.keyPoints.empty())
    {
      if (debugLevelParam >= 2)
        cerr << ERROR_HEADER << "Couldn't detect keypoints of body part " << bodyPart.getPartID() << endl;
    }
    else
    {
      SurfDescriptorExtractor extractor;
      extractor.compute(imgMat, partModel.keyPoints, partModel.descriptors);
      if (partModel.descriptors.empty() && debugLevelParam >= 2)
      {
        cerr << ERROR_HEADER << "Couldn't compute descriptors of body part " << bodyPart.getPartID() << endl;
      }
    }
#endif
    return partModel;
  }

  LimbLabel SurfDetector::generateLabel(BodyPart bodyPart, Frame *frame, Point2f j0, Point2f j1)
  {
    stringstream detectorName;
    detectorName << getID();

    comparer_bodyPart = &bodyPart;

    PartModel generatedPartModel = computeDescriptors(bodyPart, j0, j1, frame->getImage(), minHessian, keyPoints);

    comparer_model = &generatedPartModel;
    comparer_j0 = &j0;
    comparer_j1 = &j1;

    LimbLabel label = Detector::generateLabel(bodyPart, j0, j1, detectorName.str(), useSURFdet);

    labelModels[frame->getID()][bodyPart.getPartID()].push_back(generatedPartModel);

    comparer_bodyPart = 0;
    comparer_model->descriptors.release();
    comparer_model = 0;
    comparer_j0 = 0;
    comparer_j1 = 0;

    return label;
  }

  float SurfDetector::compare(void)
  {
    if (comparer_bodyPart == 0 || comparer_model == 0 || comparer_j0 == 0 || comparer_j1 == 0)
    {
      stringstream ss;
      ss << "Compare parameters are invalid: " << (comparer_bodyPart == 0 ? "comparer_bodyPart == 0 " : "") << (comparer_model == 0 ? "comparer_model == 0 " : "") << (comparer_j0 == 0 ? "comparer_j0 == 0 " : "") << (comparer_j1 == 0 ? "comparer_j1 == 0" : "") << endl;
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    return compare(*comparer_bodyPart, *comparer_model, *comparer_j0, *comparer_j1);
  }

  float SurfDetector::compare(BodyPart bodyPart, PartModel model, Point2f j0, Point2f j1)
  {
    if (model.descriptors.empty())
    {
      if (debugLevelParam >= 2)
        cerr << ERROR_HEADER << "Model descriptors are empty" << endl;
      return -1.0f;
    }

    float score = 0;
    uint32_t count = 0;
    FlannBasedMatcher matcher;
    vector <vector <DMatch>> matches;

    float length = getBoneLength(j0, j1);
    float width = getBoneWidth(length, bodyPart);
    float coeff = sqrt(pow(length, 2) + pow(width, 2));

    for (map <uint32_t, map <uint32_t, PartModel>>::iterator framePartModels = partModels.begin(); framePartModels != partModels.end(); ++framePartModels)
    {
      for (map <uint32_t, PartModel>::iterator partModel = framePartModels->second.begin(); partModel != framePartModels->second.end(); ++partModel)
      {
        if (partModel->first != static_cast <uint32_t> (bodyPart.getPartID()))
        {
          continue;
        }
        else
        {
          if (partModel->second.descriptors.empty() || model.descriptors.empty())
          {
            if (debugLevelParam >= 2)
              cerr << ERROR_HEADER << "PartModel descriptors of body part [" << partModel->first << "] are empty" << endl;
          }
          else
          {
            try
            {
              if (partModel->second.descriptors.rows > 1 && model.descriptors.rows > 1)
              {
                matcher.knnMatch(model.descriptors, partModel->second.descriptors, matches, 2);
                float s = 0;
                for (uint32_t i = 0; i < matches.size(); i++)
                {
                  if ((matches[i][0].distance < knnMatchCoeff * (matches[i][1].distance)) && ((int)matches[i].size() <= 2 && (int)matches[i].size()>0))
                  {
                    s += matches[i][0].distance / coeff;
                    count++;
                  }
                }
                score += s / matches.size();
              }
              else
              {
                if (debugLevelParam >= 1)
                  cerr << ERROR_HEADER << "Can't match descriptors of body part [" << partModel->first << "]: Not enough descriptors" << endl;
              }
            }
            catch (...)
            {
              if (debugLevelParam >= 1)
                cerr << ERROR_HEADER << "Can't match descriptors of body part [" << partModel->first << "]" << endl;
            }
          }
          break;
        }
      }
    }
    if (count == 0)
      return -1.0f;
    return (score / (float)count);
  }

  map <uint32_t, map <uint32_t, SurfDetector::PartModel>> SurfDetector::getPartModels(void)
  {
    return partModels;
  }

  map <uint32_t, map <uint32_t, vector <SurfDetector::PartModel>>> SurfDetector::getLabelModels(void)
  {
    return labelModels;
  }

}

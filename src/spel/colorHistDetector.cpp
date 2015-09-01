#include "colorHistDetector.hpp"

#define ERROR_HEADER __FILE__ << ":" << __LINE__ << ": "
namespace SPEL
{
  // PartModel Constructor 
  // Initialization "partHistogram" and "bgHistogram" with _nBins^3 elements capacity 3-D arrays 
  ColorHistDetector::PartModel::PartModel(uint8_t _nBins) : nBins(_nBins)
  {
    partHistogram.resize(nBins);
    bgHistogram.resize(nBins);
    for (uint8_t r = 0; r < nBins; r++)
    {
      try
      {
        partHistogram.at(r).resize(nBins);
      }
      catch (...)
      {
        stringstream ss;
        ss << "Couldn't find partHistogram " << "[" << r << "]";
        throw logic_error(ss.str());
      }
      try
      {
        bgHistogram.at(r).resize(nBins);
      }
      catch (...)
      {
        stringstream ss;
        ss << "Couldn't find bgHistogram " << "[" << r << "]";
        throw logic_error(ss.str());
      }
      for (uint8_t g = 0; g < nBins; g++)
      {
        try
        {
          partHistogram.at(r).at(g).resize(nBins);
        }
        catch (...)
        {
          stringstream ss;
          ss << "Couldn't find partHistogram " << "[" << r << "][" << g << "]";
          throw logic_error(ss.str());
        }
        try
        {
          bgHistogram.at(r).at(g).resize(nBins);
        }
        catch (...)
        {
          stringstream ss;
          ss << "Couldn't find bgHistogram " << "[" << r << "][" << g << "]";
          throw logic_error(ss.str());
        }
        for (int b = 0; b < nBins; b++)
        {
          try
          {
            partHistogram.at(r).at(g).at(b) = 0.0;
          }
          catch (...)
          {
            stringstream ss;
            ss << "Couldn't find partHistogram " << "[" << r << "][" << g << "][" << b << "]";
            throw logic_error(ss.str());
          }
          try
          {
            bgHistogram.at(r).at(g).at(b) = 0.0;
          }
          catch (...)
          {
            stringstream ss;
            ss << "Couldn't find bgHistogram " << "[" << r << "][" << g << "][" << b << "]";
            throw logic_error(ss.str());
          }
        }
      }
    }
    sizeFG = 0;
  }

  // Copy all fields of the "PartModel" structure
  ColorHistDetector::PartModel &ColorHistDetector::PartModel::operator=(const PartModel &model)
  {
    this->nBins = model.nBins;
    this->partHistogram = model.partHistogram;
    this->bgHistogram = model.bgHistogram;
    this->sizeFG = model.sizeFG;
    this->sizeBG = model.sizeBG;
    this->fgNumSamples = model.fgNumSamples;
    this->bgNumSamples = model.bgNumSamples;
    this->fgSampleSizes = model.fgSampleSizes;
    this->bgSampleSizes = model.bgSampleSizes;
    this->fgBlankSizes = model.fgBlankSizes;
    return *this;
  }

  // Constructor with initialization of constant field "nBins"
  ColorHistDetector::ColorHistDetector(uint8_t _nBins) : nBins(_nBins)
  {
    id = 0x434844;
  }

  ColorHistDetector::~ColorHistDetector(void)
  {
    for (auto &&p : pixelDistributions)
      p.second.release();
    for (auto &&p : pixelLabels)
      p.second.release();
  }

  // Returns unique ID of "ColorHistDetector" object
  int ColorHistDetector::getID(void) const
  {
    return id;
  }

  // Change ID of "ColorHistDetector" object
  void ColorHistDetector::setID(int _id)
  {
    id = _id;
  }

  // Builds a histograms of all polygons for pre-marked frames
  void ColorHistDetector::train(vector <Frame*> _frames, map <string, float> params)
  {
    frames = _frames; // vector of pointers - presents a sequence of frames
    sort(frames.begin(), frames.end(), Frame::FramePointerComparer); // sorting frames by id
    //const float scaleParam = 1; // scaling coefficient
    //const string sScaleParam = "scaleParam";
#ifdef DEBUG
    const uint8_t debugLevel = 5;
#else
    const uint8_t debugLevel = 1;
#endif // DEBUG
    const string sDebugLevel = "debugLevel";
    // first we need to check all used params
    //params.emplace(sScaleParam, scaleParam);
    params.emplace(sDebugLevel, debugLevel);

    debugLevelParam = static_cast <uint8_t> (params.at(sDebugLevel));

    if (frames.size() == 0)
      throw logic_error("No input frames"); // the sequence of frames is empty
    partModels.clear();
    // Find skeleton from first keyframe or lockframe
    Skeleton skeleton;

    const string sMaxFrameHeight = "maxFrameHeight";

    params.emplace(sMaxFrameHeight, frames.at(0)->getFrameSize().height);

    maxFrameHeight = params.at(sMaxFrameHeight);

    bool bFind = false; // flag, indicate the presence of marked frame in the sequence
    for (vector <Frame*>::iterator i = frames.begin(); i != frames.end(); ++i)
    {
      Frame *f = *i;
      if (f->getFrametype() == KEYFRAME || f->getFrametype() == LOCKFRAME)
      {
        skeleton = f->getSkeleton();
        bFind = true; // marked frame was found
        break;
      }
    }
    if (bFind == false)
    {
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << "No neither keyframes nor lockframes" << endl;
      throw logic_error("No neither keyframes nor lockframes");
    }

    tree <BodyPart> partTree;
    // Handling all frames
    for (vector <Frame*>::iterator frameNum = frames.begin(); frameNum != frames.end(); ++frameNum)
    {
      if ((*frameNum)->getFrametype() != KEYFRAME && (*frameNum)->getFrametype() != LOCKFRAME)
      {
        continue; // skip unmarked frames
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
      // Create local variables
      map <int32_t, vector <Point3i>> partPixelColours; // the set of RGB-colours of pixel's for current body part
      map <int32_t, vector <Point3i>> bgPixelColours; // the set of RGB-colours for a pixels of background
      map <int32_t, int> blankPixels;  // pixels outside the mask
      skeleton = workFrame->getSkeleton(); // copy marking from current frame
      multimap <int32_t, POSERECT <Point2f>> polygons;  // polygons for this frame
      multimap <int32_t, float> polyDepth; // used for evaluation of overlapped polygons
      partTree = skeleton.getPartTree(); // the skeleton body parts
      // Handling all bodyparts on the frames
      for (tree <BodyPart>::iterator iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
      {
        partPixelColours.insert(pair <int32_t, vector <Point3i>>(iteratorBodyPart->getPartID(), vector <Point3i>())); // container initialization for conserve colours set for each of body parts
        bgPixelColours.insert(pair <int32_t, vector <Point3i>>(iteratorBodyPart->getPartID(), vector <Point3i>())); // container initialization for conserve background colours set for each of body parts
        blankPixels.insert(pair <int32_t, int>(iteratorBodyPart->getPartID(), 0)); // container initialization for counting blank pixels for each of body parts
        Point2f j1, j0;  // temporary adjacent joints   
        BodyJoint *joint = 0; // temporary conserve a joints
        joint = skeleton.getBodyJoint(iteratorBodyPart->getParentJoint()); // the parent node of current body part pointer 

        if (joint == 0)
        {
          if (debugLevelParam >= 1)
            cerr << ERROR_HEADER << "Invalid parent joint" << endl;
          break; // a joint has no marking on the frame
        }
        j0 = joint->getImageLocation(); // coordinates of current joint
        joint = 0;
        joint = skeleton.getBodyJoint(iteratorBodyPart->getChildJoint()); // the child node of current body part pointer
        if (joint == 0)
        {
          if (debugLevelParam >= 1)
            cerr << ERROR_HEADER << "Invalid child joint" << endl;
          break; // a joint has no marking on the frame
        }
        j1 = joint->getImageLocation(); // coordinates of current joint
        float boneLength = getBoneLength(j0, j1); // distance between nodes
        //TODO (Vitaliy Koshura): Check this!
        float boneWidth;
        try
        { //currents body part polygon width 
          boneWidth = getBoneWidth(boneLength, *iteratorBodyPart);
        }
        catch (...)
        {
          stringstream ss;
          ss << "Can't get LWRatio value";
          if (debugLevelParam >= 1)
            cerr << ERROR_HEADER << ss.str() << endl;
          throw logic_error(ss.str());
        }
        Point2f direction = j1 - j0; // used as estimation of the vector's direction
        float rotationAngle = float(spelHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI)); //bodypart tilt angle 
        iteratorBodyPart->setRotationSearchRange(rotationAngle);
        POSERECT <Point2f> poserect = getBodyPartRect(*iteratorBodyPart, j0, j1);
        polygons.insert(pair <int32_t, POSERECT <Point2f>>(iteratorBodyPart->getPartID(), poserect));
        polyDepth.insert(pair <int32_t, float>(iteratorBodyPart->getPartID(), skeleton.getBodyJoint(iteratorBodyPart->getParentJoint())->getSpaceLocation().z));
      }
      skeleton.setPartTree(partTree);
      workFrame->setSkeleton(skeleton);
      Mat maskMat = workFrame->getMask(); // copy mask from the current frame
      Mat imgMat = workFrame->getImage(); // copy image from the current frame
      // Range over all pixels of the frame
      for (int32_t i = 0; i < imgMat.cols; i++)
      {
        for (int32_t j = 0; j < imgMat.rows; j++)
        {
          Vec3b intensity;
          try
          {
            intensity = imgMat.at<Vec3b>(j, i);  // copy RGB color of current pixel
          }
          catch (...)
          {
            stringstream ss;
            ss << "Couldn't get imgMat value of indeces " << "[" << j << "][" << i << "]";
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
          // Copy the current pixel colour components
          uint8_t blue = intensity.val[0];
          uint8_t green = intensity.val[1];
          uint8_t red = intensity.val[2];
          uint8_t mintensity = 0;
          try
          {
            mintensity = maskMat.at<uint8_t>(j, i);  // copy current pixel mask value 
          }
          catch (...)
          {
            stringstream ss;
            ss << "Couldn't get maskMat value of indeces " << "[" << j << "][" << i << "]";
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
          bool blackPixel = mintensity < 10;
          int partHit = -1; // will be equal to -1 until is not found polygon, which contains the point
          float depth = 0;
          // Handling all poligons
          for (tree <BodyPart>::iterator iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
          {
            uint32_t partNumber = iteratorBodyPart->getPartID();
            bool bContainsPoint = false;
            try
            {
              vector <POSERECT <Point2f>> partPolygons;
              // Copy poligons to "PartPoligons"
              multimap <int32_t, POSERECT <Point2f>>::iterator lower = polygons.lower_bound(partNumber), upper = polygons.upper_bound(partNumber);
              transform(lower, upper, back_inserter(partPolygons), [](std::pair <int32_t, POSERECT<Point2f>> const &pair) { return pair.second; });
              // Checking whether a pixel belongs to the current and to another polygons            
              for (vector <POSERECT <Point2f>>::iterator iteratorPartPolygons = partPolygons.begin(); iteratorPartPolygons != partPolygons.end(); ++iteratorPartPolygons)
              {
                if ((bContainsPoint = iteratorPartPolygons->containsPoint(Point2f((float)i, (float)j)) > 0) == true)
                {
                  break;; // was found polygon, which contain current pixel
                }
              }
            }
            catch (...)
            {
              stringstream ss;
              ss << "There is no such polygon for body part " << partNumber;
              if (debugLevelParam >= 1)
                cerr << ERROR_HEADER << ss.str() << endl;
              throw logic_error(ss.str());
            }
            try
            {
              vector <float> partDepths;
              multimap <int32_t, float>::iterator lower = polyDepth.lower_bound(partNumber), upper = polyDepth.upper_bound(partNumber);
              transform(lower, upper, back_inserter(partDepths), [](std::pair <int32_t, bool> const &pair) { return pair.second; }); // copy "polyDepth" to "PartDepth"
              // Checkig polygons overlapping
              for (vector <float>::iterator iteratorPartDepths = partDepths.begin(); iteratorPartDepths != partDepths.end(); ++iteratorPartDepths)
              {
                if (bContainsPoint && partHit == -1)
                {
                  partHit = partNumber; // store the number of the first found polygon
                  depth = *iteratorPartDepths;
                }
                else if (bContainsPoint && *iteratorPartDepths < depth) // How, for float tempDepthSign?/////////////////
                {
                  partHit = partNumber;
                  depth = *iteratorPartDepths;
                }
              }
            }
            catch (...)
            {
              stringstream ss;
              ss << "There is no such polyDepth parameter for body part " << partNumber;
              if (debugLevelParam >= 1)
                cerr << ERROR_HEADER << ss.str() << endl;
              throw logic_error(ss.str());
            }
          }
          if (partHit != -1) // if was found polygon, that contains this pixel
          {
            if (!blackPixel) // if pixel color isn't black
            {
              try
              {
                partPixelColours.at(partHit).push_back(Point3i(red, green, blue)); // add colour of this pixel to part[partHit] colours
              }
              catch (...)
              {
                stringstream ss;
                ss << "There is no partPixelColours for body part " << partHit;
                if (debugLevelParam >= 1)
                  cerr << ERROR_HEADER << ss.str() << endl;
                throw logic_error(ss.str());
              }
              // For all bodyparts
              for (tree <BodyPart>::iterator p = partTree.begin(); p != partTree.end(); ++p)
              {
                if ((int32_t)p->getPartID() != partHit) // if current poligon wasn't found first in the previous enumeration???
                {
                  try
                  {
                    bgPixelColours.at(p->getPartID()).push_back(Point3i(red, green, blue)); // add colour of this pixel to part[partHit] background colours
                  }
                  catch (...)
                  {
                    stringstream ss;
                    ss << "There is no such bgPixelColours for body part " << p->getPartID();
                    if (debugLevelParam >= 1)
                      cerr << ERROR_HEADER << ss.str() << endl;
                    throw logic_error(ss.str());
                  }
                }
              }
            }
            else
            {
              try
              {
                blankPixels.at(partHit)++; // otherwise take stock this pixel to blank pixel counter
              }
              catch (...)
              {
                stringstream ss;
                ss << "There is no such blankPixels for body part " << partHit;
                if (debugLevelParam >= 1)
                  cerr << ERROR_HEADER << ss.str() << endl;
                throw logic_error(ss.str());
              }
            }
          }
          else // if  not found polygon, that contains this pixel 
          { // For all bodyparts
            for (tree <BodyPart>::iterator p = partTree.begin(); p != partTree.end(); ++p)
            {
              try
              {
                bgPixelColours.at(p->getPartID()).push_back(Point3i(red, green, blue));
              }
              catch (...)
              {
                stringstream ss;
                ss << "There is no such bgPixelColours for body part " << p->getPartID();
                if (debugLevelParam >= 1)
                  cerr << ERROR_HEADER << ss.str() << endl;
                throw logic_error(ss.str());
              }
            }
          }
        }
      }

      // Create model for each bodypart
      for (tree <BodyPart>::iterator iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
      {
        int32_t partNumber = iteratorBodyPart->getPartID();
        if (partModels.find(partNumber) == partModels.end())
        {
          PartModel model(nBins);
          partModels.insert(pair <int32_t, PartModel>(partNumber, model)); //add a new model to end of models list
        }
        try
        {
          PartModel partModel = partModels.at(partNumber);
          vector <Point3i> partPixelColoursVector; // temporary variable
          vector <Point3i> bgPixelColoursVector; // temporary variable
          int blankPixelsCount;
          try
          {
            partPixelColoursVector = partPixelColours.at(partNumber); // copy part color set for current bodypart
          }
          catch (...)
          {
            stringstream ss;
            ss << "There is no such partPixelColours for body part " << partNumber;
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
          try
          {
            blankPixelsCount = blankPixels.at(partNumber);  // copy blanck pixel count for current bodypart
          }
          catch (...)
          {
            stringstream ss;
            ss << "There is no such blankPixels for body part " << partNumber;
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
          try
          {
            bgPixelColoursVector = bgPixelColours.at(partNumber); // copy background color set for current bodypart
          }
          catch (...)
          {
            stringstream ss;
            ss << "There is no such bgPixelColours for body part " << partNumber;
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
          addPartHistogram(partModel, partPixelColoursVector, blankPixelsCount); // building histogram for current bodypart colours
          addBackgroundHistogram(partModel, bgPixelColoursVector); // building histograms for current bodypart background colours
          partModels.at(partNumber) = partModel; // copy result to part models set
          if (debugLevelParam >= 2)
            cerr << "Found part model: " << partNumber << endl;
        }
        catch (...)
        {
          stringstream ss;
          ss << "Could not find part model " << partNumber;
          if (debugLevelParam >= 1)
            cerr << ERROR_HEADER << ss.str() << endl;
          throw logic_error(ss.str());
        }

      }
      delete workFrame;
    }
  }

  // Returns a labels vector of possible body parts position
  map <uint32_t, vector <LimbLabel> > ColorHistDetector::detect(Frame *frame, map <string, float> params, map <uint32_t, vector <LimbLabel>> limbLabels)
  {
    const string sUseCSdet = "useCSdet";

    params.emplace(sUseCSdet, useCSdet);

    //now set actual param values
    useCSdet = params.at(sUseCSdet);

    pixelDistributions = buildPixelDistributions(frame); // matrix contains the probability that the particular pixel belongs to current bodypart
    pixelLabels = buildPixelLabels(frame, pixelDistributions); // matrix contains relative estimations that the particular pixel belongs to current bodypart

    auto result = Detector::detect(frame, params, limbLabels);

    for (auto &&var : pixelDistributions)
    {
      var.second.release();
    }
    pixelDistributions.clear();

    for (auto &&var : pixelLabels)
    {
      var.second.release();
    }
    pixelLabels.clear();

    return result;
  }

  // Return nBins
  uint8_t ColorHistDetector::getNBins(void) const
  {
    return nBins;
  }

  // Returns relative frequency of the RGB-color reiteration in "PartModel" 
  float ColorHistDetector::computePixelBelongingLikelihood(const PartModel &partModel, uint8_t r, uint8_t g, uint8_t b)
  { // Scaling of colorspace, finding the colors interval, which now gets this color
    uint8_t factor = static_cast<uint8_t> (ceil(pow(2, 8) / partModel.nBins));
    try
    {
      return partModel.partHistogram.at(r / factor).at(g / factor).at(b / factor); // relative frequency of current color reiteration 
    }
    catch (...)
    {
      stringstream ss;
      ss << "Couldn't find partHistogram " << "[" << (int)r / factor << "][" << (int)g / factor << "][" << (int)b / factor << "]";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
  }

  // Build into the "partModel" a histogram of the color set "partColors"
  void ColorHistDetector::setPartHistogram(PartModel &partModel, const vector <Point3i> &partColors)
  {
    // do not add sample if the number of pixels is zero
    if (partColors.size() == 0)
      return;
    uint8_t factor = static_cast<uint8_t> (ceil(pow(2, 8) / partModel.nBins)); // colorspace scaling coefficient
    partModel.sizeFG = static_cast <uint32_t> (partColors.size());
    partModel.fgNumSamples = 1;
    partModel.fgSampleSizes.clear();
    partModel.fgSampleSizes.push_back(static_cast <uint32_t> (partColors.size()));

    // clear histogram first
    for (uint8_t r = 0; r < partModel.nBins; r++)
    {
      for (uint8_t g = 0; g < partModel.nBins; g++)
      {
        for (uint8_t b = 0; b < partModel.nBins; b++)
        {
          try
          {
            partModel.partHistogram.at(r).at(g).at(b) = 0.0;
          }
          catch (...)
          {
            stringstream ss;
            ss << "Couldn't find partHistogram " << "[" << r << "][" << g << "][" << b << "]";
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
        }
      }
    }
    // Scaling of colorspace, reducing the capacity and number of colour intervals that are used to construct the histogram
    for (uint32_t i = 0; i < partColors.size(); i++)
    {
      uint8_t r, g, b;
      try
      {
        r = static_cast<uint8_t> (partColors.at(i).x / factor);
        g = static_cast<uint8_t> (partColors.at(i).y / factor);
        b = static_cast<uint8_t> (partColors.at(i).z / factor);
      }
      catch (...)
      {
        stringstream ss;
        ss << "Couldn't get partColors with index " << i;
        if (debugLevelParam >= 1)
          cerr << ERROR_HEADER << ss.str() << endl;
        throw logic_error(ss.str());
      }
      try
      {
        partModel.partHistogram.at(r).at(g).at(b)++; // increment the frequency of interval, that this color have hit
      }
      catch (...)
      {
        stringstream ss;
        ss << "Couldn't find partHistogram " << "[" << r << "][" << g << "][" << b << "]";
        if (debugLevelParam >= 1)
          cerr << ERROR_HEADER << ss.str() << endl;
        throw logic_error(ss.str());
      }
    }
    for (uint8_t r = 0; r < partModel.nBins; r++)
    {
      for (uint8_t g = 0; g < partModel.nBins; g++)
      {
        for (uint8_t b = 0; b < partModel.nBins; b++)
        {
          // normalise the histograms
          try
          {
            partModel.partHistogram.at(r).at(g).at(b) /= partModel.sizeFG;
          }
          catch (...)
          {
            stringstream ss;
            ss << "Couldn't find partHistogram " << "[" << r << "][" << g << "][" << b << "]";
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
        }
      }
    }
  }

  // Take stock of the additional set of colors in the histogram
  void ColorHistDetector::addPartHistogram(PartModel &partModel, const vector <Point3i> &partColors, uint32_t nBlankPixels)
  {
    if (partColors.size() == 0) //do not add sample if the number of pixels is zero
      return;
    //un-normalise
    for (uint8_t r = 0; r < partModel.nBins; r++)
    {
      for (uint8_t g = 0; g < partModel.nBins; g++)
      {
        for (uint8_t b = 0; b < partModel.nBins; b++)
        {
          try
          {
            partModel.partHistogram.at(r).at(g).at(b) *= partModel.sizeFG; // converting the colors relative frequency into the pixels number
          }
          catch (...)
          {
            stringstream ss;
            ss << "Couldn't find partHistogram " << "[" << r << "][" << g << "][" << b << "]";
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
        }
      }
    }

    int factor = (int)ceil(pow(2, 8) / partModel.nBins); // colorspace scaling coefficient
    partModel.sizeFG += static_cast <uint32_t> (partColors.size());
    partModel.fgNumSamples++;
    partModel.fgSampleSizes.push_back(static_cast <uint32_t> (partColors.size()));

    // Scaling of colorspace, reducing the capacity and number of colour intervals
    // Adjustment of the histogram
    for (uint32_t i = 0; i < partColors.size(); i++)
    {
      Point3i color;
      try
      {
        color = partColors.at(i);
      }
      catch (...)
      {
        stringstream ss;
        ss << "Couldn't find partColor " << "[" << i << "]";
        if (debugLevelParam >= 1)
          cerr << ERROR_HEADER << ss.str() << endl;
        throw logic_error(ss.str());
      }
      uint8_t r = static_cast<uint8_t> (color.x / factor);
      uint8_t g = static_cast<uint8_t> (color.y / factor);
      uint8_t b = static_cast<uint8_t> (color.z / factor);
      try
      {
        partModel.partHistogram.at(r).at(g).at(b)++; // increment the frequency of interval, that this color have hit
      }
      catch (...)
      {
        stringstream ss;
        ss << "Couldn't find partHistogram " << "[" << r << "][" << g << "][" << b << "]";
        if (debugLevelParam >= 1)
          cerr << ERROR_HEADER << ss.str() << endl;
        throw logic_error(ss.str());
      }
    }

    //renormalise
    for (uint8_t r = 0; r < partModel.nBins; r++)
    {
      for (uint8_t g = 0; g < partModel.nBins; g++)
      {
        for (uint8_t b = 0; b < partModel.nBins; b++)
        {
          //normalise the histograms
          try
          {
            partModel.partHistogram.at(r).at(g).at(b) /= partModel.sizeFG;
          }
          catch (...)
          {
            stringstream ss;
            ss << "Couldn't find partHistogram " << "[" << r << "][" << g << "][" << b << "]";
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
        }
      }
    }

    partModel.fgBlankSizes.push_back(nBlankPixels); // add the number of blank pixels for this model
  }

  // Totalization the number of used samples
  float ColorHistDetector::getAvgSampleSizeFg(const PartModel &partModel)
  {
    float sum = 0;
    for (uint32_t i = 0; i < partModel.fgSampleSizes.size(); i++)
    {
      try
      {
        sum += partModel.fgSampleSizes.at(i);
      }
      catch (...)
      {
        stringstream ss;
        ss << "Couldn't find fgSampleSizes " << "[" << i << "]";
        if (debugLevelParam >= 1)
          cerr << ERROR_HEADER << ss.str() << endl;
        throw logic_error(ss.str());
      }
    }
    sum /= partModel.fgNumSamples;
    return sum;
  }

  // Averaging the number of samples, that united from two sets
  float ColorHistDetector::getAvgSampleSizeFgBetween(const PartModel &partModel, uint32_t s1, uint32_t s2)
  {
    if (s1 >= partModel.fgSampleSizes.size() || s2 >= partModel.fgSampleSizes.size())
      return 0;
    try
    {
      return (partModel.fgSampleSizes.at(s1) + partModel.fgSampleSizes.at(s2)) / 2.0f;
    }
    catch (...)
    {
      stringstream ss;
      ss << "Couldn't find fgSampleSizes " << "[" << s1 << "] or fgSampleSizes [" << s2 << "]";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
  }

  //TODO (Vitaliy Koshura): Need unit test
  // Euclidean distance between part histograms
  float ColorHistDetector::matchPartHistogramsED(const PartModel &partModelPrev, const PartModel &partModel)
  {
    float distance = 0;
    for (uint8_t r = 0; r < partModel.nBins; r++)
    {
      for (uint8_t g = 0; g < partModel.nBins; g++)
      {
        for (uint8_t b = 0; b < partModel.nBins; b++)
        {
          // accumulation of the Euclidean distances between the points
          try
          {
            distance += pow(partModel.partHistogram.at(r).at(g).at(b) - partModelPrev.partHistogram.at(r).at(g).at(b), 2);
          }
          catch (...)
          {
            stringstream ss;
            ss << "Couldn't find partHistogram " << "[" << r << "][" << g << "][" << b << "]";
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
        }
      }
    }
    return sqrt(distance);
  }

  // Background histogram
  void ColorHistDetector::addBackgroundHistogram(PartModel &partModel, const vector <Point3i> &bgColors)
  {
    if (bgColors.size() == 0)
      return;
    // unnormalise
    for (uint8_t r = 0; r < partModel.nBins; r++)
    {
      for (uint8_t g = 0; g < partModel.nBins; g++)
      {
        for (uint8_t b = 0; b < partModel.nBins; b++)
        {
          try
          {
            partModel.bgHistogram.at(r).at(g).at(b) *= partModel.sizeBG;
          }
          catch (...)
          {
            stringstream ss;
            ss << "Couldn't find bgHistogram " << "[" << r << "][" << g << "][" << b << "]";
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
        }
      }
    }
    uint32_t factor = (uint32_t)ceil(pow(2, 8) / partModel.nBins); // colorspace scaling coefficient
    partModel.sizeBG += static_cast <uint32_t> (bgColors.size());
    partModel.bgNumSamples++;
    partModel.bgSampleSizes.push_back(static_cast <uint32_t> (bgColors.size()));
    for (uint32_t i = 0; i < bgColors.size(); i++)
    {
      Point3i color;
      try
      {
        color = bgColors.at(i);
      }
      catch (...)
      {
        stringstream ss;
        ss << "Couldn't find bgColors " << "[" << i << "]";
        if (debugLevelParam >= 1)
          cerr << ERROR_HEADER << ss.str() << endl;
        throw logic_error(ss.str());
      }
      try
      {
        partModel.bgHistogram.at(color.x / factor).at(color.y / factor).at(color.z / factor)++; // increment the frequency of interval, that this color have hit
      }
      catch (...)
      {
        stringstream ss;
        ss << "Couldn't find bgHistogram " << "[" << color.x / factor << "][" << color.y / factor << "][" << color.z / factor << "]";
        if (debugLevelParam >= 1)
          cerr << ERROR_HEADER << ss.str() << endl;
        throw logic_error(ss.str());
      }
    }
    // renormalise
    for (uint8_t r = 0; r < partModel.nBins; r++)
    {
      for (uint8_t g = 0; g < partModel.nBins; g++)
      {
        for (uint8_t b = 0; b < partModel.nBins; b++)
        {
          try
          {
            partModel.bgHistogram.at(r).at(g).at(b) /= (float)partModel.sizeBG;
          }
          catch (...)
          {
            stringstream ss;
            ss << "Couldn't find bgHistogram " << "[" << r << "][" << g << "][" << b << "]";
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
        }
      }
    }
  }

  // Returns a matrix, that contains relative frequency of the pixels colors reiteration 
  map <int32_t, Mat> ColorHistDetector::buildPixelDistributions(Frame *frame)
  {
    Skeleton skeleton = frame->getSkeleton(); // copy skeleton from the frame
    tree <BodyPart> partTree = skeleton.getPartTree(); // copy part tree from the skeleton
    Mat imgMat = frame->getImage(); // copy image from the frame
    Mat maskMat = frame->getMask(); // copy mask from the frame
    uint32_t width = imgMat.cols;
    uint32_t height = imgMat.rows;
    uint32_t mwidth = maskMat.cols;
    uint32_t mheight = maskMat.rows;
    map <int32_t, Mat> pixelDistributions;
    if (width != mwidth || height != mheight) // error if mask and image sizes don't match
    {
      stringstream ss;
      ss << "Mask size not equal image size";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    // For all bodyparts
    for (tree <BodyPart>::iterator iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
    {
      Mat t = Mat(height, width, DataType <float>::type); // create empty matrix
      int partID = iteratorBodyPart->getPartID();
      try
      {
        PartModel partModel = partModels.at(partID); // copy part model of current bodybart
        // For all pixels
        for (uint32_t x = 0; x < width; x++)
        {
          for (uint32_t y = 0; y < height; y++)
          {
            Vec3b intensity = imgMat.at<Vec3b>(y, x);
            // Copy components of the current pixel color
            uint8_t blue = intensity.val[0];
            uint8_t green = intensity.val[1];
            uint8_t red = intensity.val[2];
            uint8_t mintensity = maskMat.at<uint8_t>(y, x); // copy mask of the current pixel
            bool blackPixel = mintensity < 10; // pixel is not significant if the mask value is less than this threshold

            t.at<float>(y, x) = blackPixel ? 0 : computePixelBelongingLikelihood(partModel, red, green, blue); // relative frequency of the current pixel color reiteration 
          }
        }
      }
      catch (...)
      {
        stringstream ss;
        ss << "Maybe couldn't find partModel " << partID;
        if (debugLevelParam >= 1)
          cerr << ERROR_HEADER << ss.str() << endl;
        throw logic_error(ss.str());
      }
      pixelDistributions.insert(pair <int32_t, Mat>(partID, t)); // add the current bodypart matrix to the set 
    }
    return pixelDistributions;
  }


  map <int32_t, Mat> ColorHistDetector::buildPixelLabels(Frame *frame, map <int32_t, Mat> pixelDistributions)
  {
    Mat maskMat = frame->getMask(); // copy mask from the frame
    uint32_t width = maskMat.cols;
    uint32_t height = maskMat.rows;
    Skeleton skeleton = frame->getSkeleton(); // copy skeleton from the frame
    tree <BodyPart> partTree = skeleton.getPartTree(); // copy part tree from the skeleton
    map <int32_t, Mat> pixelLabels;
    // For all body parts
    for (tree <BodyPart>::iterator iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
    {
      Mat t = Mat(height, width, DataType <float>::type); // create empty matrix
      Mat tt;
      try
      { // Matrix, that contains relative frequency of the pixels colors reiteration for current body part
        tt = pixelDistributions.at(iteratorBodyPart->getPartID());
      }
      catch (...)
      {
        stringstream ss;
        ss << "Couldn't find distributions for body part " << iteratorBodyPart->getPartID();
        if (debugLevelParam >= 1)
          cerr << ERROR_HEADER << ss.str() << endl;
        throw logic_error(ss.str());
      }
      // For all pixels
      for (uint32_t x = 0; x < width; x++)
      {
        for (uint32_t y = 0; y < height; y++)
        {
          uint8_t mintensity = maskMat.at<uint8_t>(y, x); //copy the current pixel mask value
          bool blackPixel = mintensity < 10; // pixel is not significant if the mask value is less than this threshold
          if (!blackPixel)
          {
            float top = 0;
            float sum = 0;
            // For all body parts
            for (tree <BodyPart>::iterator i = partTree.begin(); i != partTree.end(); ++i)
            {
              Mat temp;
              try
              {
                temp = pixelDistributions.at(i->getPartID()); // matrix of the pixels colors frequency for current body part
              }
              catch (...)
              {
                stringstream ss;
                ss << "Couldn't find pixel distributions for body part " << i->getPartID();
                if (debugLevelParam >= 1)
                  cerr << ERROR_HEADER << ss.str() << endl;
                throw logic_error(ss.str());
              }
              try
              {
                if (temp.at<float>(y, x) > top) // search max value of the current bodypart pixel color frequency
                  top = temp.at<float>(y, x);
                sum += temp.at<float>(y, x);
              }
              catch (...)
              {
                stringstream ss;
                ss << "Couldn't find value of temp " << "[" << y << "][" << x << "]";
                if (debugLevelParam >= 1)
                  cerr << ERROR_HEADER << ss.str() << endl;
                throw logic_error(ss.str());
              }
            }
            try
            {
              t.at<float>(y, x) = (top == 0) ? 0 : tt.at<float>(y, x) / (float)top;
            }
            catch (...)
            {
              stringstream ss;
              ss << "Couldn't find t " << "[" << y << "][" << x << "] or tt [" << y << "][" << x << "]";
              if (debugLevelParam >= 1)
                cerr << ERROR_HEADER << ss.str() << endl;
              throw logic_error(ss.str());
            }
          }
          else
          {
            try
            {
              t.at<float>(y, x) = 0;
            }
            catch (...)
            {
              stringstream ss;
              ss << "Couldn't find value of t " << "[" << y << "][" << x << "]";
              if (debugLevelParam >= 1)
                cerr << ERROR_HEADER << ss.str() << endl;
              throw logic_error(ss.str());
            }
          }
        }
      }
      pixelLabels.insert(pair<int32_t, Mat>(iteratorBodyPart->getPartID(), t)); // insert the resulting matrix into the set "pixelLabels" 
    }
    return pixelLabels;
  }

  float ColorHistDetector::compare(void)
  {
    if (comparer_bodyPart == 0 || comparer_frame == 0 || comparer_j0 == 0 || comparer_j1 == 0)
    {
      stringstream ss;
      ss << "Compare parameters are invalid: " << (comparer_bodyPart == 0 ? "comparer_bodyPart == 0 " : "") << (comparer_frame == 0 ? "comparer_frame == 0 " : "") << (comparer_j0 == 0 ? "comparer_j0 == 0" : "") << (comparer_j1 == 0 ? "comparer_j1 == 0" : "") << endl;
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    try
    {
      return compare(*comparer_bodyPart, *comparer_frame, pixelDistributions, pixelLabels, *comparer_j0, *comparer_j1);
    }
    catch (logic_error ex)
    {
      if (debugLevelParam >= 1)
      {
          string frameType;
          if((*comparer_frame)->getFrametype()==KEYFRAME)
              frameType="Keyframe";
          else if((*comparer_frame)->getFrametype()==LOCKFRAME)
              frameType="Lockframe";
          else
              frameType="Interpolation";
        cerr << ERROR_HEADER << "Dirty Label: " << " Frame("<<frameType<< "): " << (*comparer_frame)->getID() << " Part: " << comparer_bodyPart->getPartID() << " " << ex.what() << endl;
      }
      return -1.0f;
    }
  }

  float ColorHistDetector::compare(BodyPart bodyPart, Frame *frame, map <int32_t, Mat> pixelDistributions, map <int32_t, Mat> pixelLabels, Point2f j0, Point2f j1)
  {
    Mat maskMat = frame->getMask(); // copy mask from the frame 
    Mat imgMat = frame->getImage(); // copy image from the frame
    Point2f boxCenter = j0 * 0.5 + j1 * 0.5; // segment center
    float boneLength = getBoneLength(j0, j1); // distance between joints
    POSERECT <Point2f> rect = getBodyPartRect(bodyPart, j0, j1); // expected bodypart location area?
    uint32_t totalPixels = 0;
    uint32_t pixelsInMask = 0;
    float totalPixelLabelScore = 0;
    float pixDistAvg = 0;
    float pixDistNum = 0;
    PartModel model;
    try
    {
      model = partModels.at(bodyPart.getPartID()); // copy part model for the "bodyPart"
    }
    catch (...)
    {
      stringstream ss;
      ss << "Couldn't get partModel of bodyPart " << bodyPart.getPartID();
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    if (getAvgSampleSizeFg(model) == 0) // error if samples count is zero
    {
      stringstream ss;
      ss << "Couldn't get avgSampleSizeFg";
      if (debugLevelParam >= 2)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax); // highlight the extreme points of the body part rect
    Mat bodyPartPixelDistribution;
    try
    {
      bodyPartPixelDistribution = pixelDistributions.at(bodyPart.getPartID());
    }
    catch (...)
    {
      stringstream ss;
      ss << "Can't get pixesDistribution [" << bodyPart.getPartID() << "]";
      if (debugLevelParam >= 2)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    Mat bodyPartLixelLabels;
    try
    {
      bodyPartLixelLabels = pixelLabels.at(bodyPart.getPartID());
    }
    catch (...)
    {
      stringstream ss;
      ss << "Can't get pixesLabels [" << bodyPart.getPartID() << "]";
      if (debugLevelParam >= 2)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    // Scan the area near the bodypart center

    float searchXMin = boxCenter.x - boneLength * 0.5;
    float searchXMax = boxCenter.x + boneLength * 0.5;
    float searchYMin = boxCenter.y - boneLength * 0.5;
    float searchYMax = boxCenter.y + boneLength * 0.5;

    for (float i = searchXMin; i < searchXMax; i++)
    {
      for (float j = searchYMin; j < searchYMax; j++)
      {
        if (i < maskMat.cols && j < maskMat.rows && i >= 0 && j >= 0) // if the point is within the image
        {
          if (i <= xmax && i >= xmin && j <= ymax && j >= ymin) // if the point within the highlight area
          {
            if (rect.containsPoint(Point2f(i, j)) > 0) // if the point belongs to the rectangle
            {
              totalPixels++; // counting of the contained pixels
              uint8_t mintensity = 0;
              try
              {
                mintensity = maskMat.at<uint8_t>((int32_t)j, (int32_t)i); // copy current point mask value 
              }
              catch (...)
              {
                stringstream ss;
                ss << "Can't get maskMat [" << (int32_t)j << "][" << (int32_t)i << "]";
                if (debugLevelParam >= 2)
                  cerr << ERROR_HEADER << ss.str() << endl;
                throw logic_error(ss.str());
              }
              bool blackPixel = mintensity < 10; // pixel is not significant if the mask value is less than this threshold
              if (!blackPixel)
              {
                try
                {
                  pixDistAvg += bodyPartPixelDistribution.at<float>(j, i); // Accumulation the "distributions" of contained pixels
                }
                catch (...)
                {
                  stringstream ss;
                  ss << "Can't get pixesDistribution [" << bodyPart.getPartID() << "][" << (int32_t)j << "][" << (int32_t)i << "]";
                  if (debugLevelParam >= 2)
                    cerr << ERROR_HEADER << ss.str() << endl;
                  throw logic_error(ss.str());
                }
                pixDistNum++; // counting of the all scanned pixels
                try
                {
                  if (bodyPartLixelLabels.at<float>(j, i))
                  {
                    totalPixelLabelScore += bodyPartLixelLabels.at<float>(j, i); // Accumulation of the pixel labels
                  }
                }
                catch (...)
                {
                  stringstream ss;
                  ss << "Can't get pixesLabels [" << bodyPart.getPartID() << "][" << (int32_t)j << "][" << (int32_t)i << "]";
                  if (debugLevelParam >= 2)
                    cerr << ERROR_HEADER << ss.str() << endl;
                  throw logic_error(ss.str());
                }
                pixelsInMask++; // counting pixels within the mask
              }
            }
          }
        }
      }
    }
    float supportScore = 0;
    float inMaskSupportScore = 0;
    pixDistAvg /= (float)pixDistNum;  // average "distributions"
    float inMaskSuppWeight = 0.5;
    if (totalPixelLabelScore > 0 && totalPixels > 10)
    {
      supportScore = (float)totalPixelLabelScore / (float)totalPixels;
      inMaskSupportScore = (float)totalPixelLabelScore / (float)pixelsInMask;
      float score = 1.0f - ((1.0f - inMaskSuppWeight) * supportScore + inMaskSuppWeight * inMaskSupportScore);
      return score;
    }
    stringstream ss;
    ss << "Dirty label!";
    if (debugLevelParam >= 2)
      cerr << ERROR_HEADER << ss.str() << endl;
    throw logic_error(ss.str());
  }

  LimbLabel ColorHistDetector::generateLabel(BodyPart bodyPart, Frame *frame, Point2f j0, Point2f j1)
  {
    stringstream detectorName;
    detectorName << getID();

    comparer_bodyPart = &bodyPart;
    comparer_frame = &frame;
    comparer_j0 = &j0;
    comparer_j1 = &j1;

    LimbLabel label = Detector::generateLabel(bodyPart, j0, j1, detectorName.str(), useCSdet);

    comparer_bodyPart = 0;
    comparer_frame = 0;
    comparer_j0 = 0;
    comparer_j1 = 0;

    return label;
  }

  //Used only as prevent a warning for "const uint8_t nBins";
  vector <Frame*> ColorHistDetector::getFrames() const
  {
    return frames;
  }

  //Used only as prevent a warning for "const uint8_t nBins";
  ColorHistDetector &ColorHistDetector::operator=(const ColorHistDetector &c)
  {
    this->frames = c.getFrames();
    return *this;
  }

}

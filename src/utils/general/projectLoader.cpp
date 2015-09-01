#include "projectLoader.hpp"
#include "imagesimilaritymatrix.hpp"
#include "lockframe.hpp"
#include "spelHelper.hpp"

ProjectLoader::ProjectLoader(string _curFolder)
{
  curFolder = _curFolder;
}
ProjectLoader::~ProjectLoader()
{
  for(auto&& frame:vFrames)
      delete frame;
  vFrames.clear();
}

void ProjectLoader::SetCurFolder(string _curFolder)
{
  curFolder = _curFolder;
}

vector <Frame*> ProjectLoader::getFrames(void)
{
  return vFrames;
}

bool ProjectLoader::Load(string fileName)
{
  const string projectNode = "Project";
  const string projectName = "name";
  const string projectImgFolderPath = "imgFolderPath";
  const string projectMaskFolderPath = "maskFolderPath";
  const string projectCamFolderPath = "camFolderPath";
  const string projectAllowScaling = "allowScaling";
  const string projectSimMathPath = "simMatPath";
  const string projectExportPath = "exportPath";
  const string bodyJointsNode = "BodyJoints";
  const string bodyPartsNode = "BodyParts";
  const string framesNode = "Frames";
  const string bodyJointNode = "BodyJoint";
  const string bodyPartNode = "BodyPart";
  const string frameNode = "Frame";
  const string bodyJointIdParam = "id";
  const string bodyJointNameParam = "name";
  const string bodyPartNameParam = "name";
  const string bodyPartIdParam = "id";
  const string bodyPartParentJointIdParam = "parentJointId";
  const string bodyPartChildJointIdParam = "childJointId";
  const string bodyJointDepthSignParam = "depthSign";
  const string bodyPartExpectedDistanceParam = "expectedDistance";
  const string bodyPartIsOccludedParam = "isOccluded";
  const string bodyPartLWRatioParam = "lwRatio";
  const string bodyPartRelativeLengthParam = "relativeLength";
  const string frameIdParam = "id";
  const string frameImgPathParam = "imgPath";
  const string frameMaskPathParam = "maskPath";
  const string frameCamPathParam = "camPath";
  const string frameIsKeyframeParam = "isKeyframe";
  const string frameGPX = "gpX";
  const string frameGPY = "gpY";
  const string bodyJointXParam = "x";
  const string bodyJointYParam = "y";

  tinyxml2::XMLDocument project;
  XMLError status = project.LoadFile(fileName.c_str());
  if (status != 0)
  {
    cerr << "Could not load the project from " << fileName << endl;
    cerr << "Error code: " << status << endl;
    return false;
  }
  XMLElement *root = project.RootElement();
  if (root == 0 || root->Name() != projectNode)
  {
    cerr << "Incorrect xml structure" << endl;
    if (root == 0)
    {
      cerr << "Empty xml" << endl;
    }
    else
    {
      cerr << "Expect: " << projectNode << endl;
      cerr << "Got: " << root->Name() << endl;
    }
    return false;
  }
  XMLNode *bodyJoints = root->FirstChildElement(bodyJointsNode.c_str());
  if (bodyJoints != 0)
  {
    bodyJoints = bodyJoints->FirstChildElement(bodyJointNode.c_str());
    if (bodyJoints == 0)
    {
      cerr << "Incorrect xml structure" << endl;
      cerr << "Couldn't find BodyJoint structure" << endl;
      return false;
    }
  }
  else
  {
    cerr << "Incorrect xml structure" << endl;
    cerr << "Couldn't find BodyJoints structure" << endl;
    return false;
  }
  XMLNode *bodyParts = root->FirstChildElement(bodyPartsNode.c_str());
  if (bodyParts != 0)
  {
    bodyParts = bodyParts->FirstChildElement(bodyPartNode.c_str());
    if (bodyParts == 0)
    {
      cerr << "Incorrect xml structure" << endl;
      cerr << "Couldn't find BodyPart structure" << endl;
      return false;
    }
  }
  else
  {
    cerr << "Incorrect xml structure" << endl;
    cerr << "Couldn't find BodyParts strunture" << endl;
    return false;
  }
  XMLNode *frames = root->FirstChildElement(framesNode.c_str());
  if (frames != 0)
  {
    frames = frames->FirstChildElement(frameNode.c_str());
    if (frames == 0)
    {
      cerr << "Incorrect xml structure" << endl;
      cerr << "Couldn't find Frame structure" << endl;
      return false;
    }
  }
  else
  {
    cerr << "Incorrect xml structure" << endl;
    cerr << "Couldn't find Frames structure" << endl;
    return false;
  }
  projectTitle = root->Attribute(projectName.c_str());
  imgFolderPath = root->Attribute(projectImgFolderPath.c_str());
  if (imgFolderPath.size() > 0 && imgFolderPath.back() != '/')
  {
    imgFolderPath += '/';
  }
  maskFolderPath = root->Attribute(projectMaskFolderPath.c_str());
  if (maskFolderPath.size() > 0 && maskFolderPath.back() != '/')
  {
    maskFolderPath += '/';
  }
  camFolderPath = root->Attribute(projectCamFolderPath.c_str());
  if (camFolderPath.size() > 0 && camFolderPath.back() != '/')
  {
    camFolderPath += '/';
  }
  allowScaling = root->BoolAttribute(projectAllowScaling.c_str());
  simMathPath = root->Attribute(projectSimMathPath.c_str());
  if (simMathPath.size() > 0 && simMathPath.back() != '/')
  {
    simMathPath += '/';
  }
  exportPath = root->Attribute(projectExportPath.c_str());
  if (exportPath.size() > 0 && exportPath.back() != '/')
  {
    exportPath += '/';
  }
  tree <BodyJoint> trBodyJoints, trBodyJointsCopy;
  tree <BodyJoint>::iterator topBodyJoints;
  topBodyJoints = trBodyJoints.begin();
  while (true)
  {
    if (bodyJoints == 0) break;
    BodyJoint joint;
    XMLElement *e = bodyJoints->ToElement();
    int id;
    id = e->IntAttribute(bodyJointIdParam.c_str());
    joint.setLimbID(id);
    string name;
    name = e->Attribute(bodyJointNameParam.c_str());
    joint.setJointName(name);
    trBodyJoints.insert(topBodyJoints, joint);
    bodyJoints = bodyJoints->NextSiblingElement();
  }
  tree <BodyPart> trBodyParts, trBodyPartsCopy;
  tree <BodyPart>::iterator topBodyParts;
  list <BodyPart> vBodyParts;
  topBodyParts = trBodyParts.begin();
  while (true)
  {
    if (bodyParts == 0) break;
    BodyPart part;
    XMLElement *e = bodyParts->ToElement();
    int parentJointId, childJointId;
    string name;
    int id;
    float expectedDistance;
    float lwRatio;
    float relativeLength;
    id = e->IntAttribute(bodyPartIdParam.c_str());
    name = e->Attribute(bodyPartNameParam.c_str());
    parentJointId = e->IntAttribute(bodyPartParentJointIdParam.c_str());
    childJointId = e->IntAttribute(bodyPartChildJointIdParam.c_str());
    expectedDistance = e->FloatAttribute(bodyPartExpectedDistanceParam.c_str());
    lwRatio = e->FloatAttribute(bodyPartLWRatioParam.c_str());
    relativeLength = e->FloatAttribute(bodyPartRelativeLengthParam.c_str());
    BodyJoint *parentJoint = 0, *childJoint = 0;
    for (topBodyJoints = trBodyJoints.begin(); topBodyJoints != trBodyJoints.end(); ++topBodyJoints)
    {
      if (parentJoint == 0 && topBodyJoints->getLimbID() == parentJointId)
      {
        parentJoint = &*topBodyJoints;
      }
      else if (childJoint == 0 && topBodyJoints->getLimbID() == childJointId)
      {
        childJoint = &*topBodyJoints;
      }
      if (parentJoint != 0 && childJoint != 0) break;
    }
    if (parentJoint == 0 || childJoint == 0)
    {
      cerr << "Could not find BodyJoint " << ((parentJoint == 0) ? parentJointId : childJointId) << endl;
      return false;
    }
    part.setPartID(id);
    part.setPartName(name);
    part.setParentJoint(parentJoint->getLimbID());
    part.setChildJoint(childJoint->getLimbID());
    part.setExpectedDistance(expectedDistance);
    part.setLWRatio(lwRatio);
    part.setRelativeLength(relativeLength);
    //trBodyParts.insert(topBodyParts, part);
    vBodyParts.push_back(part);
    bodyParts = bodyParts->NextSiblingElement();
  }
  BuildBodyPartTree(vBodyParts, trBodyParts, topBodyParts);
  int32_t firstFrameCols = -1;
  int32_t firstFrameRows = -1;
  while (true)
  {
    if (frames == 0) break;
    Frame *f = 0;
    XMLElement *e = frames->ToElement();
    string imgPath, maskPath, camPath;
    int id;
    bool isKeyFrame;
    Point2f gp;
    id = e->IntAttribute(frameIdParam.c_str());
    imgPath = e->Attribute(frameImgPathParam.c_str());
    maskPath = e->Attribute(frameMaskPathParam.c_str());
    camPath = e->Attribute(frameCamPathParam.c_str());
    isKeyFrame = e->BoolAttribute(frameIsKeyframeParam.c_str());
    gp.x = e->FloatAttribute(frameGPX.c_str());
    gp.y = e->FloatAttribute(frameGPY.c_str());
    if (isKeyFrame == true)
    {
      f = new Keyframe();
    }
    else
    {
      f = new Interpolation();
    }
    f->setID(id);
    Mat image = imread(curFolder + imgFolderPath + imgPath, CV_LOAD_IMAGE_COLOR);
    if (!image.data)
    {
      cerr << "Could not find file " << imgFolderPath + imgPath << endl;
      return false;
    }
    int32_t imageOriginalCols = image.cols;
    int32_t imageOriginalRows = image.rows;
    ResizeImage(image, firstFrameCols, firstFrameRows);
    f->setImage(image);
    Mat mask = imread(curFolder + maskFolderPath + maskPath, CV_LOAD_IMAGE_GRAYSCALE);
    if (!mask.data)
    {
      cerr << "Could not find file " << maskFolderPath + maskPath << endl;
      return false;
    }
    ResizeImage(mask, firstFrameCols, firstFrameRows);
    f->setMask(mask);
    f->setGroundPoint(gp);
    spelHelper::copyTree(trBodyJointsCopy, trBodyJoints);
    if (isKeyFrame)
    {
      bodyJoints = frames->FirstChildElement(bodyJointsNode.c_str());
      bodyJoints = bodyJoints->FirstChildElement(bodyJointNode.c_str());
      topBodyJoints = trBodyJointsCopy.begin();
      while (true)
      {
        if (bodyJoints == 0) break;
        XMLElement *e = bodyJoints->ToElement();
        int id;
        float x, y;
        bool depthSign;
        id = e->IntAttribute(bodyJointIdParam.c_str());
        x = e->FloatAttribute(bodyJointXParam.c_str());
        y = e->FloatAttribute(bodyJointYParam.c_str());
        if (imageOriginalCols != image.cols || imageOriginalRows != image.rows)
        {
          float colsFactor = (float)(image.cols / imageOriginalCols);
          float rowsFactor = (float)(image.rows / imageOriginalRows);
          x *= colsFactor;
          y *= rowsFactor;
        }
        depthSign = e->BoolAttribute(bodyJointDepthSignParam.c_str());
        BodyJoint *joint = 0;
        for (topBodyJoints = trBodyJointsCopy.begin(); topBodyJoints != trBodyJointsCopy.end(); ++topBodyJoints)
        {
          if (joint == 0 && topBodyJoints->getLimbID() == id)
          {
            joint = &*topBodyJoints;
          }
          if (joint != 0) break;
        }
        if (joint == 0)
        {
          cerr << "Could not find BodyJoint " << id;
          return false;
        }
        Point2f imgLocation = Point2f(x, y);
        joint->setImageLocation(imgLocation);
        joint->setDepthSign(depthSign);
        bodyJoints = bodyJoints->NextSiblingElement();
      }
    }
    spelHelper::copyTree(trBodyPartsCopy, trBodyParts);
    if (isKeyFrame)
    {
      bodyParts = frames->FirstChildElement(bodyPartsNode.c_str());
      bodyParts = bodyParts->FirstChildElement(bodyPartNode.c_str());
      topBodyParts = trBodyPartsCopy.begin();
      while (true)
      {
        if (bodyParts == 0) break;
        XMLElement *e = bodyParts->ToElement();
        int id;
        bool isOccluded;
        id = e->IntAttribute(bodyPartIdParam.c_str());
        isOccluded = e->BoolAttribute(bodyPartIsOccludedParam.c_str());
        BodyPart *part = 0;
        for (topBodyParts = trBodyPartsCopy.begin(); topBodyParts != trBodyPartsCopy.end(); ++topBodyParts)
        {
          if (part == 0 && topBodyParts->getPartID() == id)
          {
            part = &*topBodyParts;
          }
          if (part != 0) break;
        }
        if (part == 0)
        {
          cerr << "Could not find BodyPart " << id;
          return false;
        }
        part->setIsOccluded(isOccluded);
        bodyParts = bodyParts->NextSiblingElement();
      }
    }
    Skeleton skeleton;
    skeleton.setJointTree(trBodyJointsCopy);
    skeleton.setPartTree(trBodyPartsCopy);
    //TODO (Vitaliy Koshura): This need to be loaded!!!!!!
    skeleton.setScale(100.0);
    f->setSkeleton(skeleton);
    vFrames.push_back(f);
    frames = frames->NextSiblingElement();

    image.release();
    mask.release();
  }

  //  kptree::print_tree_bracketed(trBodyParts);
  //  cout << endl << "end of tree" << endl;

  return true;
}

bool ProjectLoader::Save(map <uint32_t, vector <LimbLabel>> labels, string outFolder, int frameID)
{
  for (auto lls = labels.begin(); lls != labels.end(); ++lls)
  {
    if (lls->second.size() == 0)
    {
      continue;
    }
    ofstream outFile;
    CreateDirectorySystemIndependent(outFolder);
    string outFileName = outFolder;
    if (outFileName[outFileName.size()] != '/')
      outFileName += "/";
    stringstream ss;
    ss << frameID;
    ss << "-";
    try
    {
      ss << lls->second.begin()->getLimbID();
    }
    catch (...)
    {
      cerr << "Can't get limb id" << endl;
      continue;
    }
    outFileName += ss.str();
    outFile.open(outFileName);
    cerr << "Writing file: " << ss.str() << endl;
    for (auto ls = lls->second.begin(); ls != lls->second.end(); ++ls)
    {
      try
      {
        outFile << ls->getLimbID() << " ";
        outFile << ls->getCenter().x << " ";
        outFile << ls->getCenter().y << " ";
        outFile << ls->getAngle() << " ";
        if (ls->getPolygon().size() < 4)
        {
          outFile << "0" << " ";
        }
        else
        {
          outFile << sqrt(spelHelper::distSquared(ls->getPolygon()[0], ls->getPolygon()[2])) << " ";
        }
        outFile << ls->getAvgScore() << " ";
        outFile << "0" << " ";
        outFile << "0" << " ";
        outFile << ((ls->getIsOccluded() == true) ? 0 : 1);
        outFile << std::endl;
      }
      catch (...)
      {
        cerr << "Empty LimbLabel" << endl;
      }
    }
    outFile.close();
  }
  return true;
}


bool ProjectLoader::drawFrameSolvlets(Solvlet sol, Frame *frame, string outFolder, Scalar color, int lineWidth)
{
  //draw
  CreateDirectorySystemIndependent(outFolder);
  string outFileName = outFolder;
  if (outFileName[outFileName.size()] != '/')
    outFileName += "/";
  outFileName += "img/";
  CreateDirectorySystemIndependent(outFileName);
  outFileName += "frameSolvlets/";
  CreateDirectorySystemIndependent(outFileName);
  stringstream ss;
  ss << frame->getID();
  ss << ".png";
  outFileName += ss.str();
  Mat image;

  try
  {
    image = frame->getImage();
  }
  catch (...)
  {
    cerr << "Can't get image from frame" << endl;
    return false;
  }

  vector<LimbLabel> labels = sol.getLabels();
  vector<LimbLabel>::iterator ls;

  for (ls = labels.begin(); ls != labels.end(); ++ls)
  {
    Point2f p1, p2, p3, p4;
    vector <Point2f> polygon;
    try
    {
      polygon = ls->getPolygon();
    }
    catch (...)
    {
      cerr << "Can't get polygon" << endl;
      continue;
    }
    try
    {
      p1 = polygon.at(0);
    }
    catch (...)
    {
      cerr << "Can't get first point from polygon" << endl;
      continue;
    }
    try
    {
      p2 = polygon.at(1);
    }
    catch (...)
    {
      cerr << "Can't get second point from polygon" << endl;
      continue;
    }
    try
    {
      p3 = polygon.at(2);
    }
    catch (...)
    {
      cerr << "Can't get third point from polygon" << endl;
      continue;
    }
    try
    {
      p4 = polygon.at(3);
    }
    catch (...)
    {
      cerr << "Can't get fourth point from polygon" << endl;
      continue;
    }
    // //temporary comment
    //    line(image, p1, p2, color, lineWidth, CV_AA);
    //    line(image, p2, p3, color, lineWidth, CV_AA);
    //    line(image, p3, p4, color, lineWidth, CV_AA);
    //    line(image, p4, p1, color, lineWidth, CV_AA);

    line(image, p1, p2, Scalar(255, 0, 0), lineWidth, CV_AA);
    line(image, p2, p3, Scalar(0, 255, 0), lineWidth, CV_AA);
    line(image, p3, p4, Scalar(0, 0, 255), lineWidth, CV_AA);
    line(image, p4, p1, Scalar(255, 0, 255), lineWidth, CV_AA);
  }

  cerr << "Writing file " << outFileName << endl;
  if (frame->getParentFrameID() != -1)
    cerr << "Parent Frame ID: " << frame->getParentFrameID() << endl;
  return imwrite(outFileName, image);
}

bool ProjectLoader::drawLockframeSolvlets(ImageSimilarityMatrix ism, Solvlet sol, Frame *frame, Frame * parentFrame, string outFolder, Scalar color, int lineWidth)
{
  //draw
  CreateDirectorySystemIndependent(outFolder);
  string outFileName = outFolder;
  if (outFileName[outFileName.size()] != '/')
    outFileName += "/";
//  outFileName += "img/";
//  CreateDirectorySystemIndependent(outFileName);
//  outFileName += "lockframeSolvlets/";
//  CreateDirectorySystemIndependent(outFileName);

  string labelsFileName = outFileName + to_string(frame->getID()) + ".bpsol";
  outFileName += to_string(frame->getID()) + ".png";

  Mat image;

  try
  {
    image = frame->getImage();
  }
  catch (...)
  {
    cerr << "Can't get image from frame" << endl;
    return false;
  }

  ofstream out;
  try
  {
    out.open(labelsFileName);
  }
  catch (...)
  {
    cerr << "Can't open bpsol file for writing" << endl;
    return false;
  }

  vector<LimbLabel> labels = sol.getLabels();
  vector<LimbLabel>::iterator ls;

  for (ls = labels.begin(); ls != labels.end(); ++ls)
  {
    Point2f p1, p2, p3, p4;
    vector <Point2f> polygon;
    Point2f c0, c1;
    ls->getEndpoints(c0, c1);
    try
    {
      polygon = ls->getPolygon();
    }
    catch (...)
    {
      cerr << "Can't get polygon" << endl;
      continue;
    }
    try
    {
      p1 = polygon.at(0);
    }
    catch (...)
    {
      cerr << "Can't get first point from polygon" << endl;
      continue;
    }
    try
    {
      p2 = polygon.at(1);
    }
    catch (...)
    {
      cerr << "Can't get second point from polygon" << endl;
      continue;
    }
    try
    {
      p3 = polygon.at(2);
    }
    catch (...)
    {
      cerr << "Can't get third point from polygon" << endl;
      continue;
    }
    try
    {
      p4 = polygon.at(3);
    }
    catch (...)
    {
      cerr << "Can't get fourth point from polygon" << endl;
      continue;
    }
    // //temporary comment
    //    line(image, p1, p2, color, lineWidth, CV_AA);
    //    line(image, p2, p3, color, lineWidth, CV_AA);
    //    line(image, p3, p4, color, lineWidth, CV_AA);
    //    line(image, p4, p1, color, lineWidth, CV_AA);

    line(image, p1, p2, Scalar(255, 0, 0), lineWidth, CV_AA);
    line(image, p2, p3, Scalar(0, 255, 0), lineWidth, CV_AA);
    line(image, p3, p4, Scalar(0, 0, 255), lineWidth, CV_AA);
    line(image, p4, p1, Scalar(255, 0, 255), lineWidth, CV_AA);

    circle(image, c0, 2, Scalar(255, 0, 255), lineWidth, CV_AA);
    circle(image, c1, 2, Scalar(0, 255, 0), lineWidth, CV_AA);

    //and write this label to file
    out << ls->toString() << endl;
  }
  out.close(); //close stream

  //solvlets are draw, now draw the skeleton from parent frame

  //ImageSimilarityMatrix ism(seq.getFrames());

  Point2f shift = ism.getShift(parentFrame->getID(), frame->getID());

  //  Frame* newFrame = new Lockframe();
  //  *newFrame = *parentFrame;

  parentFrame->shiftSkeleton2D(shift); //shift the skeleton by the correct amount

  Skeleton parentSkel = parentFrame->getSkeleton(); //this is the shifted skeleton
  Skeleton skel = frame->getSkeleton();
  //Skeleton skel = vFrames[frame->getID()].getSkeleton();

  tree<BodyPart> partTree = parentSkel.getPartTree();

  //draw the skeletal prior
  for (tree<BodyPart>::iterator partIter = partTree.begin(); partIter != partTree.end(); ++partIter)
  {
    //get joints
    int j0, j1;
    j0 = partIter->getParentJoint();
    j1 = partIter->getChildJoint();

    Point2f p0, p1;

    p0 = parentSkel.getBodyJoint(j0)->getImageLocation();
    p1 = parentSkel.getBodyJoint(j1)->getImageLocation();

    line(image, p0, p1, Scalar(0, 255, 255), lineWidth, CV_AA);

    //    //draw the child and parent joints
    //    circle(image, p0, 2, Scalar(255, 0, 255), lineWidth, CV_AA);
    //    circle(image, p1, 2, Scalar(0, 255, 0), lineWidth, CV_AA);

    //draw angle arcs
    float pixelRadius = partIter->getSearchRadius();
    float angleRadius = partIter->getRotationSearchRange();

    circle(image, 0.5*p0 + 0.5*p1, (int)pixelRadius, Scalar(0, 0, 0, 100), lineWidth, CV_AA);
    double startAngleUpright = spelHelper::angle2D(1.0, 0.0, (p1 - p0).x, (p1 - p0).y)*180.0 / M_PI;

    ellipse(image, 0.5*p0 + 0.5*p1, cv::Size((int)pixelRadius, (int)pixelRadius), 0, startAngleUpright - angleRadius, startAngleUpright + angleRadius, Scalar(0, 255, 0), lineWidth, CV_AA);
    ellipse(image, 0.5*p0 + 0.5*p1, cv::Size((int)pixelRadius, (int)pixelRadius), 0, startAngleUpright - 2, startAngleUpright + 2, Scalar(0, 0, 255), lineWidth*0.5, CV_AA); //draw a 1 degree tick mark
  }

  //draw the skeleton resulting from solve
  partTree = skel.getPartTree();
  tree<BodyJoint> jointTree = skel.getJointTree();
  for (tree<BodyPart>::iterator partIter = partTree.begin(); partIter != partTree.end(); ++partIter)
  {
    //get joints
    int j0, j1;
    j0 = partIter->getParentJoint();
    j1 = partIter->getChildJoint();

    Point2f p0, p1;

    p0 = skel.getBodyJoint(j0)->getImageLocation();
    p1 = skel.getBodyJoint(j1)->getImageLocation();

    line(image, p0, p1, Scalar(0, 255, 0), lineWidth, CV_AA);
  }

  cerr << "Writing file " << outFileName << endl;
  if (frame->getParentFrameID() != -1)
    cerr << "Parent Frame ID: " << frame->getParentFrameID() << endl;

  parentFrame->shiftSkeleton2D(-shift); //undo the shift

  return imwrite(outFileName, image);
}

bool ProjectLoader::drawSkeleton(Frame *frame, string outFolder, Scalar color, int lineWidth)
{
  //draw
  CreateDirectorySystemIndependent(outFolder);
  string outFileName = outFolder;
  if (outFileName[outFileName.size()] != '/')
    outFileName += "/";

  outFileName += to_string(frame->getID()) + ".png";

  Mat image;

  try
  {
    image = frame->getImage();
  }
  catch (...)
  {
    cerr << "Can't get image from frame" << endl;
    return false;
  }

  Skeleton skel = frame->getSkeleton();

  tree<BodyPart> partTree = skel.getPartTree();

  Scalar col((0, 255, 255)); //yellow for frames

  if(frame->getFrametype()==KEYFRAME)
      col = Scalar(0, 0, 255); //red for keyframes

  //draw the skeletal prior
  for (tree<BodyPart>::iterator partIter = partTree.begin(); partIter != partTree.end(); ++partIter)
  {
    //get joints
    int j0, j1;
    j0 = partIter->getParentJoint();
    j1 = partIter->getChildJoint();

    Point2f p0, p1;

    p0 = skel.getBodyJoint(j0)->getImageLocation();
    p1 = skel.getBodyJoint(j1)->getImageLocation();

    line(image, p0, p1, col, lineWidth, CV_AA);

    //    //draw the child and parent joints
    circle(image, p0, lineWidth, col, lineWidth, CV_AA);
    circle(image, p1, lineWidth, col, lineWidth, CV_AA);

    //draw angle arcs
    //float pixelRadius = partIter->getSearchRadius();
    //float angleRadius = partIter->getRotationSearchRange();

    //circle(image, 0.5*p0 + 0.5*p1, (int)pixelRadius, Scalar(0, 0, 0, 100), lineWidth, CV_AA);
    //double startAngleUpright = spelHelper::angle2D(1.0, 0.0, (p1 - p0).x, (p1 - p0).y)*180.0 / M_PI;

    //ellipse(image, 0.5*p0 + 0.5*p1, cv::Size((int)pixelRadius, (int)pixelRadius), 0, startAngleUpright - angleRadius, startAngleUpright + angleRadius, Scalar(0, 255, 0), lineWidth, CV_AA);
    //ellipse(image, 0.5*p0 + 0.5*p1, cv::Size((int)pixelRadius, (int)pixelRadius), 0, startAngleUpright - 2, startAngleUpright + 2, Scalar(0, 0, 255), lineWidth*0.5, CV_AA); //draw a 1 degree tick mark
  }

  cerr << "Writing file " << outFileName << endl;

  return imwrite(outFileName, image);
}




bool ProjectLoader::Draw(map <uint32_t, vector <LimbLabel>> labels, Frame *frame, string outFolder, int frameID, Scalar color, Scalar optimalColor, int lineWidth)
{
  CreateDirectorySystemIndependent(outFolder);
  string outFileName = outFolder;
  if (outFileName[outFileName.size()] != '/')
    outFileName += "/";
  outFileName += "img/";
  CreateDirectorySystemIndependent(outFileName);
  outFileName += "detectorOutput/";
  CreateDirectorySystemIndependent(outFileName);
  string tempFileName = outFileName;

  stringstream ss;
  ss << frameID;
  ss << ".png";
  outFileName += ss.str();
  Mat image, tempImage;
  try
  {
    image = frame->getImage().clone();
    tempImage = image.clone();
  }
  catch (...)
  {
    cerr << "Can't get image from frame" << endl;
    return false;
  }

  Skeleton s = frame->getSkeleton();
  tree <BodyPart> bp = s.getPartTree();
  Mat temp = tempImage.clone();
  for (tree<BodyPart>::iterator bpi = bp.begin(); bpi != bp.end(); ++bpi)
  {
    Point2f j1, j0;  // temporary adjacent joints
    BodyJoint *joint = 0; // temporary conserve a joints
    joint = s.getBodyJoint(bpi->getParentJoint()); // the parent node of current body part pointer

    if (joint == 0)
      break; // a joint has no marking on the frame

    j0 = joint->getImageLocation(); // coordinates of current joint
    joint = 0;
    joint = s.getBodyJoint(bpi->getChildJoint()); // the child node of current body part pointer

    if (joint == 0)
      break; // a joint has no marking on the frame

    j1 = joint->getImageLocation(); // coordinates of current joint
    float boneLength = (j0 == j1) ? 1.0f : (float)sqrt(spelHelper::distSquared(j0, j1)); // distance between nodes
    //TODO (Vitaliy Koshura): Check this!
    float boneWidth;
    try
    { //currents body part polygon width
      float ratio = (*bpi).getLWRatio();
      if (ratio == 0)
      {
        stringstream ss;
        ss << "Ratio can't be 0";
#ifdef DEBUG
        cerr << ss.str() << endl;
#endif  // DEBUG
      }
      boneWidth = boneLength / ratio;
    }
    catch (...)
    {
      stringstream ss;
      ss << "Can't get LWRatio value";
      cerr << ss.str() << endl;
      throw logic_error(ss.str());
    }
    Point2f direction = j1 - j0; // used as estimation of the vector's direction
    float rotationAngle = float(spelHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI)); //bodypart tilt angle
    bpi->setRotationSearchRange(rotationAngle);

    Point2f boxCenter = j0 * 0.5 + j1 * 0.5;

    float angle = float(spelHelper::angle2D(1, 0, j1.x - j0.x, j1.y - j0.y) * (180.0 / M_PI));
    Point2f c1, c2, c3, c4, polyCenter;
    c1 = Point2f(0.f, 0.5f * boneWidth);
    c2 = Point2f(boneLength, 0.5f * boneWidth);
    c3 = Point2f(boneLength, -0.5f * boneWidth);
    c4 = Point2f(0.f, -0.5f * boneWidth);
    polyCenter = Point2f(boneLength * 0.5f, 0.f);
    c1 = spelHelper::rotatePoint2D(c1, polyCenter, angle) + boxCenter - polyCenter;
    c2 = spelHelper::rotatePoint2D(c2, polyCenter, angle) + boxCenter - polyCenter;
    c3 = spelHelper::rotatePoint2D(c3, polyCenter, angle) + boxCenter - polyCenter;
    c4 = spelHelper::rotatePoint2D(c4, polyCenter, angle) + boxCenter - polyCenter;
    POSERECT <Point2f> rect = POSERECT <Point2f>(c1, c2, c3, c4);

    line(temp, rect.point1, rect.point2, Scalar(255, 0, 0), lineWidth, CV_AA);
    line(temp, rect.point2, rect.point3, Scalar(0, 255, 0), lineWidth, CV_AA);
    line(temp, rect.point3, rect.point4, Scalar(0, 0, 255), lineWidth, CV_AA);
    line(temp, rect.point4, rect.point1, Scalar(255, 0, 255), lineWidth, CV_AA);
  }
  string fileName = tempFileName;
  fileName += "skeleton/";
  CreateDirectorySystemIndependent(fileName);
  stringstream ssTemp;
  ssTemp << frameID << ".png";
  fileName += ssTemp.str();

  cerr << "Writing file " << fileName << endl;

  imwrite(fileName, temp);

  temp.release();

  for (auto lls = labels.begin(); lls != labels.end(); ++lls)
  {
    if (lls->second.size() == 0)
    {
      continue;
    }
    int j = 0;
    for (auto ls = lls->second.begin(); ls != lls->second.end(); ++ls)
    {
      Point2f p1, p2, p3, p4;
      vector <Point2f> polygon;
      try
      {
        polygon = ls->getPolygon();
      }
      catch (...)
      {
        cerr << "Can't get polygon" << endl;
        continue;
      }
      try
      {
        p1 = polygon.at(0);
      }
      catch (...)
      {
        cerr << "Can't get first point from polygon" << endl;
        continue;
      }
      try
      {
        p2 = polygon.at(1);
      }
      catch (...)
      {
        cerr << "Can't get second point from polygon" << endl;
        continue;
      }
      try
      {
        p3 = polygon.at(2);
      }
      catch (...)
      {
        cerr << "Can't get third point from polygon" << endl;
        continue;
      }
      try
      {
        p4 = polygon.at(3);
      }
      catch (...)
      {
        cerr << "Can't get fourth point from polygon" << endl;
        continue;
      }

      Mat temp = tempImage.clone();
      line(temp, p1, p2, Scalar(255, 0, 0), lineWidth, CV_AA);
      line(temp, p2, p3, Scalar(0, 255, 0), lineWidth, CV_AA);
      line(temp, p3, p4, Scalar(0, 0, 255), lineWidth, CV_AA);
      line(temp, p4, p1, Scalar(255, 0, 255), lineWidth, CV_AA);

      string fileName = tempFileName;
      fileName += "raw/";
      CreateDirectorySystemIndependent(fileName);
      stringstream ssTemp;
      ssTemp << frameID << "/";
      fileName += ssTemp.str();
      CreateDirectorySystemIndependent(fileName);
      ssTemp.str(string());
      ssTemp.clear();
      ssTemp << ls->getLimbID() << "/";
      fileName += ssTemp.str();
      CreateDirectorySystemIndependent(fileName);
      ssTemp.str(string());
      ssTemp.clear();
      ssTemp << j << ".png";
      fileName += ssTemp.str();

      cerr << "Writing file " << fileName << endl;

      imwrite(fileName, temp);

      temp.release();

      line(image, p1, p2, color, lineWidth, CV_AA);
      line(image, p2, p3, color, lineWidth, CV_AA);
      line(image, p3, p4, color, lineWidth, CV_AA);
      line(image, p4, p1, color, lineWidth, CV_AA);

      j++;
    }
  }
  for (auto lls = labels.begin(); lls != labels.end(); ++lls)
  {
    if (lls->second.size() == 0)
    {
      continue;
    }
    for (auto ls = lls->second.begin(); ls != lls->second.end(); ++ls)
    {
      Point2f p1, p2, p3, p4;
      vector <Point2f> polygon;
      try
      {
        polygon = ls->getPolygon();
      }
      catch (...)
      {
        cerr << "Can't get polygon" << endl;
        continue;
      }
      try
      {
        p1 = polygon.at(0);
      }
      catch (...)
      {
        cerr << "Can't get first point from polygon" << endl;
        continue;
      }
      try
      {
        p2 = polygon.at(1);
      }
      catch (...)
      {
        cerr << "Can't get second point from polygon" << endl;
        continue;
      }
      try
      {
        p3 = polygon.at(2);
      }
      catch (...)
      {
        cerr << "Can't get third point from polygon" << endl;
        continue;
      }
      try
      {
        p4 = polygon.at(3);
      }
      catch (...)
      {
        cerr << "Can't get fourth point from polygon" << endl;
        continue;
      }
      line(image, p1, p2, optimalColor, lineWidth, CV_AA);
      line(image, p2, p3, optimalColor, lineWidth, CV_AA);
      line(image, p3, p4, optimalColor, lineWidth, CV_AA);
      line(image, p4, p1, optimalColor, lineWidth, CV_AA);
      break;
    }
  }

  cerr << "Writing file " << outFileName << endl;

  bool result = imwrite(outFileName, image);

  image.release();
  tempImage.release();

  return result;
}

void ProjectLoader::ResizeImage(Mat &image, int32_t &cols, int32_t &rows)
{
  if (cols <= 0 || rows <= 0)
  {
    cols = image.cols;
    rows = image.rows;
  }
  if (cols != image.cols || rows != image.rows)
  {
    resize(image, image, cvSize(cols, rows));
  }
}

void ProjectLoader::BuildBodyPartTree(list <BodyPart> vBodyParts, tree <BodyPart> &trBodyPart, tree <BodyPart>::iterator &root)
{
  if (vBodyParts.size() == 0)
    return;
  try
  {
    list <BodyPart>::iterator i = vBodyParts.begin();
    while (i != vBodyParts.end())
    {
      if (i->getPartID() != 0)
        continue;
      tree <BodyPart>::iterator parent = trBodyPart.insert(root, *vBodyParts.begin());
      //cerr << trBodyPart.size() << endl;
      vBodyParts.remove(*vBodyParts.begin());
      AddChildBodyPartsToTree(vBodyParts, trBodyPart, parent);
      if (vBodyParts.size() > 0)
      {
        cerr << "Not all BodyParts were parsed" << endl;
        return;
      }
      break;
    }
  }
  catch (...)
  {
    cerr << "Can't get BodyPart of index 0 from vBodyParts" << endl;
    return;
  }
}
void ProjectLoader::AddChildBodyPartsToTree(list <BodyPart> &vBodyParts, tree <BodyPart> &trBodyPart, tree <BodyPart>::iterator &parent)
{
  list <BodyPart>::iterator i = vBodyParts.begin();
  while (i != vBodyParts.end())
  {
    // it's a root
    if (parent->getPartID() == trBodyPart.begin()->getPartID())
    {
      if (parent->getParentJoint() == i->getParentJoint() || parent->getChildJoint() == i->getParentJoint())
      {
        tree <BodyPart>::iterator inserted = trBodyPart.append_child(parent, *i);
        vBodyParts.remove(*i);
        AddChildBodyPartsToTree(vBodyParts, trBodyPart, inserted);
        i = vBodyParts.begin();
      }
      else
      {
        ++i;
      }
    }
    else if (parent->getChildJoint() == i->getParentJoint())
    {
      tree <BodyPart>::iterator inserted = trBodyPart.append_child(parent, *i);
      vBodyParts.remove(*i);
      AddChildBodyPartsToTree(vBodyParts, trBodyPart, inserted);
      i = vBodyParts.begin();
    }
    else
    {
      ++i;
    }
  }
}

bool ProjectLoader::CreateDirectorySystemIndependent(string dirName)
{
#ifdef WINDOWS
  if ((CreateDirectory(dirName.c_str(), NULL) == 0) &&
    (GetLastError() == ERROR_ALREADY_EXISTS))
  {
    return false;
  }
  return true;
#endif // WINDOWS
#ifdef UNIX
  if ((mkdir(dirName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0) && (errno == EACCES))
  {
    return false;
  }
  return true;
#endif // UNIX
}

string ProjectLoader::getProjectTitle()
{
  return this->projectTitle;
}

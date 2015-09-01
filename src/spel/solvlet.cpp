#include "solvlet.hpp"

namespace SPEL
{

  Solvlet::Solvlet(void)
  {
    frameId = -1;
  }
  Solvlet::Solvlet(int id, vector<LimbLabel> _labels)
  {
    frameId = id;
    labels = _labels;
  }

  Solvlet::~Solvlet(void)
  {
  }

  Solvlet &Solvlet::operator=(const Solvlet &s)
  {
    if (this == &s)
    {
      return *this;
    }
    this->setLabels(s.getLabels());
    this->setFrameID(s.getFrameID());
    return *this;
  }

  bool Solvlet::operator<(const Solvlet &s) const
  {
      return this->getFrameID()<s.getFrameID();
  }

  bool Solvlet::operator>(const Solvlet &s) const
  {
      return this->getFrameID()>s.getFrameID();
  }

  // bool operator!=(const Solvlet &s) const
  // {

  // }

  int Solvlet::getFrameID(void) const
  {
    return frameId;
  }

  void Solvlet::setFrameID(int _id)
  {
    frameId = _id;
  }

  vector<LimbLabel> Solvlet::getLabels(void) const
  {
    return labels;
  }

  const vector<LimbLabel>* Solvlet::getLabelsPtr() const{
    return &labels;
  }

  void Solvlet::setLabels(vector<LimbLabel> _labels)
  {
    labels = _labels;
  }

  Skeleton Solvlet::toSkeleton(const Skeleton &example)
  {
    Skeleton retSkel = example;

    tree<BodyPart> partTree = retSkel.getPartTree();
    tree<BodyJoint> jointTree = retSkel.getJointTree();
    tree<BodyJoint>::iterator cjIter, pjIter, jointIter;

    assert(partTree.size() == labels.size()); //there should be the same number of body parts

    for (tree<BodyPart>::iterator partIter = partTree.begin(); partIter != partTree.end(); ++partIter)
    {
      for (auto i = 0; i < labels.size(); ++i)
      {
        if (partIter->getPartID() == labels[i].getLimbID()) //if you find the right label
        {
          //get joint IDs
          int partID = partIter->getPartID(); //part
          int cJointID = partIter->getChildJoint(); //child joint
          int pJointID = partIter->getParentJoint(); //parent joint

          Point2f pj, cj; //parent and child joints from label
          labels[i].getEndpoints(pj, cj); //set them from label


          cjIter = jointTree.end();
          pjIter = jointTree.end();

          //identify these nodes in the joint tree
          for (jointIter = jointTree.begin(); jointIter != jointTree.end(); ++jointIter)
          {
            if (jointIter->getLimbID() == cJointID)
              cjIter = jointIter;
            if (jointIter->getLimbID() == pJointID)
              pjIter = jointIter;
            if (cjIter != jointTree.end() && pjIter != jointTree.end())
              break;
          }

          if (partID == 0) //root
          {
            //set 2D joint locations
            pjIter->setImageLocation(pj); //set parent joint only for root node
          }
          cjIter->setImageLocation(cj); //set child joint for all other nodes
        } //TODO: introduce a more complex scheme of doing this, such as finding midpoints, or points
      }
    }

    retSkel.setJointTree(jointTree);
    retSkel.infer3D();

    return retSkel;
  }

  float Solvlet::evaluateSolution(Frame* frame, map<string, float> params)
  {
      vector<LimbLabel> labels = this->getLabels();
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

  //Skeleton MainWindow::skeletonFromLabels(vector<LimbLabel> labels)
  //{
  //    QLOG_TRACE() << "skeletonFromLabels()" << endl;
  //    Skeleton skel;
  //    tree<Point3f> * partTree = skel.getPartTree();
  //    tree<Point3f>::iterator iter, parent, child;

  //    if(labels.size()!=skel.getNumBones())
  //    {
  //        cerr << "Incorrect number of labels. " << endl;
  //        return skel;
  //    }
  //    else
  //    {
  //        //build a skeleton from the labels
  //        for(iter=partTree->begin(); iter!=partTree->end(); ++iter)
  //        {
  //            //compute intersection from child to parent
  //            //making all the fake parts (1-5) simply connect the non-fake parts
  //            //this leaves the torso unmodified
  //            if(iter->x==0) //torso
  //            {
  //                Point2f x,y;
  //                labels[0].getEndpoints(x,y);
  //                skel.setImageJoint(0, x);
  //                skel.setImageJoint(1, y);
  //            }
  //            else if(iter->x<=10) //"fake" parts, just stretch from label to label, so these remain unchanged
  //            {
  //                vector<Point2i> bones(skel.getBones());
  //                LimbLabel currentLabel = labels[iter->x];
  //                Point2f cx, cy;
  //                currentLabel.getEndpoints(cx,cy);

  //                //1 and 2 connect to bottom (px)
  //                //3 4 and 5 to top (py)

  //                int pJoint = bones[iter->x].x;
  //                int cJoint = bones[iter->x].y;
  //                //we know parent is the root bone
  //                //we are always setting the y bone, the x bone is set by torso already

  //                skel.setImageJoint(pJoint,cx);
  //                skel.setImageJoint(cJoint,cy);
  //            }
  //            else
  //            {
  //                vector<Point2i> bones(skel.getBones());

  //                LimbLabel currentLabel = labels[iter->x];
  //                parent = partTree->parent(iter);
  //                LimbLabel parentLabel = labels[parent->x];

  //                Point2f px, py, cx, cy;
  //                currentLabel.getEndpoints(cx,cy);
  //                parentLabel.getEndpoints(px,py);

  //                //compute midpoint between the bottom of parent (py) and top of current bone (cx), and set it as the joint

  //                //py = skel.get2Djoints()[bones[iter->x].x];

  //                Point2f mid = 0.5*cx+0.5*py;

  //                int index = bones[iter->x].x; //first joint of child

  //                skel.setImageJoint(index, mid); //set to midpoint between the two
  //                if(iter->x>=13 || iter->x==9)
  //                {
  //                    //these are also tips, set them to label's cy
  //                    skel.setImageJoint(bones[iter->x].y, cy);
  //                }
  //            //altering orientation slightly
  //            }
  //        }
  //        return skel;
  //    }
  //    return skel;
  //}

}

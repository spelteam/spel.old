#if defined(WINDOWS) && defined(_MSC_VER)
#include <Windows.h>
#endif

#include "TestsFunctions.hpp"
  
namespace SPEL
{
  //Build bodypart rectangle on bodypart joints
  POSERECT<Point2f> BuildPartRect(BodyJoint *j0, BodyJoint *j1, float LWRatio)
  {
    Point2f p0 = j0->getImageLocation(), p1 = j1->getImageLocation();
    float boneLength = (float)sqrt(spelHelper::distSquared(p0, p1)); // distance between nodes
    float boneWidth = boneLength / LWRatio;
    Point2f boxCenter = p0 * 0.5 + p1 * 0.5; // the bobypart center  coordinates
    // Coordinates for drawing of the polygon at the coordinate origin
    Point2f c1 = Point2f(0.f, 0.5f * boneWidth);
    Point2f c2 = Point2f(boneLength, 0.5f * boneWidth);
    Point2f c3 = Point2f(boneLength, -0.5f * boneWidth);
    Point2f c4 = Point2f(0.f, -0.5f * boneWidth);
    Point2f polyCenter = Point2f(boneLength * 0.5f, 0.f); // polygon center 
    Point2f direction = p1 - p0; // used as estimation of the vector's direction
    float rotationAngle = float(spelHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI)); //bodypart tilt angle 
    // Rotate and shift the polygon to the bodypart center
    c1 = spelHelper::rotatePoint2D(c1, polyCenter, rotationAngle) + boxCenter - polyCenter;
    c2 = spelHelper::rotatePoint2D(c2, polyCenter, rotationAngle) + boxCenter - polyCenter;
    c3 = spelHelper::rotatePoint2D(c3, polyCenter, rotationAngle) + boxCenter - polyCenter;
    c4 = spelHelper::rotatePoint2D(c4, polyCenter, rotationAngle) + boxCenter - polyCenter;
    POSERECT <Point2f> poserect(c1, c2, c3, c4);
    return poserect;
  }

  //Build the rectangles for all of bodyparts
  map<int, POSERECT<Point2f>> SkeletonRects(Skeleton skeleton)
  {
    map<int, POSERECT<Point2f>> Rects;
    tree<BodyPart> PartTree = skeleton.getPartTree();
    for (tree<BodyPart>::iterator BP_iterator = PartTree.begin(); BP_iterator != PartTree.end(); BP_iterator++)
    {
      BodyJoint *j0 = skeleton.getBodyJoint(BP_iterator->getChildJoint());
      BodyJoint *j1 = skeleton.getBodyJoint(BP_iterator->getParentJoint());
      POSERECT<Point2f> Rect = BuildPartRect(j0, j1, BP_iterator->getLWRatio());
      Rects.emplace(pair<int, POSERECT<Point2f>>((*BP_iterator).getPartID(), Rect));
    }
    return Rects;
  }

  // Selecting locations of all body part from skeleton
  map<int, pair<Point2f, Point2f>> getPartLocations(Skeleton skeleton)
  {
    map<int, pair<Point2f, Point2f>> PartLocations;
    BodyJoint* J0, *J1;
    Point2f p0, p1;
    tree <BodyPart> partTree = skeleton.getPartTree();
    for (tree <BodyPart>::iterator i = partTree.begin(); i != partTree.end(); ++i)
    {
      J0 = skeleton.getBodyJoint(i->getChildJoint());
      J1 = skeleton.getBodyJoint(i->getParentJoint());
      p0 = J0->getImageLocation();
      p1 = J1->getImageLocation();
      PartLocations.emplace(pair<int, pair<Point2f, Point2f>>(i->getPartID(), pair<Point2f, Point2f>(p0, p1)));
    }
    return PartLocations;
  }

  //Returns "true" if polygon is crossed (occluded) by "rect"
  bool IsCrossed(vector<Point2f> PolygonPoints, POSERECT<Point2f> rect)
  {
    bool Crossed = 0;
    for (int i = 0; i < PolygonPoints.size() - 1; i++)
    {
      int k = i;
      if (k == PolygonPoints.size() - 1)
        k = 0;
      Point2f delta = PolygonPoints[i + 1] - PolygonPoints[k];
      float n = 2 * max(abs(delta.x), abs(delta.y));
      float dx = delta.x / n;
      float dy = delta.y / n;
      float x = PolygonPoints[i].x, y = PolygonPoints[i].y;
      for (int k = 0; k < n; k++)
      {
        Crossed = Crossed || (rect.containsPoint(Point2f(x, y)) > 0);
        y = y + dy;
        x = x + dx;
      }
    }
    return Crossed;
  }

  //Returns "true" if "rect1" is crossed (occluded) by "rect2"
  bool IsCrossed(POSERECT<Point2f> rect1, POSERECT<Point2f> rect2)
  {
    vector<Point2f> Polygon = rect1.asVector();
    bool Crossed = IsCrossed(Polygon, rect2);
    Polygon.clear();
    return Crossed;
  }

  // For calculation of polygons depth priority
  class depthCompare
  {
  public:
      bool operator () (pair<int, int> X, pair<int, int> Y)
      {
          return Y.second > X.second;
      }
  };

  //For the each polygon select all polygons, which crossed it 
  vector<vector<pair<int, int>>> CrossingsList(map<int, POSERECT<Point2f>> Rects, map<int, int> depth)
  {
    vector<vector<pair<int, int>>> Crosses;
    for (int i = 0; i < Rects.size(); i++)
    {
      vector<pair<int, int>> X;
      vector<Point2f> Polygon = Rects[i].asVector();
      for (int k = 0; k < Rects.size(); k++)
        if (depth[k] < depth[i])
        {
          if (IsCrossed(Polygon, Rects[k]))
            X.push_back(pair<int, int>(k, depth[k]));
        }
      sort(X.begin(), X.end(), depthCompare());
      Crosses.push_back(X);
      X.clear();
      Polygon.clear();
    }
    return Crosses;
  }

  //Build set of the rect pixels colours 
  vector <Point3i> GetPartColors(Mat image, Mat mask, POSERECT < Point2f > rect)
  {
    vector <Point3i> PartColors;
    float xmin, ymin, xmax, ymax;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
    for (int x = xmin; x < xmax; x++)
    {
      for (int y = ymin; y < ymax; y++)
      {
        if ((rect.containsPoint(Point2f(x, y)) > 0) && (mask.at<uint8_t>(y, x) > 9))
        {
          Vec3b color = image.at<Vec3b>(y, x);
          PartColors.push_back(Point3i(color[0], color[1], color[2]));
        }
      }
    }
    return PartColors;
  }

  //Normalization of the  histogram
  void  Normalize(vector <vector <vector <float>>> &Histogramm, int nBins, int pixelsCount)
  {
    for (int r = 0; r < nBins; r++)
      for (int g = 0; g < nBins; g++)
        for (int b = 0; b < nBins; b++)
          Histogramm[b][g][r] = Histogramm[b][g][r] / pixelsCount;
  }

  //Output histogram into text file
  void PutHistogram(ofstream &fout, vector <vector <vector <float>>> &Histogramm, int sizeFG)
  {
    int nBins = 8;
    for (int r = 0; r < nBins; r++)
      for (int g = 0; g < nBins; g++)
        for (int b = 0; b < nBins; b++)
          if (Histogramm[b][g][r]>0)
            fout << "Histogram[" << r << "," << g << "," << b << "] = " << Histogramm[b][g][r] * sizeFG << ";\n";
  }

  //Loading frames from project

  vector<Frame*> LoadTestProject(string FilePath, string FileName)
  {
#if defined(WINDOWS) && defined(_MSC_VER)
    if (IsDebuggerPresent())
      FilePath = "Debug/" + FilePath;
#endif
    ProjectLoader projectLoader(FilePath);
    projectLoader.Load(FilePath + FileName);
    vector<Frame*> frames, temp = projectLoader.getFrames();
    for (int i = 0; i < temp.size(); i++)
    {
      frames.push_back(new Frame(temp[i]->getFrametype()));
      temp[i]->clone(frames[i]);
    }
    return frames;
  }

  //Set parameters from the frames sequence
  map <string, float> SetParams(vector<Frame*> frames, Sequence **seq)
  {
    //This fragment produces crash with message: "The program has exited with code 3 (0x3)."
    map <string, float> params;
    *seq = new Sequence(0, "colorHistDetector", frames);
    if (*seq != 0)
    {
      (*seq)->estimateUniformScale(params);
      (*seq)->computeInterpolation(params);
    }
    return params;
  }

  //Counting of keyframes in set of frames 
  int keyFramesCount(vector<Frame*> frames)
  {
    int KeyframesCount = 0;
    for (int i = 0; i < frames.size(); i++)
      if (frames[i]->getFrametype() == KEYFRAME)
        KeyframesCount++;
    return KeyframesCount;
  }

  //Returns  index of first keyframe
  int FirstKeyFrameNum(vector<Frame*> frames)
  {
    int i = 0, FirstKeyframe = -1;
    while ((i < frames.size()) && (FirstKeyframe == -1))
    {
      if (frames[i]->getFrametype() == KEYFRAME)
        FirstKeyframe = i;
      i++;
    }
    return FirstKeyframe;
  }
}
#include "spelHelper.hpp"
#include "projectLoader.hpp"
#include "colorHistDetector.hpp"
#include "frame.hpp"
#include <fstream>
#include <iostream>
  
namespace SPEL
{ 
  const int NBins = 8;
  
  const uint8_t Factor = static_cast<uint8_t> (ceil(pow(2, 8) / NBins)); // Colorspace scaling coefficient Model.nBins

  map<int, POSERECT<Point2f>> SkeletonRects(Skeleton skeleton); // Build the rectangles for all of bodyparts
  POSERECT<Point2f> BuildPartRect(BodyJoint *j0, BodyJoint *j1, float LWRatio); // Build rectangle on bodypart joints
  map<int, pair<Point2f, Point2f>> getPartLocations(Skeleton skeleton); // Selecting locations of all body part from skeleton
  bool IsCrossed(vector<Point2f> PolygonPoints, POSERECT<Point2f> rect); // Returns "true" if polygon is crossed (occluded) by "rect"
  bool IsCrossed(POSERECT<Point2f> rect1, POSERECT<Point2f> rect2); // Returns "true" if "rect1" is crossed (occluded) by "rect2"
  vector<vector<pair<int, int>>> CrossingsList(map<int, POSERECT<Point2f>> Rects, map<int, int> depth); // For the each polygon select all polygons, which crossed it 
  vector <Point3i> GetPartColors(Mat image, Mat mask, POSERECT < Point2f > rect); // Build set of the rect pixels colours 
  void  Normalize(vector <vector <vector <float>>> &Histogramm, int nBins, int pixelsCount); // Normalization of the  histogram
  void PutHistogram(ofstream &fout, vector <vector <vector <float>>> &Histogramm, int sizeFG); // Output histogram into text file
  vector<Frame*> LoadTestProject(string FilePath, string FileName); // Loading frames from project
  map <string, float> SetParams(vector<Frame*> frames, Sequence **seq); // Set parameters from the frames sequence 
  int keyFramesCount(vector<Frame*> frames); // Counting of keyframes in set of frames 
  int FirstKeyFrameNum(vector<Frame*> frames); // Returns  index of first keyframe
}
#if defined(WINDOWS) && defined(_MSC_VER)
#include <Windows.h>
#endif
#include <gtest/gtest.h>
#include <detector.hpp>
#include <colorHistDetector.hpp>
#include <fstream>

using namespace cv;
using namespace std;
namespace SPEL
{
  class TestingDetector : public ColorHistDetector
  {
  public:
    Mat DeRotate(Mat imgSource, POSERECT <Point2f> &initialRect, float angle, Size size);
    float GetBoneLength(Point2f begin, Point2f end);
    float GetBoneWidth(float length, BodyPart bodyPart);
  };

  Mat TestingDetector::DeRotate(Mat imgSource, POSERECT <Point2f> &initialRect, float angle, Size size)
  {
    return  TestingDetector::rotateImageToDefault(imgSource, initialRect, angle, size);
  }

  float TestingDetector::GetBoneLength(Point2f begin, Point2f end)
  {
    return TestingDetector::getBoneLength(begin, end);
  }

  float TestingDetector::GetBoneWidth(float length, BodyPart bodyPart)
  {
    return  TestingDetector::getBoneWidth(length, bodyPart);
  }

  POSERECT<Point2f> CreateRect(float x1, float x2, float y1, float y2)
  {
    Point2f a(x1, y1), b(x2, y1), c(x2, y2), d(x1, y2), E(0, 0);
    POSERECT <Point2f> rect(a, b, c, d);
    return rect;
  }

  // Rotation of the rectangle around center 
  POSERECT<Point2f> RotateRect(POSERECT<Point2f> &rect, float angle)
  {
    POSERECT<Point2f> RotatedRect;
    Point2f center = rect.GetCenter<Point2f>();
    RotatedRect.point1 = spelHelper::rotatePoint2D(rect.point1, center, angle);
    RotatedRect.point2 = spelHelper::rotatePoint2D(rect.point2, center, angle);
    RotatedRect.point3 = spelHelper::rotatePoint2D(rect.point3, center, angle);
    RotatedRect.point4 = spelHelper::rotatePoint2D(rect.point4, center, angle);
    return RotatedRect;
  }

  // Filling the rectangle
  void FillRect(Mat &Img, POSERECT<Point2f> &rect, Vec3b colour)
  {
    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
    for (int i = (int)xmin; i <= (int)xmax; i++)
      for (int j = (int)ymin; j <= (int)ymax; j++)
        Img.at<Vec3b>(j, i) = colour;
  }

  void FillRotatedRect(Mat &Img, POSERECT<Point2f> &rect, Vec3b colour)
  {
    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
    for (int i = (int)xmin; i <= (int)xmax; i++)
      for (int j = (int)ymin; j <= (int)ymax; j++)
        if (rect.containsPoint(Point2f((float)i, (float)j)) == 1)
          Img.at<Vec3b>(j, i) = colour;
  }

  void FillRectRand(Mat &Img, POSERECT<Point2f> &rect)
  {
    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
    Vec3b colour;
    const int c = 250;
    for (int i = (int)xmin; i <= (int)xmax; i++)
      for (int j = (int)ymin; j <= (int)ymax; j++)
      {
        colour = Vec3b(uchar(rand() * c / RAND_MAX), uchar(rand() * c / RAND_MAX), uchar(rand() * c / RAND_MAX));
        Img.at<Vec3b>(j, i) = colour;
      }
  }

  // Rotation the rectangle image
  Mat RotateImage(Mat Img, POSERECT<Point2f> &rect, float angle)
  {
    Size size = Img.size();
    Mat img2 = Mat(size, CV_8UC3, Scalar(0, 0, 0));
    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
    Point2f center = rect.GetCenter<Point2f >();
    Point2f E;
    for (int i = (int)xmin; i <= (int)xmax; i++)
    {
      for (int j = (int)ymin; j <= (int)ymax; j++)
      {
        E = Point2i(i, j);
        E = spelHelper::rotatePoint2D(E, center, angle);
        if ((0 < E.x) && (E.x < size.width) && (0 < E.y) && (E.y < size.height))
          img2.at<Vec3b>((int32_t)(E.y), (int32_t)(E.x)) = Img.at<Vec3b>(j, i);
      }
    }
    return img2;
  }

  // Searching the rectangle extreme coordinates at image
  void GetExtremePoints(Mat Img, Vec3b bgColour, Point2f &Min, Point2f &Max)
  {
    Size size = Img.size();
    Min = Point2f((float)size.width + 1, (float)size.height + 1);
    Max = Point2f(-1, -1);
    for (int i = 0; i < size.width; i++)
      for (int j = 0; j < size.height; j++)
        if (Img.at<Vec3b>(j, i) != bgColour)
        {
          if (i < (int)Min.x) { Min.x = (float)i; }
          if (i > (int)Max.x) { Max.x = (float)i; }
          if (j < (int)Min.y) { Min.y = (float)j; }
          if (j > (int)Max.y) { Max.y = (float)j; }
        }
  }

  TEST(DetectorTests, rotateImageToDefault_OneColor)
  {
    // Prepare input data
    int rows = 200, cols = 200; // image size
    float angle = 45; // the rotation angle
    float x1 = 10.0, x2 = 40.0, y1 = 20.0, y2 = 60.0; // the rectangle vertices
    POSERECT <Point2f> rect = CreateRect(x1, x2, y1, y2);
    POSERECT <Point2f> RotatedRect = RotateRect(rect, angle);
    Point2f center = rect.GetCenter<Point2f >();
    Point2f center2 = RotatedRect.GetCenter<Point2f >();

    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
    Point2f MinR(xmin, ymin);
    Point2f MaxR(xmax, ymax);

    Mat img1 = Mat(Size(cols, rows), CV_8UC3, Scalar(0, 0, 0));
    Mat img2 = Mat(Size(cols, rows), CV_8UC3, Scalar(0, 0, 0));

    Vec3b colour(0, 0, 255); // color fill 

    FillRect(img1, rect, colour);
    FillRotatedRect(img2, RotatedRect, colour);

    TestingDetector chd;
    Size size((int)(xmax - xmin + 1), (int)(ymax - ymin + 1));

    // Testing
    Mat  X = chd.DeRotate(img2, RotatedRect, angle, size);

    imwrite("image.jpg", img1);
    imwrite("rotated_image.jpg", img2);
    imwrite("derotated_image.jpg", X);

    Point2f Min, Max;

    // Check X image
    GetExtremePoints(X, Vec3b(255, 255, 255), Min, Max);

    // Checking extreme points
    float delta = 1; // tolerable linear error
    EXPECT_LE(Min.x, delta);
    EXPECT_LE(abs(Max.x + xmin - MaxR.x), delta);
    EXPECT_LE(Min.y, delta);
    EXPECT_LE(abs(Max.y + ymin - MaxR.y), delta);

    // Checking matching of the images points with one fill color 
    uint32_t S = 0;
    uint32_t S0 = (uint32_t)((xmax - xmin + 1)*(ymax - ymin + 1));
    for (int i = (int)xmin; i <= (int)xmax; i++)
      for (int j = (int)ymin; j <= (int)ymax; j++)
        if (img1.at<Vec3b>(j, i) == X.at<Vec3b>((int)(j - ymin), (int)(i - xmin)))
          S++;


    float epsilon = 20; // tolerable error of the matching points number, %
    float FalsePixels = (float)(100 * (S0 - S) / S0);
    cout << "FalsePixels:\t" << FalsePixels << "\tepsilon:\t" << epsilon << endl;
    EXPECT_LE(FalsePixels, epsilon);

    img1.release();
    img2.release();
    X.release();
  }

  TEST(DetectorTests, rotateImageToDefault_RandColor)
  {
    int rows = 200, cols = 200; // image size
    float angle = 45; // the rotetion angle
    float x1 = 10.0, x2 = 40.0, y1 = 20.0, y2 = 60.0; // the rectangle vertices
    POSERECT <Point2f> rect = CreateRect(x1, x2, y1, y2);
    POSERECT <Point2f> RotatedRect = RotateRect(rect, angle);
    Point2f center = rect.GetCenter<Point2f >();
    Point2f center2 = RotatedRect.GetCenter<Point2f >();

    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
    Point2f MinR(xmin, ymin);
    Point2f MaxR(xmax, ymax);

    Mat img1 = Mat(Size(cols, rows), CV_8UC3, Scalar(0, 0, 0));
    Mat img2 = Mat(Size(cols, rows), CV_8UC3, Scalar(0, 0, 0));

    FillRectRand(img1, rect);
    img2 = RotateImage(img1, rect, angle);

    TestingDetector chd;
    Size size((int)(xmax - xmin + 1), (int)(ymax - ymin + 1));

    POSERECT <Point2f> RotatedRectCopy = RotatedRect;

    Mat  X = chd.DeRotate(img2, RotatedRect, angle, size);

    imwrite("image_RandColor.jpg", img1);
    imwrite("rotated_image_RandColor.jpg", img2);
    imwrite("derotated_image_RandColor.jpg", X);

    Point2f Min, Max;

    // Check X image
    GetExtremePoints(X, Vec3b(255, 255, 255), Min, Max);

    // Checking extreme points
    float delta = 1; // tolerable linear error
    EXPECT_LE(Min.x, delta);
    EXPECT_LE(abs(Max.x + xmin - MaxR.x), delta);
    EXPECT_LE(Min.y, delta);
    EXPECT_LE(abs(Max.y + ymin - MaxR.y), delta);

    // Checking matching of the images points with one fill color 
    uint32_t S = 0;
    uint32_t S0 = (uint32_t)((xmax - xmin + 1)*(ymax - ymin + 1));
    for (int i = (int)xmin; i <= (int)xmax; i++)
      for (int j = (int)ymin; j <= (int)ymax; j++)
        if (img1.at<Vec3b>(j, i) == X.at<Vec3b>(((int)(j - ymin), (int)(i - xmin))))
          S++;

    float epsilon = 100; // tolerable error of the matching points number, %
    float FalsePixels = (float)(100 * (S0 - S) / S0);
    cout << "FalsePixels:\t" << FalsePixels << "\tepsilon:\t" << epsilon << endl;
    EXPECT_LE(FalsePixels, epsilon);

    img1.release();
    img2.release();
    X.release();
  }

  TEST(DetectorTests, rotateImageToDefault_FileImage)
  {
#ifdef WINDOWS
#ifdef DEBUG
    Mat img1, img2, img3, img4, Q1, Q2, Q3, Q4;
    if (IsDebuggerPresent())
    {
      img1 = imread("Debug/speltests_TestData/ImageRotationTestData/image1.jpg");
      img2 = imread("Debug/speltests_TestData/ImageRotationTestData/image2.jpg");
      img3 = imread("Debug/speltests_TestData/ImageRotationTestData/image3.jpg");
      img4 = imread("Debug/speltests_TestData/ImageRotationTestData/image4.jpg");
      Q1 = imread("Debug/speltests_TestData/ImageRotationTestData/Q1.jpg");
      Q2 = imread("Debug/speltests_TestData/ImageRotationTestData/Q2.jpg");
      Q3 = imread("Debug/speltests_TestData/ImageRotationTestData/Q3.jpg");
      Q4 = imread("Debug/speltests_TestData/ImageRotationTestData/Q4.jpg");
    }
    else
    {
      img1 = imread("speltests_TestData/ImageRotationTestData/image1.jpg");
      img2 = imread("speltests_TestData/ImageRotationTestData/image2.jpg");
      img3 = imread("speltests_TestData/ImageRotationTestData/image3.jpg");
      img4 = imread("speltests_TestData/ImageRotationTestData/image4.jpg");
      Q1 = imread("speltests_TestData/ImageRotationTestData/Q1.jpg");
      Q2 = imread("speltests_TestData/ImageRotationTestData/Q2.jpg");
      Q3 = imread("speltests_TestData/ImageRotationTestData/Q3.jpg");
      Q4 = imread("speltests_TestData/ImageRotationTestData/Q4.jpg");
    }
#else
    Mat img1 = imread("Release/speltests_TestData/ImageRotationTestData/image1.jpg");
    Mat img2 = imread("Release/speltests_TestData/ImageRotationTestData/image2.jpg");
    Mat img3 = imread("Release/speltests_TestData/ImageRotationTestData/image3.jpg");
    Mat img4 = imread("Release/speltests_TestData/ImageRotationTestData/image4.jpg");
    Mat Q1 = imread("Release/speltests_TestData/ImageRotationTestData/Q1.jpg");
    Mat Q2 = imread("Release/speltests_TestData/ImageRotationTestData/Q2.jpg");
    Mat Q3 = imread("Release/speltests_TestData/ImageRotationTestData/Q3.jpg");
    Mat Q4 = imread("Release/speltests_TestData/ImageRotationTestData/Q4.jpg");
#endif  // DEBUG
#else
    Mat img1 = imread("speltests_TestData/ImageRotationTestData/image1.jpg");
    Mat img2 = imread("speltests_TestData/ImageRotationTestData/image2.jpg");
    Mat img3 = imread("speltests_TestData/ImageRotationTestData/image3.jpg");
    Mat img4 = imread("speltests_TestData/ImageRotationTestData/image4.jpg");
    Mat Q1 = imread("speltests_TestData/ImageRotationTestData/Q1.jpg");
    Mat Q2 = imread("speltests_TestData/ImageRotationTestData/Q2.jpg");
    Mat Q3 = imread("speltests_TestData/ImageRotationTestData/Q3.jpg");
    Mat Q4 = imread("speltests_TestData/ImageRotationTestData/Q4.jpg");
#endif  // WINDOWS
    imwrite("Q01.jpg", Q1);
    imwrite("Q02.jpg", Q2);
    imwrite("Q03.jpg", Q3);
    imwrite("Q04.jpg", Q4);

    // Prepare input data
    Size img_size = img1.size();

    float angle = 45; // the rotetion angle
    float x1 = 10.0, x2 = 39.0, y1 = 20.0, y2 = 59.0; // the rectangle vertices
    POSERECT <Point2f> rect = CreateRect(x1, x2, y1, y2);
    POSERECT <Point2f> RotatedRect = RotateRect(rect, angle);
    Point2f center = rect.GetCenter<Point2f >();

    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
    Point2f MinR(xmin, ymin);
    Point2f MaxR(xmax, ymax);

    TestingDetector chd;
    Size size((int)(xmax - xmin + 1), (int)(ymax - ymin + 1));

    // Testing

    Mat  X1 = chd.DeRotate(Q1, RotatedRect, angle, size);
    Mat  X2 = chd.DeRotate(Q2, RotatedRect, angle, size);
    Mat  X3 = chd.DeRotate(Q3, RotatedRect, angle, size);
    Mat  X4 = chd.DeRotate(Q4, RotatedRect, angle, size);

    imwrite("derotated_image1.jpg", X1);
    imwrite("derotated_image2.jpg", X2);
    imwrite("derotated_image3.jpg", X3);
    imwrite("derotated_image4.jpg", X4);

    Point2f Min1, Max1;
    Point2f Min2, Max2;
    Point2f Min3, Max3;
    Point2f Min4, Max4;

    // Check X image
    GetExtremePoints(X1, Vec3b(255, 255, 255), Min1, Max1);
    GetExtremePoints(X2, Vec3b(255, 255, 255), Min2, Max2);
    GetExtremePoints(X3, Vec3b(255, 255, 255), Min3, Max3);
    GetExtremePoints(X4, Vec3b(255, 255, 255), Min4, Max4);

    // Checking extreme points
    float delta = 1; // tolerable linear error
    EXPECT_LE(Min1.x, delta);
    EXPECT_LE(abs(Max1.x + xmin - MaxR.x), delta);
    EXPECT_LE(Min1.y, delta);
    EXPECT_LE(abs(Max1.y + ymin - MaxR.y), delta);

    EXPECT_LE(Min2.x, delta);
    EXPECT_LE(abs(Max2.x + xmin - MaxR.x), delta);
    EXPECT_LE(Min2.y, delta);
    EXPECT_LE(abs(Max2.y + ymin - MaxR.y), delta);

    // Checking matching of the images points with one fill color 
    float epsilon = 75; // tolerable error of the matching points number, %

    uint32_t S = 0;
    uint32_t S0 = (uint32_t)((xmax - xmin + 1)*(ymax - ymin + 1));

    for (int i = (int)xmin; i <= (int)xmax; i++)
      for (int j = (int)ymin; j <= (int)ymax; j++)
        if (img1.at<Vec3b>(j, i) == X1.at<Vec3b>((int)(j - ymin), (int)(i - xmin)))
          S++;
    float FalsePixels = (float)(100 * (S0 - S) / S0);
    cout << "FalsePixels:\t" << FalsePixels << "\tepsilon:\t" << epsilon << endl;
    EXPECT_LE(FalsePixels, epsilon);

    S = 0;
    for (int i = (int)xmin; i <= (int)xmax; i++)
      for (int j = (int)ymin; j <= (int)ymax; j++)
        if (img2.at<Vec3b>(j, i) == X2.at<Vec3b>((int)(j - ymin), (int)(i - xmin)))
          S++;
    FalsePixels = (float)(100 * (S0 - S) / S0);
    cout << "FalsePixels:\t" << FalsePixels << "\tepsilon:\t" << epsilon << endl;
    EXPECT_LE(FalsePixels, epsilon);

    S = 0;
    for (int i = (int)xmin; i <= (int)xmax; i++)
      for (int j = (int)ymin; j <= (int)ymax; j++)
        if (img3.at<Vec3b>(j, i) == X3.at<Vec3b>((int)(j - ymin), (int)(i - xmin)))
          S++;
    FalsePixels = (float)(100 * (S0 - S) / S0);
    cout << "FalsePixels:\t" << FalsePixels << "\tepsilon:\t" << epsilon << endl;
    EXPECT_LE(FalsePixels, epsilon);

    S = 0;
    for (int i = (int)xmin; i <= (int)xmax; i++)
      for (int j = (int)ymin; j <= (int)ymax; j++)
        if (img4.at<Vec3b>(j, i) == X4.at<Vec3b>((int)(j - ymin), (int)(i - xmin)))
          S++;
    FalsePixels = (float)(100 * (S0 - S) / S0);
    cout << "FalsePixels:\t" << FalsePixels << "\tepsilon:\t" << epsilon << endl;
    EXPECT_LE(FalsePixels, epsilon);

    img1.release();
    img2.release();
    img3.release();
    img4.release();

    Q1.release();
    Q2.release();
    Q3.release();
    Q4.release();

    X1.release();
    X2.release();
    X3.release();
    X4.release();
  }

  TEST(DetectorTests, getBoneLength)
  {
    const int nBins = 8;
    TestingDetector detector;
    Point2f begin(0, 0), end(1, 0);

    float length = detector.GetBoneLength(begin, end);
    cout << length;
    EXPECT_EQ(end.x - begin.x, length);
  }

  TEST(DetectorTests, getBoneWidth)
  {
    const int nBins = 8;
    TestingDetector detector;
    BodyPart part;
    float lwRatio = 2.0;
    part.setLWRatio(lwRatio);
    float length = 1;

    float width = detector.GetBoneWidth(length, part);
    cout << length;
    EXPECT_EQ(length / lwRatio, width);
  }

  //Output limbLabels set into text file
  void PutLimbLabels(ofstream &fout, string S, map <uint32_t, vector <LimbLabel>> X)
  {
    fout << S << endl;
    for (int p = 0; p < X.size(); p++)
    {
      for (int i = 0; i < X[p].size(); i++)
      {
        fout << endl << "  limbID " << X[p][i].getLimbID() << ", angle " << X[p][i].getAngle() << ", poligon" << X[p][i].getPolygon() << ", scores ";
        vector<Score> S = X[p][i].getScores();
        for (int k = 0; k < S.size(); k++)
          fout << S[k].getScore() << ", ";
      }
      fout << endl;
    }
    fout << endl << "===========================================" << endl;
  }

  /*
  //Temporary test. Data loss is not checked - only repeating
  //The test for universal function "merge"
  TEST(DetectorTests, merge)
  {
  vector <vector <LimbLabel>> A;
  vector <vector <LimbLabel>> B;

  //Create polygons
  int polygonsCount = 10;
  vector<vector<Point2f>> Polygons;
  for (int i = 0; i < polygonsCount; i++)
  Polygons.push_back(vector < Point2f > { Point2f(i, i) });

  //Craeate set of limbLabels "A"
  int partsCount = 2;
  int A_size = 9;
  for (int p = 0; p < partsCount; p++)
  {
    vector<LimbLabel> temp_partLabels;
    for (int i = 0; i < A_size; i++)
    {
      int N = (Polygons.size() - 1)*rand() / RAND_MAX;
      LimbLabel temp_label(p, Point2f(N, N), N, Polygons[N], vector < Score > { Score(float((rand()) * 100 / RAND_MAX) / 100, "", 1)});
      temp_partLabels.push_back(temp_label);
    }
    A.push_back(temp_partLabels);
  }

  //Craeate set of limbLabels "B"
  int B_size = 9;
  for (int p = 0; p < partsCount; p++)
  {
    vector<LimbLabel> temp_partLabels;
    for (int i = 0; i < B_size; i++)
    {
      int N = (Polygons.size() - 1)*rand() / RAND_MAX;
      LimbLabel temp_label(p, Point2f(N, N), N, Polygons[N], vector < Score > { Score(float((rand()) * 100 / RAND_MAX) / 100, "", 1)});
      temp_partLabels.push_back(temp_label);
    }
    B.push_back(temp_partLabels);
  }

  //Run "merge"
  ColorHistDetector D;
  vector <vector <LimbLabel>> H = D.merge(A, B);
  vector <vector <LimbLabel>> A1 = D.merge(A, A);
  vector <vector <LimbLabel>> B1 = D.merge(B, B);
  vector <vector <LimbLabel>> C = D.merge(A1, B1);

  //Put all input and output labels sets into text file
  ofstream fout("Detector_merge.txt");
  PutLimbLabels(fout,"LimbLabels set A:", A);
  PutLimbLabels(fout,"LimbLabels set B:", B);
  PutLimbLabels(fout, "merge(A, B):", H);
  PutLimbLabels(fout, "merge(A, A):", A1);
  PutLimbLabels(fout,"merge(B, B):", B1);
  PutLimbLabels(fout, "merge(merge(A, A), merge(B, B)) :", C);
  fout.close();

  cout << "See input abd output limbLabels values in the file  'Detector_merge.txt'\n";

  //Compare
  for (int p = 0; p < C.size(); p++)
    for (int i = 0; i < C[p].size(); i++)
    {
      //Search equal polygon values in curent part labels
      for (int k = 0; k < C[p].size(); k++)
        if (i != k) EXPECT_FALSE(C[p][i].getPolygon() == C[p][k].getPolygon()) << "partID = " << p << ": polygons of label_Num " << i << " and " << k << "is equal" << endl;
      vector<Score> LimbScores = C[p][i].getScores();
      //Search equal scores in curent label
      for (int k = 0; k < LimbScores.size(); k++)
        for (int t = 0; t < LimbScores.size(); t++)
      if (t != k) EXPECT_FALSE(LimbScores[k] == LimbScores[t]) << ", PartID = " << p << ": Equal scores in label with Num = " << i << endl;
    }
  }
  */

  TEST(DetectorTests, merge)
  {
    map <uint32_t, vector <LimbLabel>> A;
    map <uint32_t, vector <LimbLabel>> B;

    //Create polygons
    int polygonsCount = 10;
    vector<vector<Point2f>> Polygons;
    for (int i = 0; i < polygonsCount; i++)
      Polygons.push_back(vector < Point2f > { Point2f(i, i) });

    //Craeate sets of limbLabels "A" and ""B
    int partsCount = 2;
    int A_size = 9;
    for (int p = 0; p < partsCount; p++)
    {
      vector<LimbLabel> tempA_partLabels;
      vector<LimbLabel> tempB_partLabels;
      for (int N = 0; N < polygonsCount; N++)
      {
        LimbLabel temp_label(p, Point2f(N, N), N, Polygons[N], vector < Score > { Score(N / (float)polygonsCount, "", 1)});
        if (rand() > 0.3*RAND_MAX)
          tempA_partLabels.push_back(temp_label);
        if (rand() > 0.3*RAND_MAX)
          tempB_partLabels.push_back(temp_label);
      }
      A.insert(pair<uint32_t, vector <LimbLabel>>(p, tempA_partLabels));
      B.insert(pair<uint32_t, vector <LimbLabel>>(p, tempB_partLabels));
      tempA_partLabels.clear();
      tempB_partLabels.clear();
    }

    //Run "merge"
    ColorHistDetector D;
    auto H = D.merge(A, B, map<uint32_t, vector<LimbLabel>>());
    auto A1 = D.merge(A, A, map<uint32_t, vector<LimbLabel>>());
    auto B1 = D.merge(B, B, map<uint32_t, vector<LimbLabel>>());
    auto C = D.merge(A1, B1, map<uint32_t, vector<LimbLabel>>());

    //Put all input and output labels sets into text file
    ofstream fout("Detector_merge.txt");
    PutLimbLabels(fout, "LimbLabels set A:", A);
    PutLimbLabels(fout, "LimbLabels set B:", B);
    PutLimbLabels(fout, "merge(A, B):", H);
    PutLimbLabels(fout, "merge(A, A):", A1);
    PutLimbLabels(fout, "merge(B, B):", B1);
    PutLimbLabels(fout, "merge(merge(A, A), merge(B, B)) :", C);
    fout.close();

    cout << "See input abd output limbLabels values in the file  'Detector_merge.txt'\n";

    //Compare
    for (int p = 0; p < C.size(); p++)
      for (int i = 0; i < C[p].size(); i++)
      {
        //Search equal polygon values in curent part labels
        for (int k = 0; k < C[p].size(); k++)
          if (i != k) EXPECT_FALSE(C[p][i].getPolygon() == C[p][k].getPolygon()) << "partID = " << p << ": polygons of label_Num " << i << " and " << k << "is equal" << endl;
        vector<Score> LimbScores = C[p][i].getScores();
        //Search equal scores in curent label
        for (int k = 0; k < LimbScores.size(); k++)
          for (int t = 0; t < LimbScores.size(); t++)
            if (t != k) EXPECT_FALSE(LimbScores[k] == LimbScores[t]) << ", PartID = " << p << ": Equal scores in label with Num = " << i << endl;
      }
  }

  // Unit test for 'merge' function with statically generated data.
  // This unit test checks only scores
  TEST(DetectorTests, merge_StaticData)
  {
    map <uint32_t, vector <LimbLabel>> a;

    //Create polygons
    const int polygonsCount = 10;
    vector<vector<Point2f>> Polygons;
    for (int i = 0; i < polygonsCount; i++)
      Polygons.push_back(vector < Point2f > { Point2f(i, i) });

    const int partCount = 2;
    const int a_size = 10;

    for (int p = 0; p < partCount; p++)
    {
      vector<LimbLabel> temp_partLabels;
      for (int i = 0; i < a_size; i++)
      {
        LimbLabel temp_label(p, Point2f(i, i), i, Polygons[i], vector < Score > { Score(i / (float)10, "a", 1)});
        temp_partLabels.push_back(temp_label);
      }
      a.insert(pair<uint32_t, vector <LimbLabel>>(p, temp_partLabels));
    }
    ColorHistDetector d;
    auto b = d.merge(a, a, map<uint32_t, vector<LimbLabel>>());

    EXPECT_EQ(a.size(), b.size());

    for (int p = 0; p < a.size(); p++)
    {
      vector <LimbLabel> temp = b[p];
      EXPECT_EQ(temp.size(), a_size);
      for (int i = 0; i < temp.size(); i++)
      {
        LimbLabel l = temp[i];
        vector <Score> scores = l.getScores();
        EXPECT_EQ(scores.size(), 1);
        EXPECT_EQ(scores[0].getScore(), a[p][i].getScores().at(0).getScore());
        EXPECT_EQ(scores[0].getDetName(), a[p][i].getScores().at(0).getDetName());
      }
    }

    map <uint32_t, vector <LimbLabel>> c;
    c = d.merge(b, c, map<uint32_t, vector<LimbLabel>>());

    for (int p = 0; p < a.size(); p++)
    {
      vector <LimbLabel> temp = c[p];
      EXPECT_EQ(temp.size(), a_size);
      for (int i = 0; i < temp.size(); i++)
      {
        LimbLabel l = temp[i];
        vector <Score> scores = l.getScores();
        EXPECT_EQ(scores.size(), 1);
        EXPECT_EQ(scores[0].getScore(), a[p][i].getScores().at(0).getScore());
        EXPECT_EQ(scores[0].getDetName(), a[p][i].getScores().at(0).getDetName());
      }
    }
    map <uint32_t, vector <LimbLabel>> g;
    g = d.merge(g, c, map<uint32_t, vector<LimbLabel>>());

    for (int p = 0; p < a.size(); p++)
    {
      vector <LimbLabel> temp = g[p];
      EXPECT_EQ(temp.size(), a_size);
      for (int i = 0; i < temp.size(); i++)
      {
        LimbLabel l = temp[i];
        vector <Score> scores = l.getScores();
        EXPECT_EQ(scores.size(), 1);
        EXPECT_EQ(scores[0].getScore(), a[p][i].getScores().at(0).getScore());
        EXPECT_EQ(scores[0].getDetName(), a[p][i].getScores().at(0).getDetName());
      }
    }

    map <uint32_t, vector <LimbLabel>> e;

    for (int p = 0; p < partCount; p++)
    {
      vector<LimbLabel> temp_partLabels;
      for (int i = 0; i < a_size; i++)
      {
        LimbLabel temp_label(p, Point2f(i, i), i, Polygons[i], vector < Score > { Score(i / (float)10, "e", 1)});
        temp_partLabels.push_back(temp_label);
      }
      e.insert(pair<uint32_t, vector <LimbLabel>>(p, temp_partLabels));
    }

    auto f = d.merge(c, e, map<uint32_t, vector<LimbLabel>>());

    for (int p = 0; p < a.size(); p++)
    {
      vector <LimbLabel> temp = f[p];
      EXPECT_EQ(temp.size(), a_size);
      for (int i = 0; i < temp.size(); i++)
      {
        LimbLabel l = temp[i];
        vector <Score> scores = l.getScores();
        EXPECT_EQ(scores.size(), 2);
        EXPECT_EQ(scores[0].getScore(), c[p][i].getScores().at(0).getScore());
        EXPECT_EQ(scores[0].getDetName(), c[p][i].getScores().at(0).getDetName());
        EXPECT_EQ(scores[1].getScore(), e[p][i].getScores().at(0).getScore());
        EXPECT_EQ(scores[1].getDetName(), e[p][i].getScores().at(0).getDetName());
      }
    }

    map <uint32_t, vector <LimbLabel>> h;
    map <uint32_t, vector <LimbLabel>> m;

    for (int p = 0; p < partCount; p++)
    {
      vector<LimbLabel> temp_partLabels;
      for (int i = 0; i < a_size / 2; i++)
      {
        LimbLabel temp_label(p, Point2f(i, i), i, Polygons[i], vector < Score > { Score(i / (float)10, "h", 1)});
        temp_partLabels.push_back(temp_label);
      }
      h.insert(pair <uint32_t, vector <LimbLabel>>(p, temp_partLabels));
    }

    for (int p = 0; p < partCount; p++)
    {
      vector<LimbLabel> temp_partLabels;
      for (int i = a_size / 2; i < a_size; i++)
      {
        LimbLabel temp_label(p, Point2f(i, i), i, Polygons[i], vector < Score > { Score(i / (float)10, "m", 1)});
        temp_partLabels.push_back(temp_label);
      }
      m.insert(pair <uint32_t, vector <LimbLabel>>(p, temp_partLabels));
    }

    auto n = d.merge(h, m, map<uint32_t, vector<LimbLabel>>());

    for (int p = 0; p < a.size(); p++)
    {
      vector <LimbLabel> temp = n[p];
      EXPECT_EQ(temp.size(), a_size);
      for (int i = 0; i < temp.size() / 2; i++)
      {
        LimbLabel l = temp[i];
        vector <Score> scores = l.getScores();
        EXPECT_EQ(scores.size(), 2);
        bool bFound = false;
        LimbLabel o;
        for (int k = 0; k < h[p].size(); k++)
        {
          if (l.getPolygon() == h[p][k].getPolygon())
          {
            bFound = true;
            o = h[p][k];
            break;
          }
        }
        EXPECT_TRUE(bFound);
        if (bFound)
        {
          EXPECT_EQ(scores[0].getScore(), o.getScores().at(0).getScore());
          EXPECT_EQ(scores[0].getDetName(), o.getScores().at(0).getDetName());
          EXPECT_EQ(scores[1].getScore(), 1.0);
          EXPECT_EQ(scores[1].getDetName(), m[p][0].getScores().at(0).getDetName());
        }
      }

      for (int i = temp.size() / 2; i < temp.size(); i++)
      {
        LimbLabel l = temp[i];
        vector <Score> scores = l.getScores();
        EXPECT_EQ(scores.size(), 2);
        bool bFound = false;
        LimbLabel q;
        for (int k = 0; k < m[p].size(); k++)
        {
          if (l.getPolygon() == m[p][k].getPolygon())
          {
            bFound = true;
            q = m[p][k];
            break;
          }
        }
        EXPECT_TRUE(bFound);
        if (bFound)
        {
          EXPECT_EQ(scores[0].getScore(), q.getScores().at(0).getScore());
          EXPECT_EQ(scores[0].getDetName(), q.getScores().at(0).getDetName());
          EXPECT_EQ(scores[1].getScore(), 1.0);
          EXPECT_EQ(scores[1].getDetName(), h[p][0].getScores().at(0).getDetName());
        }
      }
    }

  }

}

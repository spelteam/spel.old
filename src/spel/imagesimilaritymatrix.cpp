#include "imagesimilaritymatrix.hpp"

namespace SPEL
{
  ImageSimilarityMatrix::ImageSimilarityMatrix(void)
  {
    //nothing to do
  }
  ImageSimilarityMatrix::~ImageSimilarityMatrix(void)
  {
    imageSimilarityMatrix.release();
    imageShiftMatrix.release();
  }

  ImageSimilarityMatrix::ImageSimilarityMatrix(const vector<Frame*>& frames)
  {
    //by default use colour
    buildImageSimilarityMatrix(frames);
  }

  ImageSimilarityMatrix::ImageSimilarityMatrix(const ImageSimilarityMatrix& m)
  {
    //by default use colour
    imageSimilarityMatrix = m.imageSimilarityMatrix;
    imageShiftMatrix = m.imageShiftMatrix;
  }

  //get ISM value at (row, col)
  float ImageSimilarityMatrix::at(int row, int col) const
  {

    if (row >= imageSimilarityMatrix.rows)
    {
      cerr << "ISM contains " << imageSimilarityMatrix.rows << " rows, cannot request row " << endl; //<< to_string(row) << end;
      return -1;
    }
    if (col >= imageSimilarityMatrix.cols)
    {
      cerr << "ISM contains " << imageSimilarityMatrix.cols << " cols, cannot request col " << endl; // << to_string(col) << end;
      return -1;
    }
    return imageSimilarityMatrix.at<float>(row, col);
  }

  //get ISM value at (row, col)
  Point2f ImageSimilarityMatrix::getShift(int row, int col) const
  {

    if (row >= imageShiftMatrix.rows)
    {
      cerr << "Shift Matrix contains " << imageShiftMatrix.rows << " rows, cannot request row " << endl; //<< to_string(row) << end;
      return Point2f();
    }
    if (col >= imageShiftMatrix.cols)
    {
      cerr << "Shift Matrix contains " << imageShiftMatrix.cols << " cols, cannot request col " << endl; // << to_string(col) << end;
      return Point2f();
    }
    return imageShiftMatrix.at<Point2f>(row, col);
  }

  bool ImageSimilarityMatrix::operator==(const ImageSimilarityMatrix &s) const
  {
    Mat result = (imageSimilarityMatrix == s.imageSimilarityMatrix);

    bool res = true;

    //if every element is 1
    for (int i = 0; i < result.rows; ++i)
    {
      for (int j = 0; j < result.cols; ++j)
      {
        if (result.at<float>(i, j) == 0)
          res = false;
      }
    }
    return res;
  }

  bool ImageSimilarityMatrix::operator!=(const ImageSimilarityMatrix &s) const
  {
    Mat result = (imageSimilarityMatrix == s.imageSimilarityMatrix);

    bool res = true;

    //if every element is 1
    for (int i = 0; i < result.rows; ++i)
    {
      for (int j = 0; j < result.cols; ++j)
      {
        if (result.at<float>(i, j) == 0)
          res = false;
      }
    }
    return !res;
  }

  ImageSimilarityMatrix& ImageSimilarityMatrix::operator=(const ImageSimilarityMatrix &s)
  {
    imageSimilarityMatrix = s.imageSimilarityMatrix;
    imageShiftMatrix = s.imageShiftMatrix;
    return *this;
  }

  bool ImageSimilarityMatrix::read(string filename)
  {
    ifstream in(filename.c_str());
    if (in.is_open())
    {
      int size;
      in >> size;

      imageSimilarityMatrix.release();
      imageShiftMatrix.release();

      imageSimilarityMatrix.create(size, size, DataType<float>::type);
      imageShiftMatrix.create(size, size, DataType<Point2f>::type);

      for (int i = 0; i < imageSimilarityMatrix.rows; ++i)
      {
        for (int j = 0; j < imageSimilarityMatrix.cols; ++j)
        {
          float score;
          in >> score;
          imageSimilarityMatrix.at<float>(i, j) = score;
        }
      }

      //@FIX fix this if it is imported

      for (int i = 0; i < imageShiftMatrix.rows; ++i)
      {
        for (int j = 0; j < imageShiftMatrix.cols; ++j)
        {
          float x, y;
          in >> x >> y;
          imageShiftMatrix.at<Point2f>(i, j) = Point2f(x, y);
        }
      }
      return true;
    }
    else
    {
      cerr << "Could not open " << filename << " for reading. " << endl;
      return false;
    }
  }

  bool ImageSimilarityMatrix::write(string filename) const
  {
    ofstream out(filename.c_str());
    if (out.is_open())
    {
      out << imageSimilarityMatrix.rows << endl; //size
      for (int i = 0; i < imageSimilarityMatrix.rows; ++i)
      {
        for (int j = 0; j < imageSimilarityMatrix.cols; ++j)
        {
          out << imageSimilarityMatrix.at<float>(i, j) << " ";
        }
        out << endl;
      }

      for (int i = 0; i < imageShiftMatrix.rows; ++i)
      {
        for (int j = 0; j < imageShiftMatrix.cols; ++j)
        {
          out << imageShiftMatrix.at<Point2f>(i, j).x << " " << imageShiftMatrix.at<Point2f>(i, j).y << " ";
        }
        out << endl;
      }
      return true;
    }
    else
    {
      cerr << "Could not open " << filename << " for writing. " << endl;
      return false;
    }
  }

  void ImageSimilarityMatrix::computeISMcell(Frame* left, Frame* right, int maxFrameHeight)
  {
    int i = left->getID(), j = right->getID();
    //only do this loop if
    if (j > i)
      return;
    if (i == j)
    {
      imageShiftMatrix.at<Point2f>(i, j) = Point2f(0, 0);
      imageSimilarityMatrix.at<float>(i, j) = 0;
      return;
    }
    //load images, compute similarity, store to matrix
    Mat imgMatOne = left->getImage();
    Mat imgMatTwo = right->getImage();

    Mat maskMatOne = left->getMask();
    Mat maskMatTwo = right->getMask();

//    float factor = 1;
//    //compute the scaling factor
//    if (maxFrameHeight != 0)
//    {
//      factor = (float)maxFrameHeight / (float)imgMatOne.rows;

//      resize(imgMatOne, imgMatOne, cvSize(imgMatOne.cols * factor, imgMatOne.rows * factor));
//      resize(imgMatTwo, imgMatTwo, cvSize(imgMatTwo.cols * factor, imgMatTwo.rows * factor));

//      resize(maskMatOne, maskMatOne, cvSize(maskMatOne.cols * factor, maskMatOne.rows * factor));
//      resize(maskMatTwo, maskMatTwo, cvSize(maskMatTwo.cols * factor, maskMatTwo.rows * factor));
//    }

    Point2f cOne, cTwo;
    float mSizeOne = 0, mSizeTwo = 0;

    // cout << "at " << i << ", " << j << " ";

    Point2f dX;
    for (int x = 0; x < maskMatOne.rows; ++x)
    {
      for (int y = 0; y < maskMatOne.cols; ++y)
      {
        Scalar intensity = maskMatOne.at<uchar>(x, y);
        Scalar mintensity = maskMatTwo.at<uchar>(x, y);

        bool darkPixel = intensity.val[0] < 10;
        bool blackPixel = mintensity.val[0] < 10; //if all intensities are zero

        if (!darkPixel) //if the pixel is non-black for maskOne
        {
          //pixOne = imgOne.pixel(x,y);
          cOne += Point2f(x, y);
          mSizeOne++;
        }

        if (!blackPixel) //if the pixel is non-black for maskOne
        {
          //pixTwo = imgTwo.pixel(x,y);
          cTwo += Point2f(x, y);
          mSizeTwo++;
        }

      }
    }
    cOne = cOne*(1.0 / mSizeOne);
    cTwo = cTwo*(1.0 / mSizeTwo);

    //cOne and cTwo now have the centres
    dX = cTwo - cOne;
    //dX = dX*pow(factor, -1.0);

    //so, dX+cOne = cTwo
    //and cOne = cTwo-dX

    //for real image coords, these points are actually reversed
    imageShiftMatrix.at<Point2f>(i, j) = Point2f(dX.y, dX.x);
    imageShiftMatrix.at<Point2f>(j, i) = Point2f(-dX.y, -dX.x);

    float similarityScore = 0;
    // float maskSimilarityScore = 0;
    for (int x = 0; x < maskMatOne.rows; ++x)
    {
      for (int y = 0; y < maskMatOne.cols; ++y)
      {
        int mintensityOne = maskMatOne.at<uchar>(x, y);

        bool darkPixel = mintensityOne < 10; //if all intensities are zero

        int mOne = 0, mTwo = 0;

        //apply the transformation
        int xTwo = x + dX.x;
        int yTwo = y + dX.y;

        //now check bounds

        int blueOne = 0;
        int greenOne = 0;
        int redOne = 0;

        int blueTwo = 255;
        int greenTwo = 255;
        int redTwo = 255;

        //compare points
        if (!darkPixel)
        {
          mOne = 1;
          Vec4b intensityOne = imgMatOne.at<Vec4b>(x, y);
          blueOne = intensityOne.val[0];
          greenOne = intensityOne.val[1];
          redOne = intensityOne.val[2];
        }
        if (xTwo < imgMatTwo.rows && xTwo >= 0 && yTwo < imgMatTwo.cols && yTwo >= 0)
        {
          Scalar mintensityTwo = maskMatTwo.at<uchar>(xTwo, yTwo);
          bool blackPixel = mintensityTwo.val[0] < 10; //if all intensities are zero

          if (!blackPixel)
          {
            mTwo = 1;
            Vec4b intensityTwo = imgMatTwo.at<Vec4b>(xTwo, yTwo);
            blueTwo = intensityTwo.val[0];
            greenTwo = intensityTwo.val[1];
            redTwo = intensityTwo.val[2];
          }
        }

        // maskSimilarityScore+=abs(mOne-mTwo);

        if (mOne^mTwo) //maximum penalty if they are different
        {
          similarityScore += pow(255, 2) + pow(255, 2) + pow(255, 2); //square of absolute difference
        }
        else //if both are in mask, or outside of mask
        {
          similarityScore += pow(redOne - redTwo, 2) + pow(greenOne - greenTwo, 2) +
            pow(blueOne - blueTwo, 2); //square of absolute difference
        }
      }
    }

    // cout << " score = " << similarityScore << endl;

    imageSimilarityMatrix.at<float>(i, j) = similarityScore;
    imageSimilarityMatrix.at<float>(j, i) = similarityScore;

    return;
  }

  void ImageSimilarityMatrix::computeMSMcell(Frame* left, Frame* right, int maxFrameHeight)
  {
    int i = left->getID(), j = right->getID();
    //only do this loop if
    if (j > i)
      return;
    if (i == j)
    {
      imageShiftMatrix.at<Point2f>(i, j) = Point2f(0, 0);
      imageSimilarityMatrix.at<float>(i, j) = 0;
      return;
    }

    Mat maskMatOne = left->getMask();
    Mat maskMatTwo = right->getMask();

//    float factor = 1;
//    //compute the scaling factor
//    if (maxFrameHeight != 0)
//    {
//      factor = (float)maxFrameHeight / (float)maskMatOne.rows;

//      resize(maskMatOne, maskMatOne, cvSize(maskMatOne.cols * factor, maskMatOne.rows * factor));
//      resize(maskMatTwo, maskMatTwo, cvSize(maskMatTwo.cols * factor, maskMatTwo.rows * factor));
//    }


    Point2f cOne, cTwo;
    float mSizeOne = 0, mSizeTwo = 0;

    // cout << "at " << to_string(i) << ", " << to_string(j) << " ";
    //compute similarity score
    //            if(i==j)
    //                cout<<"SPECIAL CASE" << endl;

    //compute deltaX from centroid
    Point2f dX;
    for (int x = 0; x < maskMatOne.rows; ++x)
    {
      for (int y = 0; y < maskMatOne.cols; ++y)
      {
        int intensity = maskMatOne.at<uchar>(y, x);
        int mintensity = maskMatTwo.at<uchar>(y, x);

        bool darkPixel = intensity < 10;
        bool blackPixel = mintensity < 10; //if all intensities are zero

        if (!darkPixel) //if the pixel is non-black for maskOne
        {
          //pixOne = imgOne.pixel(x,y);
          cOne += Point2f(x, y);
          mSizeOne++;
        }

        if (!blackPixel) //if the pixel is non-black for maskOne
        {
          //pixTwo = imgTwo.pixel(x,y);
          cTwo += Point2f(x, y);
          mSizeTwo++;
        }
      }
    }
    cOne = cOne*(1.0 / mSizeOne);
    cTwo = cTwo*(1.0 / mSizeTwo);

    //cOne and cTwo now have the centres
    dX = cTwo - cOne;
    //dX = dX*pow(factor, -1.0);

    imageShiftMatrix.at<Point2f>(i, j) = Point2f(dX.y, dX.x);
    imageShiftMatrix.at<Point2f>(j, i) = Point2f(-dX.y, -dX.x);

    //so, dX+cOne = cTwo
    //and cOne = cTwo-dX

    // float similarityScore = 0;
    float maskSimilarityScore = 0;
    for (int x = 0; x < maskMatOne.rows; ++x)
    {
      for (int y = 0; y < maskMatOne.cols; ++y)
      {
        int mintensityOne = maskMatOne.at<uchar>(j, i);

        bool darkPixel = mintensityOne < 10; //if all intensities are zero

        //QColor pixOne;
        //QColor pixTwo;
        int mOne = 0, mTwo = 0;

        //apply the transformation
        int xTwo = x + dX.x;
        int yTwo = y + dX.y;

        int mintensityTwo = maskMatTwo.at<uchar>(yTwo, xTwo);

        bool blackPixel = mintensityTwo < 10; //if all intensities are zero

        //compare points
        if (!darkPixel)
        {
          mOne = 1;
        }
        if (xTwo < maskMatTwo.rows && xTwo >= 0 && yTwo < maskMatTwo.cols && yTwo >= 0 && !blackPixel)
        {
          mTwo = 1;
        }

        maskSimilarityScore += abs(mOne - mTwo);
      }
    }

    imageSimilarityMatrix.at<float>(i, j) = maskSimilarityScore;
    imageSimilarityMatrix.at<float>(j, i) = maskSimilarityScore;

    return;
  }

  void ImageSimilarityMatrix::buildMaskSimilarityMatrix(const vector<Frame*>& frames, int maxFrameHeight)
  {
    //create matrices and fill with zeros
    // imageSimilarityMatrix.create(frames.size(), frames.size(), DataType<float>::type);
    imageSimilarityMatrix.release();
    imageShiftMatrix.release();

    imageSimilarityMatrix.create(frames.size(), frames.size(), DataType<float>::type);
    imageShiftMatrix.create(frames.size(), frames.size(), DataType<Point2f>::type);

    for (uint32_t i = 0; i < frames.size(); ++i)
    {
      for (uint32_t j = 0; j < frames.size(); ++j)
      {
        // imageSimilarityMatrix.at<float>(i,j) = 0;
        imageSimilarityMatrix.at<float>(i, j) = 0;
      }
    }
    //store the futures
    vector<future<void> > futures;

    //set-up finished

    //compute mask centroid offsets
    for (uint32_t i = 0; i < frames.size(); ++i)
    {
      for (uint32_t j = 0; j < frames.size(); ++j)
      {
        Frame* left = frames[i];
        Frame* right = frames[j];
        futures.push_back(std::async(&ImageSimilarityMatrix::computeMSMcell, this, left, right, maxFrameHeight));
      }
    }

    for (auto &e : futures) {
      e.get();
    }

    return;
  }

  void ImageSimilarityMatrix::buildImageSimilarityMatrix(const vector<Frame*>& frames, int maxFrameHeight)
  {
    cerr << "building ISM matrix" << endl;
    //create matrices and fill with zeros
    imageSimilarityMatrix.create(frames.size(), frames.size(), DataType<float>::type);
    imageShiftMatrix.create(frames.size(), frames.size(), DataType<Point2f>::type);
    // maskSimilarityMatrix.create(frames.size(), frames.size(), DataType<float>::type);

    for (uint32_t i = 0; i < frames.size(); ++i)
    {
      for (uint32_t j = 0; j < frames.size(); ++j)
      {
        imageSimilarityMatrix.at<float>(i, j) = 0;
        imageShiftMatrix.at<Point2f>(i, j) = Point2f(0, 0);
      }
    }

    //compute mask centroid offsets

    //store the futures
    vector<future<void> > futures;

    for (uint32_t i = 0; i < frames.size(); ++i)
    {
      for (uint32_t j = 0; j < frames.size(); ++j)
      {
        Frame* left = frames[i];
        Frame* right = frames[j];
        futures.push_back(std::async(&ImageSimilarityMatrix::computeISMcell, this, left, right, maxFrameHeight));
        //computeISMcell(frames, i, j);
      }
    }
    //get all futures

    for (auto &e : futures) {
      e.get();
    }

    return;
  }

  float ImageSimilarityMatrix::min() const//find the non-zero minimum in the image similarity matrix
  {
    float min = FLT_MAX;
    for (int i = 0; i < imageSimilarityMatrix.rows; ++i)
    {
      for (int j = 0; j < imageSimilarityMatrix.cols; ++j)
      {
        float val = imageSimilarityMatrix.at<float>(i, j);
        if (val != 0 && val < min && i != j)
          min = val;
      }
    }
    // cout << "THE MINUMUM FOR THIS ISM IS: " << min << endl;
    return min;
  }

  float ImageSimilarityMatrix::max() const//find the non-zero minimum in the image similarity matrix
  {
    float max = -1;
    for (int i = 0; i < imageSimilarityMatrix.rows; ++i)
    {
      for (int j = 0; j < imageSimilarityMatrix.cols; ++j)
      {
        float val = imageSimilarityMatrix.at<float>(i, j);
        if (val > max)
          max = val;
      }
    }

    return max;
  }

  float ImageSimilarityMatrix::mean() const//find the non-zero minimum in the image similarity matrix
  {
    float sum = 0;
    float count = 0;
    for (int i = 0; i < imageSimilarityMatrix.rows; ++i)
    {
      for (int j = 0; j < imageSimilarityMatrix.cols; ++j)
      {
        float val = imageSimilarityMatrix.at<float>(i, j);
        if (val != 0 && i != j)
        {
          count++;
          sum += val;
        }
      }
    }
    float mean = sum / count;
    // cout << "THE MEAN FOR THIS ISM IS: " << mean << endl;
    return mean;
  }

  float ImageSimilarityMatrix::stddev() const
  {
    float mean = this->mean();
    float sum = 0;
    float count = 0;
    for (int i = 0; i < imageSimilarityMatrix.rows; ++i)
    {
      for (int j = 0; j < imageSimilarityMatrix.cols; ++j)
      {
        float val = imageSimilarityMatrix.at<float>(i, j);
        if (val != 0 && i != j)
        {
          count++;
          sum += pow(val - mean, 2);
        }
      }
    }
    float sd = sum / count;
    // cout << "THE MEAN FOR THIS ISM IS: " << mean << endl;
    return sqrt(sd);
  }

  float ImageSimilarityMatrix::getPathCost(vector<int> path) const//get cost for path through ISM
  {
    //check that the path is valid
    for (uint32_t i = 0; i < path.size(); ++i)
    {
      if (!(path[i] < imageSimilarityMatrix.rows))
      {
        cerr << "Path contains invalid node " << path[i] << endl;
        return -1;
      }
    }
    float cost = 0;
    for (uint32_t i = 1; i < path.size(); ++i) //get the cost from previous node to this node to the end
    {
      cost += imageSimilarityMatrix.at<float>(path[i - 1], path[i]);
    }
    return cost;
  }

  //return the size of the ISM
  uint32_t ImageSimilarityMatrix::size() const
  {
    return imageSimilarityMatrix.rows;
  }

  Mat ImageSimilarityMatrix::clone()
  {
    return imageSimilarityMatrix.clone();
  }

}

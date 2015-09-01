#ifndef _IMAGESIMILARITYMATRIX_HPP_
#define _IMAGESIMILARITYMATRIX_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <vector>
#include <string>
#include <fstream>
#include <future>

// OpenCV
#include <opencv2/opencv.hpp>

// tree.hh
#include <tree_util.hh>

#include "frame.hpp"

namespace SPEL
{
  using namespace std;
  using namespace cv;

  class ImageSimilarityMatrix
  {
  public:

    ///constructors
    ImageSimilarityMatrix(void);
    ImageSimilarityMatrix(const ImageSimilarityMatrix& m);
    ImageSimilarityMatrix(const vector<Frame*>& frames);

    ///destructor
    virtual ~ImageSimilarityMatrix(void);

    virtual void buildImageSimilarityMatrix(const vector<Frame*>& frames, int maxFrameHeight = 0);
    virtual void buildMaskSimilarityMatrix(const vector<Frame*>& frames, int maxFrameHeight = 0);

    virtual bool read(string filename);
    virtual bool write(string filename) const;

    virtual float min() const;
    virtual float mean() const;
    virtual float max() const;
    virtual float stddev() const;

    virtual float at(int row, int col) const;
    virtual Point2f getShift(int row, int col) const;
    ///get cost for path through ISM
    virtual float getPathCost(vector<int> path) const;

    virtual uint32_t size() const;

    virtual bool operator==(const ImageSimilarityMatrix &s) const;
    virtual bool operator!=(const ImageSimilarityMatrix &s) const;
    virtual ImageSimilarityMatrix & operator=(const ImageSimilarityMatrix &s);

    virtual Mat clone(); //return a Mat clone of ISM

  protected:

    virtual void computeMSMcell(Frame* left, Frame* right, int maxFrameHeight);
    virtual void computeISMcell(Frame* left, Frame* right, int maxFrameHeight);
    ///the image similarity matrix
    Mat imageSimilarityMatrix;
    Mat imageShiftMatrix;

  };
}
#endif  // _IMAGESIMILARITYMATRIX_HPP_


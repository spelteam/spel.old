#if defined(WINDOWS) && defined(_MSC_VER)
#include <Windows.h>
#endif

#include <gtest/gtest.h>
#include "imagesimilaritymatrix.hpp"

namespace SPEL
{
  class TestSMatrix : public ImageSimilarityMatrix
  {
  public:
    Mat getImageShiftMatrix();
  };

  Mat TestSMatrix::getImageShiftMatrix()
  {
    return imageShiftMatrix;
  }

  class ImageSimilarityMatrixTests : public testing::Test
  {
  protected:
    virtual void SetUp()
    {
      FilePath = "speltests_TestData/SimilarityMatrixTestsData/";
#if defined(WINDOWS) && defined(_MSC_VER)
      if (IsDebuggerPresent())
          FilePath = "Debug/" + FilePath;
#endif
      a = A.read(FilePath + "In_Matrix.txt");
    }

    String FilePath;
    ImageSimilarityMatrix A;
    bool a;
  };

  TEST_F(ImageSimilarityMatrixTests, WriteAndRead)
    {
      int rows = 3, cols = rows;
      TestSMatrix X, Y;

      // Testing "read""
      bool b = X.read(FilePath + "In_Matrix.txt");
      Mat X_ShiftMatrix = X.getImageShiftMatrix();

      ASSERT_TRUE(b);
      ASSERT_EQ(rows, X.size());
      for (int i = 0; i < rows; i++)
        for (int k = 0; k < cols; k++)
        {
          float x = float(rows*i + k +1);
          EXPECT_EQ(x, X.at(i,k));
          EXPECT_EQ(Point2f(x/10, x/10), X_ShiftMatrix.at<Point2f>(i, k));
        }
      
      // Testing "write"
      b = X.write(FilePath + "Out_Matrix.txt");
      ASSERT_TRUE(b);

      b = Y.read(FilePath + "Out_Matrix.txt");
      Mat Y_ShiftMatrix = Y.getImageShiftMatrix();
      ASSERT_TRUE(b);
      ASSERT_EQ(rows, Y.size());
      for (int i = 0; i < rows; i++)
        for (int k = 0; k < cols; k++)
        {
          float x = float(rows*i + k + 1);
          EXPECT_EQ(x, Y.at(i, k));
          EXPECT_EQ(Point2f(x / 10, x / 10), Y_ShiftMatrix.at<Point2f>(i, k));
        }  	
    }

  TEST_F(ImageSimilarityMatrixTests, at)
  {
    EXPECT_EQ(1.0, A.at(0, 0));
  }

  TEST_F(ImageSimilarityMatrixTests, getShift)
  {
    EXPECT_EQ(Point2f(0.1,0.1), A.getShift(0, 0));
  }

  TEST_F(ImageSimilarityMatrixTests, Operators)
  {
    ImageSimilarityMatrix B, C;
    bool b, c;

    b = B.read(FilePath + "In_Matrix.txt");
    c = C.read(FilePath + "In_Matrix2.txt");
    ASSERT_TRUE(a && b && c);

    // Operator "=="
    EXPECT_TRUE(A == B); // This does not work. Error in "ImageSimilarityMatrix::operator=="???
    EXPECT_FALSE(A == C);

    //Operator "!="
    EXPECT_FALSE(A != B);
    EXPECT_TRUE(A != C);

    //Operator "="
    C = A;
    EXPECT_TRUE(A == C);
  }

  TEST_F(ImageSimilarityMatrixTests, min)
   {
     ASSERT_TRUE(a);
     EXPECT_FLOAT_EQ(2.0, A.min());
   }

  TEST_F(ImageSimilarityMatrixTests, max)
  {
    ASSERT_TRUE(a);
    EXPECT_FLOAT_EQ(9.0, A.max());
  }

  TEST_F(ImageSimilarityMatrixTests, mean)
  {
    ASSERT_TRUE(a);
    EXPECT_FLOAT_EQ(5.0, A.mean());
  }

  TEST_F(ImageSimilarityMatrixTests, stddev)
  {
    ASSERT_TRUE(a);
    EXPECT_FLOAT_EQ(2.1602468, A.stddev());
  }

  TEST_F(ImageSimilarityMatrixTests, getPathCost)
  {
    vector<int> path = { 0, 1, 2 };

    ASSERT_TRUE(a);
    EXPECT_FLOAT_EQ(8.0, A.getPathCost(path));

    path.push_back(4);
    EXPECT_FLOAT_EQ(-1.0, A.getPathCost(path));
    path.clear();
  }

  TEST_F(ImageSimilarityMatrixTests, size)
  {
    ASSERT_TRUE(a);
    EXPECT_EQ(3, A.size());
  }

  TEST_F(ImageSimilarityMatrixTests, clone)
  {
    Mat B = A.clone();
    ASSERT_EQ((int)A.size(), B.size().width);
    for (int i = 0; i < A.size(); i++)
      for (int k = 0; k < A.size(); k++)
      {
        float x = float(A.size()*i + k + 1);
        EXPECT_EQ(x, B.at<float>(i, k));
      }
  }
}
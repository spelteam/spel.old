#ifndef _BODYPART_HPP_
#define _BODYPART_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <string>

#include "bodyJoint.hpp"
#include "spelHelper.hpp"

namespace SPEL
{
  using namespace std;
  /// Objects of this class used as elements for building a skeleton model
  /// See [[Skeleton]].hpp for more info

  class BodyPart
  {
  public:
    BodyPart(void);
    BodyPart(const BodyPart& bodyPart);
    BodyPart(BodyPart&& bodyPart);
    BodyPart(int id, string name, int pJoint, int cJoint, bool isOcc = false, float spaceLen = 0);
    virtual ~BodyPart(void);
    virtual BodyPart& operator=(const BodyPart& bodyPart);
    virtual BodyPart& operator=(BodyPart&& bodyPart);
    /// comparsion by unique index
    virtual bool operator==(const BodyPart &bp) const;
    /// comparsion by address
    virtual bool operator!=(const BodyPart &bp) const;
    // get and set: All these functions just give access to the object fields
    virtual int getPartID(void) const;
    virtual void setPartID(int _partID);
    virtual string getPartName(void) const;
    virtual void setPartName(string _partName);
    virtual int getParentJoint(void) const;
    virtual void setParentJoint(int _parentJoint);
    virtual int getChildJoint(void) const;
    virtual void setChildJoint(int _childJoint);
    virtual bool getIsOccluded(void) const;
    virtual void setIsOccluded(bool _isOccluded);
    virtual float getExpectedDistance(void) const;
    virtual void setExpectedDistance(float _expectedDistance);
    virtual POSERECT <Point2f> getPartPolygon(void) const;
    virtual void setPartPolygon(POSERECT <Point2f> _partPolygon);
    virtual float getLWRatio(void) const;
    virtual void setLWRatio(float _lwRatio);
    virtual float getRelativeLength(void) const;
    virtual void setRelativeLength(float _relativeLength);
    //search parameters
    virtual float getSearchRadius(void) const;
    virtual void setSearchRadius(float _searchRadius);
    virtual float getRotationSearchRange(void) const;
    virtual void setRotationSearchRange(float _rotationAngle);
  private:
    /// identifier, must be unique within the limits of class
    int partID;
    /// the object name, respectively to a place in a skeleton model 
    string partName;
    /// identifier of adjacent overlying joint/node (see BodyJoint.hpp)
    int parentJoint;
    /// identifier of adjacent underlying joint/node (see BodyJoint.hpp)
    int childJoint;
    /// when "true" - then this body part is overlapped in a frame, used in the skeleton recovery algorithm
    bool isOccluded;
    /// expected distance to parent bodypart, as a multiplier of this part's length
    float expectedDistance;
    /// rectangle is used as simplified representation of body part 
    POSERECT <Point2f> partPolygon;
    /// coefficient of proportionality is used for scaling
    float lwRatio;
    /// 3d relative length
    float relativeLength;
    /// search radius for detection of this bodypart
    float searchRadius;
    /// rotation angle range to search through
    float rotationSearchRange;
  };

  std::ostream& operator<<(std::ostream& os, const BodyPart &bp);

}
#endif  // _BODYPART_HPP_


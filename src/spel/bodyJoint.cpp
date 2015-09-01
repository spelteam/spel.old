#include "bodyJoint.hpp"
// See bodyJoint.hpp and Skeleton.hpp for more info
namespace SPEL
{
  //default constructor
  BodyJoint::BodyJoint(void)
  {
    setLimbID(0);
    setJointName("");
    setImageLocation({ 0.0, 0.0 });
    setSpaceLocation({ 0.0, 0.0, 0.0 });
    setDepthSign(false);
  }

  // constructor with params
  BodyJoint::BodyJoint(int id, string name, Point2f imgLoc, Point3f spaceLoc, bool depth)
  {
    setLimbID(id);
    setJointName(name);
    setImageLocation(imgLoc);
    setSpaceLocation(spaceLoc);
    setDepthSign(depth);
  }

  //copy constructor
  BodyJoint::BodyJoint(const BodyJoint &bodyJoint)
    : limbID(bodyJoint.limbID),
    jointName(bodyJoint.jointName),
    imageLocation(bodyJoint.imageLocation),
    spaceLocation(bodyJoint.spaceLocation),
    depthSign(bodyJoint.depthSign)
  {
  }

  //move constructor
  BodyJoint::BodyJoint(BodyJoint &&bodyJoint)
    : limbID(std::move(bodyJoint.limbID)),
    jointName(std::move(bodyJoint.jointName)),
    imageLocation(std::move(bodyJoint.imageLocation)),
    spaceLocation(std::move(bodyJoint.spaceLocation)),
    depthSign(std::move(bodyJoint.depthSign))
  {
  }

  BodyJoint::~BodyJoint(void)
  {
    return;
  }

  int BodyJoint::getLimbID(void) const
  {
    return limbID;
  }

  void BodyJoint::setLimbID(int _limbID)
  {
    limbID = _limbID;
  }

  string BodyJoint::getJointName(void) const
  {
    return jointName;
  }

  void BodyJoint::setJointName(string _jointName)
  {
    jointName = _jointName;
  }

  Point2f BodyJoint::getImageLocation(void) const
  {
    return imageLocation;
  }

  void BodyJoint::setImageLocation(Point2f _imageLocation)
  {
    imageLocation = _imageLocation;
  }

  Point3f BodyJoint::getSpaceLocation(void) const
  {
    return spaceLocation;
  }

  void BodyJoint::setSpaceLocation(Point3f _spaceLocation)
  {
    spaceLocation = _spaceLocation;
  }

  bool BodyJoint::getDepthSign(void) const
  {
    return depthSign;
  }

  void BodyJoint::setDepthSign(bool _depthSign)
  {
    depthSign = _depthSign;
  }

  BodyJoint& BodyJoint::operator=(const BodyJoint& bodyJoint){
    if (&bodyJoint == this) return *this;

    limbID = bodyJoint.limbID;
    jointName = bodyJoint.jointName;
    imageLocation = bodyJoint.imageLocation;
    spaceLocation = bodyJoint.spaceLocation;
    depthSign = bodyJoint.depthSign;

    return *this;
  }

  BodyJoint& BodyJoint::operator=(BodyJoint&& bodyJoint){
    limbID = std::move(bodyJoint.limbID);
    std::swap(jointName, bodyJoint.jointName);
    std::swap(imageLocation, bodyJoint.imageLocation);
    std::swap(spaceLocation, bodyJoint.spaceLocation);
    depthSign = std::move(bodyJoint.depthSign);

    return *this;
  }

  bool BodyJoint::operator==(const BodyJoint &bj) const
  {
    return this->getLimbID() == bj.getLimbID();
  }

  bool BodyJoint::operator!=(const BodyJoint &bj) const
  {
    return !(*this == bj);
  }

}

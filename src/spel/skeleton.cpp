#include "skeleton.hpp"
//See Skeleton.hpp for more info
namespace SPEL
{
  //default constructor
  Skeleton::Skeleton(void)
  {
    /// name of the specific instance of
    name = "uninitialised";
    /// tree of bodyparts is component of the body model
    /// scale factor, used for scaling
    scale = 0;
  }

  Skeleton::Skeleton(const Skeleton &s)
  {
    this->setName(s.getName());
    this->setPartTree(s.getPartTree());
    this->setJointTree(s.getJointTree());
    this->setScale(s.getScale());
  }

  Skeleton::~Skeleton(void)
  {
  }

  // constructor with params
  Skeleton &Skeleton::operator=(const Skeleton &s)
  {
    if (this == &s)
    {
      return *this;
    }
    this->setName(s.getName());
    this->setPartTree(s.getPartTree());
    this->setJointTree(s.getJointTree());
    this->setScale(s.getScale());
    return *this;
  }

  bool Skeleton::operator==(const Skeleton &s) const
  {
    tree <BodyPart> src1 = this->getPartTree();
    tree <BodyPart> src2 = s.getPartTree();
    return equal(src1.begin(), src1.end(), src2.begin());
  }

  bool Skeleton::operator!=(const Skeleton &s) const
  {
    return !(*this == s);
  }

  string Skeleton::getName(void) const
  {
    return name;
  }

  void Skeleton::setName(string _name)
  {
    name = _name;
  }

  tree <BodyPart> Skeleton::getPartTree(void) const
  {
    return partTree;
  }

  tree<BodyPart>* Skeleton::getPartTreePtr(){
    return &partTree;
  }

  void Skeleton::setPartTree(tree <BodyPart> _partTree)
  {
    partTree = _partTree;
  }

  tree <BodyJoint> Skeleton::getJointTree(void) const
  {
    return jointTree;
  }

  tree<BodyJoint>* Skeleton::getJointTreePtr(){
    return &jointTree;
  }

  void Skeleton::setJointTree(tree <BodyJoint> _jointTree)
  {
    jointTree = _jointTree;
  }

  float Skeleton::getScale(void) const
  {
    return scale;
  }

  void Skeleton::setScale(float _scale)
  {
    scale = _scale;
  }

  uint32_t Skeleton::getPartTreeCount(void) const
  {
    return (uint32_t)partTree.size();
  }

  //void Skeleton::shift(Point2f point) //shift in 2D and recompute 3D?
  //{
  //    for(tree <BodyJoint>::iterator i = jointTree.begin(); i != jointTree.end(); ++i)
  //    {
  //        //add point to every joint
  //        Point2f prevLoc = i->getImageLocation();
  //        Point2f nextLoc = prevLoc+point;
  //        i->setImageLocation(nextLoc);
  //    }
  //}

  BodyJoint *Skeleton::getBodyJoint(int jointID) const
  {
    BodyJoint *joint = 0;
    for (tree <BodyJoint>::iterator i = jointTree.begin(); i != jointTree.end(); ++i)
    {
      if (i->getLimbID() == jointID)
      {
        joint = &*i;
      }
    }
    return joint;
  }

  BodyPart* Skeleton::getBodyPart(int partID) const
  {
    auto it = partTree.begin();
    while (it != partTree.end()){
      if (it->getPartID() == partID){
        return &(*it);
      }
      ++it;
    }
    return nullptr;
  }

  void Skeleton::infer2D(void)
  {
    for (tree <BodyJoint>::iterator tree = jointTree.begin(); tree != jointTree.end(); ++tree)
    {
      tree->setImageLocation(Point2f(tree->getSpaceLocation().x*scale, tree->getSpaceLocation().y*scale));
    }
  }

  void Skeleton::infer3D(void)
  {
    map <uint32_t, float> dz;

    assert(scale != 0);

    for (tree <BodyPart>::iterator tree = partTree.begin(); tree != partTree.end(); ++tree)
    {
      float len3d = tree->getRelativeLength();
      float len2d = sqrt(spelHelper::distSquared(getBodyJoint(tree->getParentJoint())->getImageLocation(), getBodyJoint(tree->getChildJoint())->getImageLocation()));
      float diff = pow(len3d, 2) - pow(len2d / scale, 2); //compute the difference, this must be the depth
      if (diff < 0)
        dz[tree->getPartID()] = 0;
      else
        dz[tree->getPartID()] = sqrt(diff);

      if (sqrt(diff) > len3d) dz[tree->getPartID()] = len3d;
    }
    for (tree <BodyPart>::iterator tree = partTree.begin(); tree != partTree.end(); ++tree)
    {
      BodyJoint *child = getBodyJoint(tree->getChildJoint());
      BodyJoint *parent = getBodyJoint(tree->getParentJoint());
      if (tree->getPartID() == 0) //if zero partID, we are on the root part
      {
        parent->setSpaceLocation(Point3f(parent->getImageLocation().x / scale, parent->getImageLocation().y / scale, 0));
      }
      float sign = child->getDepthSign() == 0 ? -1.0 : 1.0;
      float z = tree == partTree.begin() ? 0.0 : parent->getSpaceLocation().z;
      child->setSpaceLocation(Point3f(child->getImageLocation().x / scale, child->getImageLocation().y / scale, sign * dz.at(tree->getPartID()) + z));
    }
  }

}

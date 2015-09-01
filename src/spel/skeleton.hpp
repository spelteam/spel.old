#ifndef _LIBPOSE_SKELETON_HPP_
#define _LIBPOSE_SKELETON_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <string>

// tree.hh
#include <tree.hh>

#include "bodyPart.hpp"

namespace SPEL
{
  using namespace std;

  ///Human body model
  class Skeleton
  {
  public:
    Skeleton(void);
    Skeleton(const Skeleton &s);
    virtual ~Skeleton(void);
    //TODO (Vitaliy Koshura): Need implementation
    //string toString(void); // must return Skeleton as string
    //TODO (Vitaliy Koshura): Need implementation
    //void learnDepth(Skeleton &skel);
    virtual void infer2D(void);
    virtual void infer3D(void);
    // All these functions just give access to the object fields
    virtual Skeleton & operator=(const Skeleton &s);
    virtual bool operator==(const Skeleton &s) const;
    virtual bool operator!=(const Skeleton &s) const;
    virtual string getName(void) const;
    virtual void setName(string _name);
    virtual tree <BodyPart> getPartTree(void) const;
    ///direct access
    virtual tree<BodyPart>* getPartTreePtr(void);
    virtual void setPartTree(tree <BodyPart> _partTree);
    virtual tree <BodyJoint> getJointTree(void) const;
    ///direct access
    virtual tree<BodyJoint>* getJointTreePtr(void);
    virtual void setJointTree(tree <BodyJoint> _jointTree);
    virtual float getScale(void) const;
    virtual void setScale(float _scale);
    //void shift(Point2f point);

    /// count of bodypart elements, included in the tree
    virtual uint32_t getPartTreeCount(void) const;
    /// search a joint by id and return a pointer to its address
    virtual BodyJoint* getBodyJoint(int jointID) const;
    /// search a body part by id and return a pointer to its address
    virtual BodyPart* getBodyPart(int partID) const;
  private:
    /// name of the specific instance of
    string name;
    /// tree of bodyparts is component of the body model
    tree <BodyPart> partTree;
    /// tree of joints is component of the body model
    tree <BodyJoint> jointTree;
    /// scale factor, used for scaling
    float scale;
  };

}

#endif  // _LIBPOSE_SKELETON_HPP_


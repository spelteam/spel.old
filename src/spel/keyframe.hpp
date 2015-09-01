#ifndef _KEYFRAME_HPP_
#define _KEYFRAME_HPP_

// SPEL definitions
#include "predef.hpp"

#include "frame.hpp"

namespace SPEL
{
  ///This class represents user defined
  ///frame( implies that user analyzes the frame
  /// and make a mark points ).
  class Keyframe : public Frame
  {
  public:
    Keyframe(void);
    virtual ~Keyframe(void);
  };

}

#endif  // _KEYFRAME_HPP_


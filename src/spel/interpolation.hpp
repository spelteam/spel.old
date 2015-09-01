#ifndef _INTERPOLATION_HPP_
#define _INTERPOLATION_HPP_

// SPEL definitions
#include "predef.hpp"

#include "frame.hpp"
namespace SPEL
{
  ///This class represents frames that
  ///interpolated between keyframes
  class Interpolation : public Frame
  {
  public:
    Interpolation(void);
    virtual ~Interpolation(void);
  };
}
#endif  // _INTERPOLATION_HPP_


#ifndef _SCORE_HPP_
#define _SCORE_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <string>

namespace SPEL
{
  using namespace std;
  /// Used to evaluate accuracy of a detection
  class Score
  {
  public:
    Score(void);
    Score(float sc, string name, float _coeff = 1.0f);
    virtual ~Score(void);
    /// copying all fields
    virtual Score & operator=(const Score &s);
    /// All this operators perform comparison by score values
    virtual bool operator<(const Score &s) const;
    virtual bool operator>(const Score &s) const;
    virtual bool operator==(const Score &s) const;
    /// This operator perform comparison by address
    virtual bool operator!=(const Score &s) const;
    /// All this functions just give access to the object fields
    virtual float getScore(void) const;
    virtual void setScore(float _score);
    virtual string getDetName(void) const;
    virtual void setDetName(string _detName);
    virtual float getCoeff(void) const;
    virtual void setCoeff(float _coeff);
    virtual bool getIsWeak(void) const;
    virtual void setIsWeak(bool _isWeak);

  private:
    /// detection score
    float score;
    /// detector name, name of the algorithm that generate the evaluation
    string detName;
    float coeff;
    /// signify label is from a badly localised part i.e. all very weak or all very similar detection scores
    bool isWeak;
  };

}

#endif  // _SCORE_HPP_


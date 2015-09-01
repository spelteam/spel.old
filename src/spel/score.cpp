#include "score.hpp"
// See Score.hpp for more info
namespace SPEL
{
  // default constructor
  Score::Score(void)
  {
    score = 0;
    detName = "";
    coeff = 1.0f;
    isWeak = false;
  }

  // constructor with params
  Score::Score(float sc, string name, float _coeff)
  {
    score = sc;
    detName = name;
    coeff = _coeff;
    isWeak = false;
  }

  Score::~Score(void)
  {
  }

  Score &Score::operator=(const Score &s)
  {
    if (this == &s)
    {
      return *this;
    }
    this->detName = s.getDetName();
    this->score = s.getScore();
    this->coeff = s.getCoeff();
    this->isWeak = s.getIsWeak();
    return *this;
  }

  float Score::getScore(void) const
  {
    return score;
  }

  void Score::setScore(float _score)
  {
    score = _score;
  }

  string Score::getDetName(void) const
  {
    return detName;
  }

  void Score::setDetName(string _detName)
  {
    detName = _detName;
  }

  bool Score::operator<(const Score &s) const
  {
    return (this->score * this->coeff < s.getScore() * s.getCoeff());
  }

  bool Score::operator>(const Score &s) const
  {
    return (this->score * this->coeff > s.getScore() * s.getCoeff());
  }

  bool Score::operator==(const Score &s) const
  {
    return (this->score * this->coeff == s.getScore() * s.getCoeff() && this->detName == s.getDetName());
  }

  bool Score::operator!=(const Score &s) const
  {
    return !(*this == s);
  }

  float Score::getCoeff(void) const
  {
    return coeff;
  }

  void Score::setCoeff(float _coeff)
  {
    coeff = _coeff;
  }

  bool Score::getIsWeak(void) const
  {
    return isWeak;
  }
  void Score::setIsWeak(bool _isWeak)
  {
    isWeak = _isWeak;
  }

}

#ifndef _PROJECTRUNNER_H_
#define _PROJECTRUNNER_H_

#include <iostream>
#include "projectLoader.hpp"

using namespace std;
using namespace SPEL;

class ProjectRunner
{
public:
  ProjectRunner(string _testName);
  int Run(int argc, char **argv, map <uint32_t, map <uint32_t, vector <LimbLabel>>> *limbLabels = 0);
  virtual void train(vector <Frame*> _frames, map <string, float> params) = 0;
  virtual map <uint32_t, vector <LimbLabel> > detect(Frame *frame, map <string, float> params, map <uint32_t, vector <LimbLabel>> limbLabels) = 0;
  virtual void DrawSpecific(string outFolder) { };
private:
  string testName;
};

#endif // _PROJECTRUNNER_H_

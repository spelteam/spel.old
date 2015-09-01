#include "projectRunner.hpp"

ProjectRunner::ProjectRunner(string _testName)
{
  testName = _testName;
}

int ProjectRunner::Run(int argc, char **argv, map <uint32_t, map <uint32_t, vector <LimbLabel>>> *limbLabels)
{
  if (argc < 3)
  {
    cout << "Usage:\t" << argv[0] << " [project.xml] [out directory] [--no-draw]" << endl;
    return -1;
  }

  bool bDraw = true;
  if (argc > 3)
  {
    for (int i = 3; i < argc; i++)
    {
      if (strcmp(argv[i], "--no-draw") == 0)
        bDraw = false;
    }
  }

  string curFolder = argv[1];
  curFolder = curFolder.substr(0, curFolder.find_last_of("/"));
  if (curFolder.back() != '/')
  {
    curFolder += '/';
  }

  ProjectLoader projectLoader(curFolder);

  cout << "Loading project..." << endl;

  if (projectLoader.Load(argv[1]) == true)
  {
    cout << "Project was successfully loaded" << endl;
  }
  else
  {
    cout << "Project was not loaded" << endl;
    return -1;
  }
  map <string, float> params;
  params.emplace("debugLevel", 3);

  vector <Frame*> allFrames = projectLoader.getFrames();
  vector <Frame*> trainFrames;
  int8_t kfCount = 0;

  Sequence *seq = 0;
  try
  {
    seq = new Sequence(0, testName, allFrames);
    if (seq != 0)
    {
      seq->estimateUniformScale(params);
      seq->computeInterpolation(params);
    }
    else
    {
      for (auto f : allFrames)
        delete f;
      allFrames.clear();
      return -1;
    }
  }
  catch (exception &e)
  {
    cerr << e.what() << endl;
    for (auto f : allFrames)
      delete f;
    allFrames.clear();
    return -1;
  }

  for (auto frame : seq->getFrames())
  {
    if (frame->getFrametype() != KEYFRAME && frame->getFrametype() != LOCKFRAME && kfCount == 1)
    {
      trainFrames.push_back(frame);
    }
    else if (frame->getFrametype() == KEYFRAME || frame->getFrametype() == LOCKFRAME)
    {
      trainFrames.push_back(frame);
      kfCount++;
    }

    if (kfCount < 2)
      continue;

    kfCount = 0;
    cout << "Training..." << endl;
    try
    {
      train(trainFrames, params);
    }
    catch (exception &e)
    {
      cerr << e.what() << endl;
      return -1;
    }

    cout << "Training complete" << endl;
    vector <Frame*>::iterator i;
    map <string, float> detectParams;
    cout << "Detecting..." << endl;
    for (i = trainFrames.begin(); i != trainFrames.end(); ++i)
    {
      tree <BodyPart> prevBodyPartTree, nextBodyPartTree;
      //check if frame has a keyframe before AND after
      bool hasPrevAnchor = false, hasFutureAnchor = false;
      //before
      for (vector<Frame*>::reverse_iterator prevAnchor(i); prevAnchor != trainFrames.rend(); ++prevAnchor)
      {
        if ((*prevAnchor)->getFrametype() == KEYFRAME || (*prevAnchor)->getFrametype() == LOCKFRAME)
        {
          hasPrevAnchor = true;
          prevBodyPartTree = (*prevAnchor)->getSkeleton().getPartTree();
          break;
        }
      }
      //after
      for (vector<Frame*>::iterator futureAnchor = i; futureAnchor != trainFrames.end(); ++futureAnchor)
      {
        if ((*futureAnchor)->getFrametype() == KEYFRAME || (*futureAnchor)->getFrametype() == LOCKFRAME)
        {
          hasFutureAnchor = true;
          nextBodyPartTree = (*futureAnchor)->getSkeleton().getPartTree();
          break;
        }
      }
      if (!hasPrevAnchor || !hasFutureAnchor)
        continue;
      Frame *f = *i;
      Skeleton frameSkeleton = f->getSkeleton();
      tree <BodyPart> frameBodyPartTree = frameSkeleton.getPartTree();
      for (tree <BodyPart>::iterator i = frameBodyPartTree.begin(); i != frameBodyPartTree.end(); ++i)
      {
        for (tree <BodyPart>::iterator j = prevBodyPartTree.begin(); j != prevBodyPartTree.end(); ++j)
        {
          if (i->getPartID() == j->getPartID())
          {
            for (tree <BodyPart>::iterator k = nextBodyPartTree.begin(); k != nextBodyPartTree.end(); ++k)
            {
              if (j->getPartID() == k->getPartID())
              {
                i->setRotationSearchRange(abs(j->getRotationSearchRange() - k->getRotationSearchRange()));
                break;
              }
            }
            break;
          }
        }
      }
      frameSkeleton.setPartTree(frameBodyPartTree);
      f->setSkeleton(frameSkeleton);
#ifndef DEBUG
      if (f->getFrametype() == INTERPOLATIONFRAME)
#endif  // DEBUG
      {
        map <uint32_t, vector <LimbLabel> > labels;
        if (limbLabels != 0)
        {
          try
          {
            labels = limbLabels->at(f->getID());
          }
          catch (...) {}
        }
          
        try
        {
          cout << "Detecting frame " << f->getID() << "..." << endl;
          labels = detect(f, detectParams, labels);
          if (limbLabels != 0)
          {
            try
            {
              limbLabels->at(f->getID()) = labels;
            }
            catch (...)
            {
              limbLabels->insert(pair<uint32_t, map <uint32_t, vector <LimbLabel>>>(f->getID(), labels));
            }
          }
          projectLoader.Save(labels, argv[2], f->getID());
          if (bDraw)
          {
            projectLoader.Draw(labels, f, argv[2], f->getID(), Scalar(0, 0, 0), Scalar(0, 0, 255), 2);
            DrawSpecific(argv[2]);
          }
        }
        catch (exception &e)
        {
          cerr << e.what() << endl;
          continue;
        }
      }
    }
    cout << "Detecting complete" << endl;
    trainFrames.clear();
  }  

  if (seq != 0)
    delete seq;

  return 0;
}

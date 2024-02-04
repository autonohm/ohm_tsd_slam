#include "Trace.h"
#include "obcore/base/System.h"
#include "obcore/base/Logger.h"

#include <iostream>
#include <fstream>
#include <iomanip>

namespace obvious
{

Trace::Trace(unsigned int dim)
{
  _dim      = dim;
  _M        = NULL;
  _S        = NULL;
}
		 
Trace::~Trace()
{
  reset();
}
	
void Trace::reset()
{
  if(_M)
  {
    delete _M;
    _M = NULL;
  }

  if(_S)
  {
    delete _S;
    _S = NULL;
  }

  for(unsigned int i=0; i<_scenes.size(); i++)
    delete _scenes[i];
  _scenes.clear();

  _ids.clear();

  _pairs.clear();

  _scores.clear();
}

void Trace::setModel(double** model, unsigned int sizeM)
{
  if(model)
  {
    if(_M) delete _M;
    _M = new Matrix(sizeM, _dim, *model);
  }
  else
  {
    LOGMSG(DBG_WARN, "Empty model passed");
  }
}

void Trace::setModel(Matrix* M, vector<unsigned int> validPoints)
{
  double** rawModel;
  System<double>::allocate(validPoints.size(), 2, rawModel);
  for(unsigned int i=0; i<validPoints.size(); i++)
  {
    rawModel[i][0] = (*M)(validPoints[i], 0);
    rawModel[i][1] = (*M)(validPoints[i], 1);
  }
  setModel(rawModel, validPoints.size());

  System<double>::deallocate(rawModel);
}

void Trace::setScene(double** scene, unsigned int sizeS)
{
  if(scene)
  {
    if(_S) delete _S;
    _S = new Matrix(sizeS, _dim, *scene);
  }
  else
  {
    LOGMSG(DBG_WARN, "Empty scene passed");
  }
}

void Trace::setScene(Matrix* S, vector<unsigned int> validPoints)
{
  double** rawScene;
  System<double>::allocate(validPoints.size(), 2, rawScene);
  for(unsigned int i=0; i<validPoints.size(); i++)
  {
    rawScene[i][0] = (*S)(validPoints[i], 0);
    rawScene[i][1] = (*S)(validPoints[i], 1);
  }
  setScene(rawScene, validPoints.size());

  System<double>::deallocate(rawScene);
}

void Trace::addAssignment(double** scene, unsigned int sizeS, vector<StrTraceCartesianPair> pairs, const double score, vector<unsigned int> id)
{
  obvious::Matrix* sceneCopy = new obvious::Matrix(sizeS, _dim, *scene);
  _scenes.push_back(sceneCopy);
  _ids.push_back(id);
  _pairs.push_back(pairs);
  _scores.push_back(score);
}

void Trace::addAssignment(Matrix* M, vector<unsigned int> idxM, Matrix* S, vector<unsigned int> idxS, Matrix* STrans, const double score, const unsigned int iterationID)
{
  if(idxM.size() != idxS.size())
  {
    LOGMSG(DBG_ERROR, "Size of model and scene assignment vectors must be equal");
    return;
  }
  double** rawScene;
  System<double>::allocate((*STrans).getCols(), 2, rawScene);
  for(unsigned int j=0; j<(*STrans).getCols(); j++)
  {
    rawScene[j][0] = (*STrans)(0, j);
    rawScene[j][1] = (*STrans)(1, j);
  }
  vector<StrTraceCartesianPair> tracePair;
  for(unsigned int i=0; i<idxM.size(); i++)
  {
    StrTraceCartesianPair p;
    p.first[0] = (*M)(idxM[i], 0);
    p.first[1] = (*M)(idxM[i], 1);
    p.second[0] = (*S)(idxS[i], 0);
    p.second[1] = (*S)(idxS[i], 1);
    tracePair.push_back(p);
  }
  vector<unsigned int> id;
  id.push_back(iterationID);
  id.push_back(idxM[0]);
  id.push_back(idxS[0]);
  addAssignment(rawScene, STrans->getCols(), tracePair, score, id);
  System<double>::deallocate(rawScene);
}

void Trace::serialize(const char* folder)
{
  char cmd[256];
  sprintf(cmd, "mkdir %s", folder);
  int retval = system(cmd);
  if(retval==0)
  {
    ofstream file;
    char filename[512];
    double minCoord[3] = { 10e12,  10e12,  10e12};
    double maxCoord[3] = {-10e12, -10e12, -10e12};

    // -------------------- Model ----------------------
    if(_M)
    {
      snprintf(filename, 512, "%s/model.dat", folder);
      file.open(filename, ios::out);
      for(unsigned int p=0; p<_M->getRows(); p++)
      {
        for(unsigned int j=0; j<_dim; j++)
        {
          double coord = (*_M)(p,j);
          if(minCoord[j]>coord) minCoord[j] = coord;
          if(maxCoord[j]<coord) maxCoord[j] = coord;
          file << std::setprecision(9) << std::fixed << coord << " ";
        }
        file << endl;
      }
      file.close();
    }
    // -------------------------------------------------

    // -------------------- Scene ----------------------
    if(_S)
    {
      snprintf(filename, 512, "%s/scene.dat", folder);
      file.open(filename, ios::out);
      for(unsigned int p=0; p<_S->getRows(); p++)
      {
        for(unsigned int j=0; j<_dim; j++)
        {
          double coord = (*_S)(p,j);
          if(minCoord[j]>coord) minCoord[j] = coord;
          if(maxCoord[j]<coord) maxCoord[j] = coord;
          file << std::setprecision(9) << std::fixed << coord << " ";
        }
        file << endl;
      }
      file.close();
    }
    // -------------------------------------------------

    // ------------- Transformed scenes ----------------
    for(unsigned int i=0; i<_scenes.size(); i++)
    {
      vector<unsigned int> id = _ids[i];
      if(id.size()==3)
        snprintf(filename, 512, "%s/scene_%05d_%05d_%05d.dat", folder, id[0], id[1], id[2]);
      else
        snprintf(filename, 512, "%s/scene_%05d.dat", folder, i);
      file.open(filename, ios::out);
      Matrix* S = _scenes[i];
      for(unsigned int p=0; p<S->getRows(); p++)
      {
        for(unsigned int j=0; j<_dim; j++)
        {
          double coord = (*S)(p,j);
          if(minCoord[j]>coord) minCoord[j] = coord;
          if(maxCoord[j]<coord) maxCoord[j] = coord;
          file << std::setprecision(9) << std::fixed << coord << " ";
        }
        file << endl;
      }
      file.close();
    }
    // -------------------------------------------------

    // ------------------ Pairs -------------------------
    for(unsigned int i=0; i<_pairs.size(); i++)
    {
      vector<unsigned int> id = _ids[i];
      if(id.size()==3)
        snprintf(filename, 512, "%s/pairs_%05d_%05d_%05d.dat", folder, id[0], id[1], id[2]);
      else
        snprintf(filename, 512, "%s/pairs_%05d.dat", folder, i);

      file.open(filename, ios::out);
      for(unsigned int p=0; p<_pairs[i].size(); p++)
      {
        StrTraceCartesianPair pair = _pairs[i][p];

        for(unsigned int j=0; j<_dim; j++)
        {
          double coord = pair.first[j];
          file << std::setprecision(9) << std::fixed << coord << " ";
        }
        for(unsigned int j=0; j<_dim; j++)
        {
          double coord = pair.second[j];
          file << std::setprecision(9) << std::fixed << coord << " ";
        }
        file << endl;
      }
      file.close();
    }
    // -------------------------------------------------

    // ------------------ Scores -----------------------
    for(unsigned int i=0; i<_scores.size(); i++)
    {
      vector<unsigned int> id = _ids[i];

      bool newfile = false;
      if(i==0)
      {
        newfile = true;
      }
      else if(id.size()==3)
      {
        newfile = (id[0] != _ids[i-1][0]);
      }

      if(newfile)
      {
        if(file.is_open()) file.close();
        if(id.size()>=2)
          snprintf(filename, 512, "%s/score_%05d.dat", folder, id[0]);
        else
          snprintf(filename, 512, "%s/score.dat", folder);

        file.open(filename, ios::out);
      }

      if(id.size()==3)
      {
        file << id[1] << " " << id[2] << " ";
      }
      else
        file << i << " ";
      file << std::setprecision(9) << std::fixed << _scores[i] << endl;
    }
    if(file.is_open()) file.close();
    // -------------------------------------------------

    // ------------------- Score3D ---------------------
    if(_ids[0].size()==3)
    {
      snprintf(filename, 512, "%s/score3D.dat", folder);
      file.open(filename, ios::out);
      for(unsigned int i=0; i<_scores.size(); i++)
      {
        vector<unsigned int> id = _ids[i];

        for(unsigned int j=0; j<id.size(); j++)
          file << id[j] << " ";
        file << std::setprecision(9) << std::fixed << _scores[i] << endl;
      }
      if(file.is_open()) file.close();

      snprintf(filename, 512, "%s/score3D.gpi", folder);
      file.open(filename, ios::out);
      file << "clear" << endl;
      file << "reset" << endl;
      file << "set hidden3d" << endl;
      file << "set dgrid3d 50,50 qnorm 2" << endl;
      file << "splot \"./score3D.dat\" u 2:3:4 w l" << endl;

      if(file.is_open()) file.close();
    }
    // -------------------------------------------------

    // -----------------Trace script -------------------
    if(_dim==2)
    {
      snprintf(filename, 512, "%s/trace.gpi", folder);
      file.open(filename, ios::out);
      file << "clear" << endl;
      file << "reset" << endl << "set terminal png" << endl;
      file << "set isosample 40" << endl;
      file << "set xrange [" << minCoord[0] << ":" << maxCoord[0] << "]" << endl;
      file << "set yrange [" << minCoord[1] << ":" << maxCoord[1] << "]" << endl;

      for(unsigned int i=0; i<_scenes.size(); i++)
      {
        vector<unsigned int> id = _ids[i];
        char buf[64];
        file << "set output \"trace_";
        if(id.size()==3)
          sprintf(buf, "%05d_%05d_%05d", id[0], id[1], id[2]);
        else
          sprintf(buf, "%05d", i);
        file << buf;
        file << ".png\"" << endl;
        file << "plot \"./model.dat\" u 1:2 w p pt 18 ps 1 t \"model\"";
        file << ", \"./scene.dat\" u 1:2 w p pt 19 ps 1 t \"scene\"";
        file << ", \"./scene_" << buf << ".dat\" u 1:2 w p pt 19 t \"scene(" << buf << ")\"";
        file << ", \"./pairs_" << buf << ".dat\" u 1:2 w p pt 20 ps 2 t \"pairs (model)\"";
        file << ", \"./pairs_" << buf << ".dat\" u 3:4 w p pt 16 ps 2 t \"pairs (scene)\"";
        file << endl;
      }

      file << "set autoscale" << endl;
      vector<unsigned int> id = _ids[0];
      if(id.size()==3)
      {
        for(unsigned int i=0; i<_scores.size(); i++)
        {
          vector<unsigned int> id = _ids[i];
          bool newfile = false;
          if(i==0)
          {
            newfile = true;
          }
          else if(id.size()==3)
          {
            newfile = (id[0] != _ids[i-1][0]);
          }

          if(newfile)
          {
            char buf[64];
            file << "set output \"score_";
            snprintf(buf, 64, "%05d", id[0]);
            file << buf << ".png\"" << endl;
            file << "plot \"./score_" << buf << ".dat\" u 2:3 w lp" << endl;
          }
        }
      }
      else
      {
        file << "set output \"score.png\"" << endl;
        file << "plot \"./score.dat\" u 1:2 w lp" << endl;
      }
      file.close();
      // -------------------------------------------------

      LOGMSG(DBG_DEBUG, "Trace serialized, execute animation script for gnuplot visualization");
    }
    else
      LOGMSG(DBG_DEBUG, "Trace serialized, animation script not available for 3D data");
  }
  else
  {
    LOGMSG(DBG_ERROR, "Delete existing directory or choose a different name for trace recording");
  }
}

}


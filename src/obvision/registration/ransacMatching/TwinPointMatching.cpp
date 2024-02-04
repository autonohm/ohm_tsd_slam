#include "TwinPointMatching.h"

#include <math.h>
#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"

using namespace std;

namespace obvious
{

TwinPointMatching::TwinPointMatching(unsigned int trials, double epsThresh, unsigned int sizeControlSet) : RandomMatching(sizeControlSet)
{
  _epsSqr = epsThresh * epsThresh;
  _trials = trials;
  _model = NULL;
  _index = NULL;
  _trace = NULL;
}

TwinPointMatching::~TwinPointMatching()
{
  if(_model)
  {
    delete _model;
    _model = NULL;
    delete _index;
    _index = NULL;
  }
}

void TwinPointMatching::initKDTree(obvious::Matrix* M, vector<unsigned int> valid)
{
  // Build FLANN tree for fast access to nearest neighbors
  unsigned int cols = M->getCols();
  unsigned int rows = valid.size();
  double** mData;
  obvious::System<double>::allocate(rows, cols, mData);
  for(unsigned int r = 0; r < rows; r++)
  {
    mData[r][0] = (*M)(valid[r], 0);
    mData[r][1] = (*M)(valid[r], 1);
  }
  if(_model)
  {
    delete _model;
    _model = NULL;
    delete _index;
    _index = NULL;
  }
  _model = new flann::Matrix<double>(&mData[0][0], rows, 2);
  flann::KDTreeSingleIndexParams p;
  _index = new flann::Index<flann::L2<double> >(*_model, p);
  _index->buildIndex();
  obvious::System<double>::deallocate(mData);
}

double** TwinPointMatching::createLutIntraDistance(obvious::Matrix* M, const bool* mask, const int maxDist)
{
  int points = (int)M->getRows();
  double** dists;
  obvious::System<double>::allocate(points, points, dists);
  for(int i = 0; i < points; i++)
  {
    if(mask[i])
    {
      int jmax = min(i+maxDist, points);
      for(int j = i+1; j < jmax; j++)
      {
        if(mask[j])
        {
          double dx = (*M)(j, 0) - (*M)(i, 0);
          double dy = (*M)(j, 1) - (*M)(i, 1);
          dists[i][j] = dx * dx + dy * dy;
        }
        else
        {
          dists[i][j] = NAN;
        }
      }
    }
  }

  return dists;
}

#define MIN_VALID_POINTS 10
obvious::Matrix TwinPointMatching::match(obvious::Matrix* M, const bool* maskM, obvious::Matrix* S,  const bool* maskS, double phiMax, const double transMax, const double resolution)
{
  obvious::Matrix TBest(3, 3);
  TBest.setIdentity();

  const unsigned int pointsInS = S->getRows();
  const unsigned int pointsInM = M->getRows();

  const double transMaxSqr = transMax*transMax;

  if(pointsInM != pointsInS)
  {
    LOGMSG(DBG_ERROR, "Model and scene need to be of same size, size of M: " << pointsInM << ", size of S: " << pointsInS);
    return TBest;
  }

  vector<unsigned int> idxMValid = extractSamples(M, maskM, 0);
  vector<unsigned int> idxSValid = extractSamples(S, maskS, 0);
  vector<unsigned int> mapMRawToValid;
  unsigned int cnt = 0;
  for(unsigned int i=0; i<M->getRows(); i++)
  {
    if(maskM[i])
      mapMRawToValid.push_back(cnt++);
    else
      mapMRawToValid.push_back(-1);
  }

  if(idxMValid.size() < MIN_VALID_POINTS || idxSValid.size() < MIN_VALID_POINTS)
  {
    LOGMSG(DBG_ERROR, "Model or scene contain too less valid points, valid in M: " << idxMValid.size() << ", valid in S: " << idxSValid.size());
    return TBest;
  }

  if(_trace)
  {
    _trace->reset();
    _trace->setModel(M, idxMValid);
    _trace->setScene(S, idxSValid);
  }

  // Calculate search "radius", i.e., maximum difference in polar indices because of rotation
  phiMax = min(phiMax, M_PI * 0.5);
  int span;
  if(resolution > 1e-6)
  {
    span = (int)(phiMax / resolution);
    if(span > (int)pointsInM) span = (int)pointsInM;
  }
  else
  {
    LOGMSG(DBG_ERROR, "Resolution not properly set: resolution = " << resolution);
    return TBest;
  }

  initKDTree(M, idxMValid);

  vector<unsigned int> idxControl;  //represents the indices of points used for Control in S.
  obvious::Matrix* Control = pickControlSet(S, idxSValid, idxControl);

  LOGMSG(DBG_DEBUG, "Valid points in scene: " << idxSValid.size() << ", Control set: " << Control->getCols());

  const unsigned int maxDist2ndSample = 10.0 / rad2deg(resolution); //(M_PI/6.0) / resolution;//(unsigned int) phiMax/resolution; //(M_PI/6.0) / resolution; //0.5 * phiMax / resolution;
  const unsigned int minDist2ndSample = 3.0 / rad2deg(resolution);
  assert(minDist2ndSample >= 1);

  double** SDists = createLutIntraDistance(S, maskS, maxDist2ndSample);


  // -----------------------------------------------------
  // Perform RANSAC scheme as follows:
  // 1) pick random point from center part of model
  // 2) pick 2nd random point from model (right of first point)
  // 3) assign scene points to 1st point
  // 4) search 2nd point in scene with similar distance (1st and 2nd point in model)
  // 5) calculate transformation
  // 6) rate control set, i.e., determine consensus
  unsigned int cntBest     = 0;
  double errBest           = 1e12;
  double cntRateBest       = 0;

#ifndef DEBUG
if (_trace)
{
  omp_set_num_threads(1);
}
#endif

#pragma omp parallel
{
  //cout<<"Number of Threads: "<< omp_get_num_threads()<<endl;
  #pragma omp for
  for(unsigned int trial = 0; trial < _trials; trial++)
  {
    //bool foundBetterMatch = false;
    // pick randomly one point in model set
    const unsigned int randIdx      = rand() % ((idxMValid.size()-1)-minDist2ndSample);
    // ... and leave at least n points for 2nd choice
    const unsigned int remainingIdx = min((unsigned int)(idxMValid.size()-randIdx-1), maxDist2ndSample);
    // Index for first model sample
    const unsigned int idx1         = idxMValid[randIdx];
    // Second model sample: Random on right side != i
    const unsigned int idx2         = idxMValid[randIdx + (rand()%(remainingIdx-minDist2ndSample)) + minDist2ndSample];

    //LOGMSG(DBG_DEBUG, "Candidates: " << i << ", " << i2);

    // Vector between model points (for determining orientation)
    double vM[2];
    vM[0] = (*M)(idx2, 0) - (*M)(idx1, 0);
    vM[1] = (*M)(idx2, 1) - (*M)(idx1, 1);

    // Centroid of model (for determining translation)
    double cM[2];
    cM[0] = ((*M)(idx1, 0) + (*M)(idx2, 0)) * 0.5;
    cM[1] = ((*M)(idx1, 1) + (*M)(idx2, 1)) * 0.5;

    const double distM = vM[0] * vM[0] + vM[1] * vM[1];

    // coordinates of point in S with similar intra-distance
    double sceneSimilar[2];

    // leftmost scene point belonging to query point idx1
    const unsigned int iMin = max((int) idx1-span, 0);
    // rightmost scene point belonging to query point idx1
    const unsigned int iMax = min(idx1+span, pointsInS);

    //LOGMSG(DBG_DEBUG, "idx1: " << idx1 << " idx2: " << idx2 << ", search range: " << iMin << " " << iMax);
    for(unsigned int i = iMin; i < iMax; i++)
    {
      if(!maskS[i]) continue;

      // Find scene sample with similar distance
      unsigned int iMinDist = 0;
      unsigned int i2max    = min(pointsInS, i+maxDist2ndSample);
      double distSMin       = 1e12;
      for(unsigned int i2 = i + minDist2ndSample; i2 < i2max; i2++)
      {
        if(!maskS[i2])continue;

        double distS = SDists[i][i2];
        assert(distS != NAN);
        double distEps = fabs(distS - distM);
        if(distEps < distSMin)
        {
          distSMin = distEps;
          iMinDist = i2;
        }
      }

      if(iMinDist==0) continue;
      assert(iMinDist > i);

      if(distSMin < _epsSqr)
      {
        sceneSimilar[0] = (*S)(iMinDist, 0);
        sceneSimilar[1] = (*S)(iMinDist, 1);
      }
      else
        continue;

      // Align scans
      double vS[2];
      vS[0] = sceneSimilar[0] - (*S)(i, 0);
      vS[1] = sceneSimilar[1] - (*S)(i, 1);

      // Calculate polar angle
      double phiM = atan2(vM[1], vM[0]);
      double phiS = atan2(vS[1], vS[0]);

      // Solution for rotational part
      double phi = phiM - phiS;

      if(fabs(phi) < phiMax)
      {
        //We can cut off the outer parts of the scene/model as
        //the rotations tells how much information is not shared by the scans
        int clippedBeams = (int) (phi / resolution);

        // Centroid of scene
        double cS[2];
        cS[0] = (sceneSimilar[0] + (*S)(i, 0)) * 0.5;
        cS[1] = (sceneSimilar[1] + (*S)(i, 1)) * 0.5;

        obvious::Matrix T = obvious::MatrixFactory::TransformationMatrix33(phi, 0, 0);

        // Calculate translation
        T(0, 2) = cM[0] - (T(0, 0) * cS[0] + T(0, 1) * cS[1]);
        T(1, 2) = cM[1] - (T(1, 0) * cS[0] + T(1, 1) * cS[1]);

        if( (T(0, 2)*T(0, 2) + T(1, 2)*T(1, 2)) > transMaxSqr)
        {
          //LOGMSG(DBG_DEBUG, "Translation is too big!");
          continue;
        }

        obvious::Matrix STemp = T * (*Control);

        // Determine how many nearest neighbors (model <-> scene) are close enough
        double q[2];
        unsigned int cntMatch = 0;
        flann::Matrix<int> indices(new int[1], 1, 1);
        flann::Matrix<double> dists(new double[1], 1, 1);
        double err = 0;

        unsigned int clippedPoints = 0;
        for(unsigned int s = 0; s < STemp.getCols(); s++)
        {
          /* Clip control points according to phi:
           *------------------------------------------
           * Cases:
           * for positive clipped points -> Scene is rotated left
           *    Scene uses: [0; size-clippedPoints] -> cut of left side
           *    Model uses: [0+clippedPoints; size] -> cut of right side
           * for negative clipped points. -> Scene is rotate right
           *    Scene uses: [0-clippedPoints; size] -> cut of right
           *    Model uses: [0; scene + clippedPoints] -> cut of left
           */

          if( idxControl[s] < (unsigned int) max(0, -clippedBeams) || idxControl[s] > min(pointsInS, pointsInS-clippedBeams) )
          {
            clippedPoints++;
            continue; // Cut of Scene Points, points that won't have a corresponding point due to rotation are ignored for the metric
          }

          //Find nearest neighbor for control point
          q[0] = STemp(0, s);
          q[1] = STemp(1, s);
          flann::Matrix<double> query(q, 1, 2);
          flann::SearchParams p(-1, 0.0);
          _index->knnSearch(query, indices, dists, 1, p);

          //Check if model point is not clipped
          unsigned int rawIdx = idxMValid[ indices[0][0] ]; //raw index of closest model point
          if(rawIdx < (unsigned int) max(0, clippedBeams) || rawIdx > min(pointsInS, pointsInS+clippedBeams))
          {
            clippedPoints++;
            continue; //Cut off point correspondences to Model points that don't have a reasonable corresponding point due to rotation.
          }

          err += dists[0][0];
          if(dists[0][0] < _epsSqr)
          {
            //err += sqrt(dists[0][0]);
            cntMatch++;
          }
        }
        err = sqrt(err);

        delete[] indices.ptr();
        delete[] dists.ptr();

        if(cntMatch == 0)
          continue;

        // Relative MatchCnt Score
        unsigned int maxMatchCnt = (STemp.getCols() - clippedPoints);
        double cntRate = (double)cntMatch / (double) maxMatchCnt;
        //double cntStepSize = 1.0 / STemp.getCols();
        double equalThres = 1e-5;//cntStepSize;// 1e-5;

#pragma omp critical
{
        bool rateCondition = ((cntRate - cntRateBest) > equalThres) && (cntMatch > cntBest);
        bool errorCondition = fabs( (cntRate-cntRateBest) < equalThres ) && (cntMatch == cntBest) && err < errBest;
        bool goodMatch = rateCondition || errorCondition;

        if(goodMatch)
        {
          errBest = err;
          cntBest = cntMatch;
          cntRateBest = cntRate;
          TBest = T;
        }
}
        if(_trace)
        {
          vector<unsigned int> idxM;
          idxM.push_back(idx1);
          idxM.push_back(idx2);
          vector<unsigned int> idxS;
          idxS.push_back(i);
          idxS.push_back(iMinDist);
          _trace->addAssignment(M, idxM, S, idxS, &STemp, err, trial);
        }
      }  // if(fabs(phi) < _phiMax)
    }  // for all points in scene
  }  // for trials
} //OpenMP END

  LOGMSG(DBG_DEBUG, "Matching result - cnt(best): " << cntBest << ", err(best): " << errBest);

  obvious::System<double>::deallocate(SDists);

  delete Control;

  return TBest;
}

}

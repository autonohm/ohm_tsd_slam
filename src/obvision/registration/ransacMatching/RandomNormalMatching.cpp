#include "RandomNormalMatching.h"

#include <math.h>
#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"
#include <limits>

#include "obcore/base/Timer.h"

using namespace std;

namespace obvious
{

#define NORMALCONSENSUS 1
#define USEKNN 1

RandomNormalMatching::RandomNormalMatching(unsigned int trials, double epsThresh, unsigned int sizeControlSet) : RandomMatching(sizeControlSet)
{
  _scaleDistance    = 1.0/(epsThresh * epsThresh);
  _scaleOrientation = 0.33;
  _trials           = trials;
  _model            = NULL;
  _index            = NULL;
  _trace            = NULL;
  _pcaSearchRange   = 10;
}

RandomNormalMatching::~RandomNormalMatching()
{
  if(_model)
  {
    delete _model;
    _model = NULL;
    delete _index;
    _index = NULL;
  }
}

void RandomNormalMatching::initKDTree(obvious::Matrix* M, vector<unsigned int> idxValid)
{
  // Build FLANN tree for fast access to nearest neighbors
  unsigned int cols = M->getCols();
  unsigned int rows = idxValid.size();
  double** mData;
  obvious::System<double>::allocate(rows, cols, mData);
  for(unsigned int r = 0; r < rows; r++)
  {
    mData[r][0] = (*M)(idxValid[r], 0);
    mData[r][1] = (*M)(idxValid[r], 1);
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

obvious::Matrix RandomNormalMatching::match(obvious::Matrix* M,
    const bool* maskM,
    obvious::Matrix* NM,
    obvious::Matrix* S,
    const bool* maskS,
    double phiMax,
    const double transMax,
    const double resolution)
{
  obvious::Matrix TBest(3, 3);
  TBest.setIdentity();

  const int pointsInM = M->getRows();
  const int pointsInS = S->getRows();

  if(pointsInM != pointsInS)
  {
    LOGMSG(DBG_ERROR, "Model and scene need to be of same size, size of M: " << pointsInM << ", size of S: " << pointsInS);
    return TBest;
  }

  if(pointsInM < 3)
  {
    LOGMSG(DBG_ERROR, "Model and scene contain too less points, size of M: " << pointsInM << ", size of S: " << pointsInS);
    return TBest;
  }

  // ----------------- Model ------------------
  obvious::Matrix* NMpca = new Matrix(pointsInM, 2); // Normals for model
  double* phiM           = new double[pointsInM];    // Orientation of model points
  bool* maskMpca         = new bool[pointsInM];      // Validity mask of model points

  memcpy(maskMpca, maskM, pointsInM*sizeof(bool));

  if(NM)
  {
    calcPhi(NM, maskM, phiM);
  }
  else // if normals are not supplied
  {
    calcNormals(M, NMpca, maskM, maskMpca, _pcaSearchRange/2);
    calcPhi(NMpca, maskMpca, phiM);
  }
  vector<unsigned int> idxMValid = extractSamples(M, maskMpca, _pcaSearchRange/2);

#if USEKNN
  initKDTree(M, idxMValid);
#endif
  // -------------------------------------------


  // ----------------- Scene -------------------
  obvious::Matrix* NSpca = new Matrix(pointsInS, 2); // Normals for scene
  double* phiS           = new double[pointsInS];    // Orientation of scene points
  bool* maskSpca         = new bool[pointsInS];      // Validity mask of scene points
  memcpy(maskSpca, maskS, pointsInS*sizeof(bool));

  // Determine number of valid samples in local scene neighborhood
  // only from these points a valid orientation is computable
  unsigned int validPoints = 0;
  for(int i=0; i<pointsInS; i++)
    if(maskSpca[i]) validPoints++;

  // Probability of point masking
  double probability = 180.0/(double)validPoints;
  if(probability<0.99)
    subsampleMask(maskSpca, pointsInS, probability);

  calcNormals(S, NSpca, maskS, maskSpca, _pcaSearchRange/2);
  calcPhi(NSpca, maskSpca, phiS);

  vector<unsigned int> idxSValid = extractSamples(S, maskSpca, _pcaSearchRange/2);
  // -------------------------------------------


  // --------------- Control set ---------------
  vector<unsigned int> idxControl;  //represents the indices of points used for Control in S.
  obvious::Matrix* Control = pickControlSet(S, idxSValid, idxControl);
  obvious::Matrix* NControl = new obvious::Matrix(idxControl.size(), 2);
  for(unsigned int i=0; i<Control->getCols(); i++)
  {
    (*NControl)(i, 0) = (*NSpca)(idxControl[i], 0);
    (*NControl)(i, 1) = (*NSpca)(idxControl[i], 1);
  }
  unsigned int pointsInC = Control->getCols();
  unsigned int cntMatchThresh = pointsInC / 3; // TODO: Determine meaningful parameter
  double* phiControl = new double[pointsInC];  // Orientation of control points
  calcPhi(NControl, NULL, phiControl);
  // -------------------------------------------//


  // Determine frustum, i.e., direction of leftmost and rightmost model point
  double thetaBoundMin = atan2((*M)(idxMValid.front(),1), (*M)(idxMValid.front(),0)); // real bounding
  double thetaBoundMax = atan2((*M)(idxMValid.back(),1),  (*M)(idxMValid.back(),0));  // real bounding

  LOGMSG(DBG_DEBUG, "Valid points in scene: " << idxSValid.size() << ", valid points in model: " << idxMValid.size() << ", Control set: " << Control->getCols());
  LOGMSG(DBG_DEBUG, "Model phi min:: " << rad2deg(thetaBoundMin) << ", Model phi max: " << rad2deg(thetaBoundMax));

  if(idxSValid.size() < 3)
  {
    LOGMSG(DBG_ERROR, "Too less valid points in scene, matchable size: " << idxSValid.size());
    return TBest;
  }

  if(idxMValid.size() < 3)
  {
    LOGMSG(DBG_ERROR, "Too less valid points in model, matchable size: " << idxMValid.size());
    return TBest;
  }

  // Check for maximum meaningful trials
  unsigned int trials = _trials;
  if(idxMValid.size()<_trials)
    trials = idxMValid.size();

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
    span = floor(phiMax / resolution);
    if(span > (int)pointsInM) span = (int)pointsInM;
  }
  else
  {
    LOGMSG(DBG_ERROR, "Resolution not properly set: resolution = " << resolution);
    return TBest;
  }

  srand (time(NULL));

  double       bestRatio = 0.0;
  unsigned int bestCnt   = 0;
  double       bestErr   = 1e12;

#ifndef DEBUG
  // trace is only possible for single threaded execution
  if(_trace)
  {
    omp_set_num_threads(1);
    LOGMSG(DBG_WARN, "Configured single-threaded execution due to application of trace module");
  }
#endif

  //Timer t;
  //t.start();
  vector<unsigned int> idxTrials = idxMValid;
#pragma omp parallel
  {
    bool* maskControl        = new bool[pointsInC];
    double* thetaControl     = new double[pointsInC];

#pragma omp for
    for(unsigned int trial = 0; trial < trials; trial++)
    {

      int idx;
#pragma omp critical
      {
        const int randIdx = rand() % (idxTrials.size());
        idx               = idxTrials[randIdx];

        // remove chosen element to avoid picking same index a second time
        idxTrials.erase(idxTrials.begin() + randIdx);
      }

      // leftmost scene point
      const int iMin = max(idx-span, _pcaSearchRange/2);
      // rightmost scene point
      const int iMax = min(idx+span, pointsInS-_pcaSearchRange/2);


      for(int i=iMin; i<iMax; i++)
      {
        if(maskSpca[i])
        {

          double phi              = phiM[idx] - phiS[i];
          if(phi>M_PI)       phi -= 2.0*M_PI;
          else if(phi<-M_PI) phi += 2.0*M_PI;

          if(fabs(phi) < phiMax)
          {
            obvious::Matrix T = obvious::MatrixFactory::TransformationMatrix33(phi, 0, 0);

            // Calculate translation
            const double sx = (*S)(i,0);
            const double sy = (*S)(i,1);
            T(0, 2) = (*M)(idx,0) - (T(0, 0) * sx + T(0, 1) * sy);
            T(1, 2) = (*M)(idx,1) - (T(1, 0) * sx + T(1, 1) * sy);

            // Transform control set
            obvious::Matrix STemp = T * (*Control);
            unsigned int pointsInControl = STemp.getCols();

            // Determine number of control points in field of view
            unsigned int maxCntMatch = 0;
            for(unsigned int j=0; j<pointsInControl; j++)
            {
              thetaControl[j] = atan2(STemp(1, j), STemp(0, j));
              if(thetaControl[j]>thetaBoundMax || thetaControl[j]<thetaBoundMin)
              {
                maskControl[j] = false;
              }
              else
              {
                maskControl[j] = true;
                maxCntMatch++;
              }
            }

            // Determine how many nearest neighbors (model <-> scene) are close enough
            unsigned int cntMatch = 0;
            flann::Matrix<int> indices(new int[1], 1, 1);
            flann::Matrix<double> dists(new double[1], 1, 1);
            double errSum = 0;
            //double scoreSum = 0.0;

            for(unsigned int s = 0; s < pointsInControl; s++)
            {
              // clip points outside of model frustum
              if(maskControl[s])
              {

#if USEKNN
                // find nearest neighbor of control point
                double q[2];
                q[0] = STemp(0, s);
                q[1] = STemp(1, s);
                flann::Matrix<double> query(q, 1, 2);
                flann::SearchParams p(-1, 0.0);
                _index->knnSearch(query, indices, dists, 1, p);
                const int idxQuery = idxMValid[indices[0][0]];
                double distConsensus   = dists[0][0];
#else
                // speeded-up NN search through back projection
                const int idxQuery = round((thetaControl[s]-thetaMin) / resolution);

                if(!maskM[idxQuery]) continue;

                double distX = (*M)(idxQuery, 0) - STemp(0, s);
                double distY = (*M)(idxQuery, 1) - STemp(1, s);
                double distConsensus  = distX*distX + distY*distY;
#endif

#if NORMALCONSENSUS
                // Experimental idea: rate matching results additionally with normal consensus
                // consensus score is in range [0, 1] -> perfect match = 0
                double normalConsensus = (1.0 - cos(phiM[idxQuery] - phiControl[s] - phi))/2.0;
                // Normalized error (weight distance and normal consensus)
                double err = distConsensus*_scaleDistance + normalConsensus*_scaleOrientation;
#else
                double err = distConsensus*_scaleDistance;
#endif

                errSum += err;
                if(err<1.0)
                  cntMatch++;
              }
            }

            delete[] indices.ptr();
            delete[] dists.ptr();

            if(cntMatch <= cntMatchThresh)
              continue;

            // Experimental rating
            double ratio = (double)cntMatch / (double) maxCntMatch;

#pragma omp critical
            {
              // Rating from Markus Kuehn
              double equalThres = 1e-5;
              bool rateCondition = ((ratio-bestRatio) > equalThres) && (cntMatch > bestCnt);
              bool similarityCondition = fabs( (ratio-bestRatio) < equalThres ) && (cntMatch == bestCnt) && errSum < bestErr;
              bool goodMatch = rateCondition ||similarityCondition;

              if(goodMatch)
              {
                bestRatio = ratio;
                bestCnt = cntMatch;
                bestErr = errSum;
                TBest = T;
              }

            }

            if(_trace)
            {
              //trace is only possible for single threaded execution
              vector<unsigned int> idxM;
              idxM.push_back(idx);
              vector<unsigned int> idxS;
              idxS.push_back(i);
              _trace->addAssignment(M, idxM, S, idxS, &STemp, errSum, trial);
            }

          }// if phiMax
        } // if maskS
      } // for i
    } // for trials

    delete [] maskControl;

  } // OMP

  //cout << "elapsed: " << t.elapsed() << endl;
  //t.reset();

  delete NMpca;
  delete NSpca;
  delete [] phiM;
  delete [] phiS;
  delete [] phiControl;
  delete [] maskMpca;
  delete [] maskSpca;

  delete Control;

  return TBest;
}

}

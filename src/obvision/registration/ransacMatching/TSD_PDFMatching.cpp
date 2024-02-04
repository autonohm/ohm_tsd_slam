#include "TSD_PDFMatching.h"

namespace obvious
{

TSD_PDFMatching::TSD_PDFMatching( TsdGrid& grid,
                                  unsigned int trials,
                                  double epsThresh,
                                  unsigned int sizeControlSet,
                                  double zrand) : RandomMatching(sizeControlSet), _grid(grid)
{

  _scaleDistance       = 1.0 / (epsThresh * epsThresh);
  _scaleOrientation    = 0.33;

  _trace               = NULL;
  _pcaSearchRange      = 10;

  _trials              = trials;

  // additional random probability to model uncertainty and avoid prob of 0.0
  _zrand               = zrand;

}

TSD_PDFMatching::~TSD_PDFMatching()
{

}

obvious::Matrix TSD_PDFMatching::match( obvious::Matrix TSensor,
                                        obvious::Matrix* M,
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
  obvious::Matrix* NMpca = new Matrix(pointsInM, 2);  // Normals for model
  double* phiM = new double[pointsInM];    // Orientation of model points
  bool* maskMpca = new bool[pointsInM];      // Validity mask of model points

  memcpy(maskMpca, maskM, pointsInM * sizeof(bool));

  if(NM)
  {
    calcPhi(NM, maskM, phiM);
  }
  else  // if normals are not supplied
  {
    calcNormals(M, NMpca, maskM, maskMpca, _pcaSearchRange/2);
    calcPhi(NMpca, maskMpca, phiM);
  }
  vector<unsigned int> idxMValid = extractSamples(M, maskMpca, _pcaSearchRange / 2);

  // -------------------------------------------

  // ----------------- Scene -------------------
  obvious::Matrix* NSpca = new Matrix(pointsInS, 2);  // Normals for scene
  double* phiS = new double[pointsInS];    // Orientation of scene points
  bool* maskSpca = new bool[pointsInS];      // Validity mask of scene points
  memcpy(maskSpca, maskS, pointsInS * sizeof(bool));

  // Determine number of valid samples in local scene neighborhood
  // only from these points a valid orientation is computable
  unsigned int validPoints = 0;
  for(int i = 0; i < pointsInS; i++)
    if(maskSpca[i])
      validPoints++;

  // Probability of point masking
  double probability = 180.0 / (double)validPoints;
  if(probability < 0.99)
    subsampleMask(maskSpca, pointsInS, probability);

  calcNormals(S, NSpca, maskS, maskSpca, _pcaSearchRange/2);
  calcPhi(NSpca, maskSpca, phiS);

  vector<unsigned int> idxSValid = extractSamples(S, maskSpca, _pcaSearchRange / 2);
  // -------------------------------------------

  // --------------- Control set ---------------
  vector<unsigned int> idxControl;  //represents the indices of points used for Control in S.
  obvious::Matrix* Control = pickControlSet(S, idxSValid, idxControl);
  obvious::Matrix* NControl = new obvious::Matrix(idxControl.size(), 2);
  for(unsigned int i = 0; i < Control->getCols(); i++)
  {
    (*NControl)(i, 0) = (*NSpca)(idxControl[i], 0);
    (*NControl)(i, 1) = (*NSpca)(idxControl[i], 1);
  }
  unsigned int pointsInC = Control->getCols();
  double* phiControl = new double[pointsInC];  // Orientation of control points
  calcPhi(NControl, NULL, phiControl);
  // -------------------------------------------//

  // Determine frustum, i.e., direction of leftmost and rightmost model point
  //double thetaMin = -((double)pointsInM - 1.0) / 2.0 * resolution;  // theoretical bounding
  double thetaBoundMin = atan2((*M)(idxMValid.front(), 1), (*M)(idxMValid.front(), 0));  // real bounding
  double thetaBoundMax = atan2((*M)(idxMValid.back(), 1), (*M)(idxMValid.back(), 0));  // real bounding

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
  if(idxMValid.size() < _trials)
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
    if(span > (int)pointsInM)
      span = (int)pointsInM;
  }
  else
  {
    LOGMSG(DBG_ERROR, "Resolution not properly set: resolution = " << resolution);
    return TBest;
  }

  srand(time(NULL));

  double bestProb = 0.0;

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

  bool* maskControl = new bool[pointsInC];

#pragma omp parallel for
  for(unsigned int trial = 0; trial < trials; trial++)
  {

    int idx;
#pragma omp critical
    {
      const int randIdx = rand() % (idxTrials.size());
      idx = idxTrials[randIdx];

      // remove chosen element to avoid picking same index a second time
      idxTrials.erase(idxTrials.begin() + randIdx);
    }

    // leftmost scene point
    const int iMin = max(idx - span, _pcaSearchRange / 2);
    // rightmost scene point
    const int iMax = min(idx + span, pointsInS - _pcaSearchRange / 2);

    for(int i = iMin; i < iMax; i++)
    {

      if(maskSpca[i])
      {
        double phi = phiM[idx] - phiS[i];
        if(phi > M_PI)
          phi -= 2.0 * M_PI;
        else if(phi < -M_PI)
          phi += 2.0 * M_PI;

        if(fabs(phi) < phiMax)
        {
          obvious::Matrix T = obvious::MatrixFactory::TransformationMatrix33(phi, 0, 0);

          // Calculate translation
          const double sx = (*S)(i, 0);
          const double sy = (*S)(i, 1);
          T(0, 2) = (*M)(idx, 0) - (T(0, 0) * sx + T(0, 1) * sy);
          T(1, 2) = (*M)(idx, 1) - (T(1, 0) * sx + T(1, 1) * sy);

          obvious::Matrix TMap = TSensor * T;

          // Transform control set
          obvious::Matrix STemp = TMap * (*Control);
          unsigned int pointsInControl = STemp.getCols();

          // Rating Daniel Ammon & Tobias Fink
          std::vector<double> probOfAllScans;  // vector for probabilities of single scans in one measurement
          double probOfActualMeasurement = 1.0;

          for (unsigned int s = 0; s < pointsInControl; s++)	// whole control set
          {
            obfloat coord[2];
            coord[0] = STemp(0, s);
            coord[1] = STemp(1, s);

            // todo: magic numbers 0.05 / 0.95
            obfloat tsd;
            if( !_grid.interpolateBilinear(coord, &tsd) )
            {
              // rating function: clipped probability --> avoid prob of 0
              // multiply all probabilities for probability of whole scan
              probOfActualMeasurement *= (1.0 - (1.0 - _zrand) * fabs(tsd));
            }
            else
            {
              probOfActualMeasurement *= _zrand;
            }
          }  // whole control set

#pragma omp critical
{
          // update T and bestProb if better than last iteration
          if(probOfActualMeasurement > bestProb)
          {
            TBest = T;
            bestProb = probOfActualMeasurement;

#ifndef DEBUG
            if(_trace)
            {
              //trace is only possible for single threaded execution
              vector<unsigned int> idxM;
              idxM.push_back(idx);
              vector<unsigned int> idxS;
              idxS.push_back(i);
              _trace->addAssignment(M, idxM, S, idxS, &STemp, 10 * probOfActualMeasurement, trial);
            }
#endif
          }
}
        } // if(fabs(phi) < phiMax)
      } // if(maskSpca[i])
    }  // for i
  }  // for trials

  //cout << "elapsed: " << t.elapsed() << endl;
  //t.reset();

  delete [] maskControl;
  delete    NMpca;
  delete    NSpca;
  delete [] phiM;
  delete [] phiS;
  delete [] phiControl;
  delete [] maskMpca;
  delete [] maskSpca;

  delete Control;

  return TBest;
}

} /* namespace obvious */

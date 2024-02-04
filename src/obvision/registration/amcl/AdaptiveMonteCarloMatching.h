#ifndef ADAPTIVEMONTECARLOMATCHING_H_
#define ADAPTIVEMONTECARLOMATCHING_H_

#include <flann/flann.hpp>
#include "obcore/math/linalg/linalg.h"
#include "obvision/registration/Trace.h"

namespace obvious
{

/**
 * @class AdaptiveMonteCarloMatching
 * @brief Matching algorithm with AMCL alignment
 * @author Daniel Ammon, Tobias Fink and Stefan May
 **/
class AdaptiveMonteCarloMatching
{
public:
  /**
   * Constructor
   * @param
   */
  AdaptiveMonteCarloMatching();

  /**
   * Destructor
   */
  virtual ~AdaptiveMonteCarloMatching();

  /**
   * Matching method
   * @param ... map
   * @param S Matrix for scene points
   * @param maskS Mask for matrix S
   */
  obvious::Matrix match(const obvious::Matrix* S,  const bool* maskS);

private:

};

}

#endif /* ADAPTIVEMONTECARLOMATCHING_H_ */

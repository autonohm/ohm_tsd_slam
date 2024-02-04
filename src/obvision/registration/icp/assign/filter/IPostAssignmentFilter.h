#ifndef IPOSTASSIGNMENTFILTER_H
#define IPOSTASSIGNMENTFILTER_H

#include "obvision/registration/icp/assign/assignbase.h"

#include <vector>

using namespace std;

namespace obvious
{

/**
 * @class IPostAssignmentFilter
 * @brief A filter for point pair rejection
 * @author Stefan May
 */
class IPostAssignmentFilter
{
	public:
		/**
		 * Default constructor
		 */
		IPostAssignmentFilter(){_active = true;};

		/**
		 * Destructor
		 */
		virtual ~IPostAssignmentFilter(){};

		virtual void filter(double** model, double** scene,
		                    vector<StrCartesianIndexPair>* pairs,
		                    vector<double>* distancesSqr,
		                    vector<StrCartesianIndexPair>* fpairs,
		                    vector<double>* fdistancesSqr,
		                    vector<unsigned int>* nonPairs) = 0;

		virtual void reset(){};

		virtual void activate(){_active = true;};

		virtual void deactivate(){_active = false;};

	public:

		bool _active;
};

}

#endif /*IPOSTASSIGNMENTFILTER_H*/

#ifndef IPREASSIGNMENTFILTER_H
#define IPREASSIGNMENTFILTER_H

#include <vector>

namespace obvious
{

/**
 * @class IPreAssignmentFilter
 * @brief A pre-filter for point pair assignment
 * @author Stefan May
 */
class IPreAssignmentFilter
{
	public:
		/**
		 * Default constructor
		 */
		IPreAssignmentFilter(){_active=true;};

		/**
		 * Destructor
		 */
		virtual ~IPreAssignmentFilter(){};

		virtual void filter(double** scene, unsigned int size, bool* mask) = 0;

    virtual void activate(){_active = true;};

    virtual void deactivate(){_active = false;};

  public:

    bool _active;

};

}

#endif /*IPREASSIGNMENTFILTER_H*/

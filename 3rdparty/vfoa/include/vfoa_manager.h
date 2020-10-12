#ifndef VFOA_MANAGER_H
#define VFOA_MANAGER_H

/**
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */


#include <list>
#include "types.h"
#include "person.h"
#include "boost/date_time/posix_time/posix_time.hpp"

using namespace std;

///Class to manage and update vfoa's
/**this class updates objects and person lists. the persons have vfoa
 * distributions, the objects don't. The locations for the objects and person
 * are given in camera coordinates, where the camera is located at {0,0,0}
 * and the plane OXY is horizontal. The basis is right handed.
 * The angles for the head pose are given according to the camera.
 * (pan = 0, tilt = 0, roll = 0) means therefore that the person is looking
 * directly at the camera. The angles are given in radians.
 */
class VfoaManager {
  public:

    ///constructor
    /**constructor for the VfoaManager class
     *\param rVerbose Tell the library whether or not to output stuff in an
     * external file.
     *\param rOutputFile File to use for outputting information if verbose is on
     * true
     */
    VfoaManager ();

    ///destructor
    /**destructor for the VfoaManager class
    */
    ~VfoaManager ();

    ///update person list and the target list from target provider
    /**Update the targets using a list of person_info_t and object_info_t.
     * these list must represent the whole scene for which the vfoas must be
     * calculated. If we also want to get the vfoa for the camera, we must add
     * the camera as an object in the object list.
     * \param rOlist list of object_info_t to update. This list must contain al
     * the objects
     * even if there is no new object or if the objects location haven't
     * changed.
     * \param  rPlist list of person_info_t to update. This list must contain
     * all the persons even if there is no new persons or if the persons
     * location
     * haven't changed.
     * \param rTimestamp timestamp of the last update.
     */
    void UpdateTargetList(object_list_t &rObjectInfoList,
        person_list_t &rPersonInfoList,
        const boost::posix_time::ptime &rTimestamp);

    ///get vfoa distributions from person list
    /**return a list of the vfoa distributions for each person
     * \return a list of objects containing a target_id_t and a
     * distribution_list_t. the target_identifier_t allows identification of
     * the person for whom the vfoa distribution is computed
     */
    identified_distribution_list_t GetVfoaList() const;

    ///get vfoa probabilities from person list
    /**return a list of the vfoa probabilities for each person
     * \return a list of objects containing a target_id_t and a
     * probabilities_list_t. the target_identifier_t allows identification of
     * the person for whom the vfoa distribution is computed
     */
    identified_probabilities_list_t GetVfoaProbabilitiesList() const;

    ///get vfoa distributions for a specific person
    /**return a list of the vfoa distributions for each person
     * \param rId id of the person whom the distribution must be returned from
     * \return a list of objects containing a target_id_t and a
     * distribution_list_t. the target_identifier_t allows identification of
     * the person for whom the vfoa distribution is computed
     */
    distribution_t GetVfoaDistributionForId(const target_id_t &rId) const;

    ///get vfoa probabilities for a specific person
    /**return a list of the vfoa probabilities for each person
     * \param rId id of the person whom the probabilities must be returned from
     * \return a list of objects containing a target_id_t and a
     * probabilities_list_t. the target_identifier_t allows identification of
     * the person for whom the vfoa probabilities is computed
     */
    probabilities_t GetVfoaProbabilitiesForId(const target_id_t &rId) const;

    ///return object info list
    /**returns the list of the used objects in the computation of the vfoas
     * \return object_list_t containing these objects
     */
    object_info_t GetObjectInfo(const target_id_t& rId)const;

    ///return person info list
    /**returns the list of the used persons in the computation of the vfoas
     * \return person_list_t containing these persons
     */
    person_info_t GetPersonInfo(const target_id_t& rId)const;

    ///returns a boolean on whether target exists or not
    /**given a specific targets returns wheter it exists or not
     * \return boolean on whether target exists or not
     */
    bool TargetExists(const target_identifier_t &rTarget) const;

  private:

//    list<Person*> mPersonList;

    typedef std::map<target_id_t, Person*> PersonMap;
    typedef std::map<target_id_t, object_info_t> ObjectInfoMap;
    typedef std::map<target_id_t, person_info_t> PersonInfoMap;

    // map from person IDs to person entries
    PersonMap m_Persons;
    // map from object IDs to object info
    ObjectInfoMap m_ObjectInfos;
    // map from person IDs to person info
    PersonInfoMap m_PersonInfos;

    identified_distribution_list_t mVfoaList;

    identified_probabilities_list_t mVfoaProbList;

    VfoaManager(const VfoaManager & rManager);

    VfoaManager& operator=(const VfoaManager & rManager);

    boost::posix_time::ptime mUpdateTimestamp;

    friend std::ostream& operator<<(std::ostream& out, VfoaManager &rManager);
};

#endif

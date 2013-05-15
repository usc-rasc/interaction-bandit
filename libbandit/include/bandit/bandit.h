/***************************************************************************
 *  include/bandit/bandit.h
 *  --------------------
 *
 *  Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of interaction-ros-pkg nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/

#ifndef LIBBANDIT_BANDIT_H_
#define LIBBANDIT_BANDIT_H_

#include <stdexcept>
#include <termios.h>
#include <string>
#include <stdint.h>

#include <boost/function.hpp>

#include <map>

#include <bandit/joint_name.h>
#include <bandit/smartservo.h>

//! A namespace containing the bluesky bandit device driver
namespace bandit
{

// =============================================================================================================================================
//! Throw the given type of exception, with a printf-style message and args
template<class __Exception, class... __Args>
static void BANDIT_EXCEPT( std::string const & msg, __Args&&... args );

// =============================================================================================================================================
// Macro for defining an exception with a given parent ( std::runtime_error should be top parent )
#define DEF_EXCEPTION( name, parent ) \
    class name  : public parent { \
    public: \
    name( const char* msg ) : parent( msg ) {} \
    }

    //! A standard Bandit exception
    DEF_EXCEPTION( BanditException, smartservo::SmartServoException );
#undef DEF_EXCEPTION

// =============================================================================================================================================
// =============================================================================================================================================

class Bandit;

// =============================================================================================================================================
//! Generic bandit joint
/*!
 *  This joint may represent a bandit joint or servo
 */
class Joint
{
private:
    Bandit * bandit_handle_;

public:
    JointName               name;
    int16_t                 mod_id;
    smartservo::WhichJoint  which;
    smartservo::JointType   type;
    int8_t                  direction;
    double                  scale;
    double                  offset;
    double                  min;
    double                  max;

    Joint
    (
        JointName && name = JointName(),
        int16_t const & mod_id_ = -1,
        smartservo::WhichJoint const & which_ = smartservo::NO_JOINT,
        smartservo::JointType const & type_ = smartservo::NO_JOINT_TYPE,
        int8_t const & direction = 0,
        double const & scale_ = 0,
        double const & offset_ = 0,
        double const & min_ = 0,
        double const & max_ = 0
    );

    void setHandle( Bandit * handle );

    double getPos() const;

    template<class... __Args>
    void setPos( __Args&&... args );

    //! Set the PID configuations for this joint
    template<class... __Args>
    void setPIDConfig( __Args&&... args );

    //! Set the joint direction of this joint
    template<class... __Args>
    void setDirection( __Args&&... args );

    //! Set the joint offset of this joint
    template<class... __Args>
    void setOffset( __Args&&... args );
};

// =============================================================================================================================================
// =============================================================================================================================================

// =============================================================================================================================================
//! A minimal driver class for communicating with bandit via serial
/*!
* This is relatively thin wrapper around the lower level Master to make
* joints more intuitive to work with.
*/
class Bandit
{
public:
    //! The type of callback required for state updates
    typedef boost::function<void()> _StateCallback;
    //! A map from joint ids to Joints
    typedef std::map<uint16_t, Joint> _JointsMap;

private:
    //! Instance of Master module to be used to communicate with joints
    smartservo::MasterModule master_;

    //! use software joint limits?
    bool                    joint_limits_;

    //! Map from joint ids to Joints
    _JointsMap joints_map_;

public:
    //! Constructor
    Bandit();

    //! Destructor;
    ~Bandit();

    //! Add a new joint definition
    template<class... __Args>
    void addJoint( __Args&&... args );

    //! value to set if we want to use software limits ( should be true unless calibrating joints )
    void useJointLimits( bool const & use_limits );

    // -----------------------------------------------------------------------------------------------------------------------------------------

    //! Return an iterator to a joint
    _JointsMap::iterator getJoint( JointName const & joint_name );

    _JointsMap::const_iterator getJoint( JointName const & joint_name ) const;

    _JointsMap & getJoints();

    _JointsMap const & getJoints() const;

    //! Return the type of a joint
    smartservo::JointType getJointType( JointName const & joint_name );

    //! Return the name of a joint
    std::string getJointName( JointName const & name );
    //! Return the id of a joint
    int16_t getJointId( JointName const & name );

    //! return the number of joints
    size_t getNumJoints();

    //! Return the minimum position of a joint
    double getJointMin( JointName const & name );

    //! Return the maximum position of a joint
    double getJointMax( JointName const & name );

    //! Get the value of a joint position
    double getJointPos( JointName const & name ) const;

    // -----------------------------------------------------------------------------------------------------------------------------------------

    //! Set the PID configuations for a particular joint.
    template<class... __Args>
    void setJointPIDConfig( JointName const & name, __Args&&... args );

    //! Set the joint direction of a particular joint
    void setJointDirection( JointName const & name, int8_t const & direction );

    //! Set the joint offset of a particular joint
    void setJointOffset( JointName const & name, double const & offset );

    //! Set the joint position of a particular joint
    void setJointPos( JointName const & name, double const & angle );

    // -----------------------------------------------------------------------------------------------------------------------------------------

    //! Open the port
    void openPort( std::string const & port_name );

    //! Close the port
    void closePort();

    //! Callback which will be triggered whenever a state update is received
    void registerStateCB( _StateCallback && callback );

    //! Process a single pending incoming message
    /*!
     *  Processes a single pending incoming serial message from the
     *  master module.  This may invoke one of the registered callback
     *  functions as appropriate.
     *
     *  \param timeout   Timeout in microseconds ( 0 for indefinite )
     */
    void processIO( uint32_t const & timeout );

    void processPackets( bool const & wait = false );

    //! Send the joint positions to bandit
    void sendAllJointPos();

    //! Sync all the joint PID configurations to the master module
    void sendAllPIDConfigs();

    bool checkAllPIDConfigs( bool const & verbose = false );
};

#include <bandit/details/bandit_impl.h>

} // bandit

#endif // LIBBANDIT_BANDIT_H_

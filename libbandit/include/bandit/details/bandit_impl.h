/***************************************************************************
 *  include/bandit/details/bandit_impl.h
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

#ifndef LIBBANDIT_BANDIT_DETAILS_BANDITIMPL_H_
#define LIBBANDIT_BANDIT_DETAILS_BANDITIMPL_H_

// =============================================================================================================================================
template<class __Exception, class... __Args>
static void BANDIT_EXCEPT( std::string const & msg, __Args&&... args )
{
    char buf[256];
    snprintf( buf, 256, ( std::string( "bandit::Bandit::%s: " ) + msg ).c_str(), __FUNCTION__, args... );
    throw __Exception( buf );
}

template<class... __Args>
void Joint::setPos( __Args&&... args )
{
    if( !bandit_handle_ ) BANDIT_EXCEPT<BanditException>( "Joint has an invalid handle." );
    bandit_handle_->setJointPos( name, args... );
}

template<class... __Args>
void Joint::setPIDConfig( __Args&&... args )
{
    if( !bandit_handle_ ) BANDIT_EXCEPT<BanditException>( "Joint has an invalid handle." );
    bandit_handle_->setJointPIDConfig( name, args... );
}

template<class... __Args>
void Joint::setDirection( __Args&&... args )
{
    if( !bandit_handle_ ) BANDIT_EXCEPT<BanditException>( "Joint has an invalid handle." );
    bandit_handle_->setJointDirection( name, args... );
}

template<class... __Args>
void Joint::setOffset( __Args&&... args )
{
    if( !bandit_handle_ ) BANDIT_EXCEPT<BanditException>( "Joint has an invalid handle." );
    bandit_handle_->setJointOffset( name, args... );
}

// =============================================================================================================================================
template<class... __Args>
void Bandit::addJoint( __Args&&... args )
{
    Joint joint( args... );
    joint.setHandle( this );

    joints_map_[joint.name] = joint;

    master_.setJointType( joint.mod_id, joint.type );
}

// =============================================================================================================================================
template<class... __Args>
void Bandit::setJointPIDConfig( JointName const & name, __Args&&... args )
{
    printf( "Setting PID config for joint %s\n", name.name_.c_str() );

    auto joint_it = getJoint( name );

    if( joint_it->second.type != smartservo::SMART_SERVO ) BANDIT_EXCEPT<BanditException>( "Tried to set PID gains on: %s, which is not a SMART_SERVO\n", name.name_.c_str() );

    master_.setJointPIDConfig( joint_it->second.mod_id, joint_it->second.which, args... );
}

#endif // LIBBANDIT_BANDIT_DETAILS_BANDITIMPL_H_

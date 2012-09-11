/***************************************************************************
 *  src/joint_name.cpp
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

#include <bandit/joint_name.h>

namespace bandit
{

// =============================================================================================================================================
JointName::JointName()
:
    id_( -1 ),
    name_( "" )
{
    //
}

// =============================================================================================================================================
JointName::JointName( int16_t const & id )
:
    id_( id ),
    name_( "" )
{
    auto const & names_map = getJointNamesMap();
    auto const & name_it = names_map.find( id );
    if( name_it != names_map.cend() ) name_ = name_it->second;
    else printf( "Warning: constructing invalid joint via id [ %i ]", id_ );
}

// =============================================================================================================================================
JointName::JointName( std::string const & name )
:
    id_( -1 ),
    name_( name )
{
    auto const & ids_map = getJointIdsMap();
    auto const & id_it = ids_map.find( name );
    if( id_it != ids_map.cend() ) id_ = id_it->second;
    else printf( "Warning: constructing invalid joint via name [ %s ]", name_.c_str() );
}

// =============================================================================================================================================
JointName::operator int16_t() const
{
    return id_;
}

// =============================================================================================================================================
JointName::operator std::string() const
{
    return name_;
}

} // bandit

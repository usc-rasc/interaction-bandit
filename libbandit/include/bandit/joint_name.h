/***************************************************************************
 *  include/bandit/joint_name.h
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

#ifndef LIBBANDIT_BANDIT_JOINTNAME_H_
#define LIBBANDIT_BANDIT_JOINTNAME_H_

#include <vector>
#include <map>
#include <string>

namespace bandit
{

// =============================================================================================================================================
//! Build a vectir if all known id-name pairs
static std::vector<std::pair<uint16_t, std::string> > const & getJointNamePairs()
{
    static const std::vector<std::pair<uint16_t, std::string> > joint_name_pairs
    {
        std::make_pair( (uint16_t) 0 , std::string( "head_tilt_joint"                        ) ),
        std::make_pair( (uint16_t) 1 , std::string( "head_pan_joint"                         ) ),
        std::make_pair( (uint16_t) 2 , std::string( "left_torso_shoulder_mounting_joint"     ) ),
        std::make_pair( (uint16_t) 3 , std::string( "left_shoulder_mounting_shoulder_joint"  ) ),
        std::make_pair( (uint16_t) 4 , std::string( "left_shoulder_bicep_joint"              ) ),
        std::make_pair( (uint16_t) 5 , std::string( "left_bicep_forearm_joint"               ) ),
        std::make_pair( (uint16_t) 6 , std::string( "left_forearm_wrist_joint"               ) ),
        std::make_pair( (uint16_t) 7 , std::string( "left_wrist_hand_joint"                  ) ),
        std::make_pair( (uint16_t) 8 , std::string( "left_hand_thumb_joint"                  ) ),
        std::make_pair( (uint16_t) 9 , std::string( "right_torso_shoulder_mounting_joint"    ) ),
        std::make_pair( (uint16_t) 10, std::string( "right_shoulder_mounting_shoulder_joint" ) ),
        std::make_pair( (uint16_t) 11, std::string( "right_shoulder_bicep_joint"             ) ),
        std::make_pair( (uint16_t) 12, std::string( "right_bicep_forearm_joint"              ) ),
        std::make_pair( (uint16_t) 13, std::string( "right_forearm_wrist_joint"              ) ),
        std::make_pair( (uint16_t) 14, std::string( "right_wrist_hand_joint"                 ) ),
        std::make_pair( (uint16_t) 15, std::string( "right_hand_thumb_joint"                 ) ),
        std::make_pair( (uint16_t) 16, std::string( "eyebrows_joint"                         ) ),
        std::make_pair( (uint16_t) 17, std::string( "mouth_top_joint"                        ) ),
        std::make_pair( (uint16_t) 18, std::string( "mouth_bottom_joint"                     ) )
    };

    return joint_name_pairs;
}

// =============================================================================================================================================
//! Build a map from joint names to joint ids
static std::map<std::string, uint16_t> const & getJointIdsMap()
{
    static bool initialized = false;
    static std::map<std::string, uint16_t> joint_ids_map;

    if( !initialized )
    {
        auto joint_name_pairs = getJointNamePairs();
        for( auto joint_name_it = joint_name_pairs.cbegin(); joint_name_it != joint_name_pairs.cend(); ++joint_name_it )
        {
            joint_ids_map[joint_name_it->second] = joint_name_it->first;
        }
        initialized = true;
    }

    return joint_ids_map;
}

// =============================================================================================================================================
//! Build a map from joint ids to joint names
static std::map<uint16_t, std::string> const & getJointNamesMap()
{
    static bool initialized = false;
    static std::map<uint16_t, std::string> joint_names_map;

    if( !initialized )
    {
        auto joint_name_pairs = getJointNamePairs();
        for( auto joint_name_it = joint_name_pairs.cbegin(); joint_name_it != joint_name_pairs.cend(); ++joint_name_it )
        {
            joint_names_map[joint_name_it->first] = joint_name_it->second;
        }
        initialized = true;
    }

    return joint_names_map;
}

// =============================================================================================================================================
// =============================================================================================================================================

// =============================================================================================================================================
//! A convenient data structure to automatically convert between joint names and joint ids
class JointName
{
public:
    int16_t id_;
    std::string name_;

    JointName();
    JointName( int16_t const & id );
    JointName( std::string const & name );

    operator int16_t() const;
    operator std::string() const;
};

} // bandit

#endif // LIBBANDIT_BANDIT_JOINTNAME_H_

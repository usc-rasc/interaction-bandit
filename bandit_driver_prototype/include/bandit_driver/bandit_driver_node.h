/***************************************************************************
 *  include/bandit_driver/bandit_driver_node.h
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

#ifndef BANDITDRIVERPROTOTYPE_BANDITDRIVER_BANDITDRIVERNODE_H_
#define BANDITDRIVERPROTOTYPE_BANDITDRIVER_BANDITDRIVERNODE_H_

#include <quickdev/node.h>

// policies
#include <quickdev/service_server_policy.h>

// objects
#include <quickdev/multi_publisher.h>
#include <quickdev/multi_subscriber.h>
#include <bandit/bandit.h>
#include <robot_state_publisher/robot_state_publisher.h>

// msgs
#include <bandit_msgs/JointArray.h>
#include <sensor_msgs/JointState.h>

// srvs
#include <bandit_msgs/GetJointProperties.h>

// others
#include <quickdev/geometry_message_conversions.h>
#include <quickdev/numeric_unit_conversions.h>
#include <quickdev/math.h>
#include <kdl_parser/kdl_parser.hpp>

typedef bandit_msgs::JointArray _BanditJointArrayMsg;
typedef sensor_msgs::JointState _JointStateMsg;

typedef bandit_msgs::GetJointProperties _GetJointPropertiesSrv;
typedef quickdev::ServiceServerPolicy<_GetJointPropertiesSrv> _GetJointPropertiesServiceServerPolicy;

QUICKDEV_DECLARE_NODE( BanditDriver, _GetJointPropertiesServiceServerPolicy )

QUICKDEV_DECLARE_NODE_CLASS( BanditDriver )
{
    typedef robot_state_publisher::RobotStatePublisher _RobotStatePublisher;

    ros::MultiPublisher<> multi_pub_;
    ros::MultiSubscriber<> multi_sub_;

    bandit::Bandit bandit_driver_;
    KDL::Tree bandit_model_;
    boost::shared_ptr<_RobotStatePublisher> bandit_state_pub_ptr_;

    _GetJointPropertiesSrv::Response cached_joint_properties_;
    bool bandit_state_initialized_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( BanditDriver ),
        bandit_state_initialized_( false )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        // -------------------------------------------------------------------------------------------------------------------------------------
        // load URDF and set up robot state publisher
        auto const urdf_filename = ros::ParamReader<std::string, 1>::readParam( nh_rel, "urdf_filename", "" );

        if( !kdl_parser::treeFromFile( urdf_filename, bandit_model_ ) )
        {
            ROS_ERROR( "Failed to construct kdl tree" );
            return QUICKDEV_GET_RUNABLE_POLICY()::interrupt();
        }

        bandit_state_pub_ptr_ = boost::make_shared<_RobotStatePublisher>( bandit_model_ );

        // -------------------------------------------------------------------------------------------------------------------------------------
        // set up bandit driver object
        auto const port = ros::ParamReader<std::string, 1>::readParam( nh_rel, "port", "/dev/ttyUSB0" );

        bandit_driver_.openPort( port.c_str() );
        bandit_driver_.useJointLimits( ros::ParamReader<bool, 1>::readParam( nh_rel, "use_joint_limits", false ) );

        // load and parse joint config
        auto & joints_map = bandit_driver_.getJoints();

        // read the full set of PID values for all joints off of the server at once
        auto joints_pid_values = ros::ParamReader<XmlRpc::XmlRpcValue, 1>::readParam( nh_rel, "pids" );
        // read the full set of config values for all joints off of the server at once
        auto joints_config_values = ros::ParamReader<XmlRpc::XmlRpcValue, 1>::readParam( nh_rel, "joints" );

        for( auto joints_it = joints_map.begin(); joints_it != joints_map.cend(); ++joints_it )
        {
            auto & joint = joints_it->second;

            std::string const & joint_name = joint.name;

            auto joint_pid_values =    ros::ParamReader<XmlRpc::XmlRpcValue, 1>::getXmlRpcValue( joints_pid_values,    joint_name );
            auto joint_config_values = ros::ParamReader<XmlRpc::XmlRpcValue, 1>::getXmlRpcValue( joints_config_values, joint_name );

            PRINT_INFO( "%s", joint_config_values.toXml().c_str() );

            if( joint.type == smartservo::SMART_SERVO )
            {
                auto const p =      ros::ParamReader<int, 1>::getXmlRpcValue( joint_pid_values, "p"     , 100   );
                auto const i =      ros::ParamReader<int, 1>::getXmlRpcValue( joint_pid_values, "i"     , 0     );
                auto const d =      ros::ParamReader<int, 1>::getXmlRpcValue( joint_pid_values, "d"     , 0     );
                auto const i_min =  ros::ParamReader<int, 1>::getXmlRpcValue( joint_pid_values, "i_min" , -4000 );
                auto const i_max =  ros::ParamReader<int, 1>::getXmlRpcValue( joint_pid_values, "i_max" , 4001  );
                auto const e_min =  ros::ParamReader<int, 1>::getXmlRpcValue( joint_pid_values, "e_min" , 50    );
                auto const offset = ros::ParamReader<int, 1>::getXmlRpcValue( joint_pid_values, "offset", 0     );

                joint.setPIDConfig( p, i, d, i_min, i_max, e_min, offset );
            }

            PRINT_INFO( "Getting joint direction" );
            auto const direction = ros::ParamReader<int   , 1>::getXmlRpcValue( joint_config_values, "direction", 1 );
            PRINT_INFO( "Getting joint origin" );
            auto const origin =    ros::ParamReader<double, 1>::getXmlRpcValue( joint_config_values, "origin"   , 0 );

            PRINT_INFO( "Setting joint direction and origin" );

            joint.setDirection( direction );
            joint.setOffset( Radian( Degree( origin ) ) );
        }

        // -------------------------------------------------------------------------------------------------------------------------------------
        // set up params service server
        _GetJointPropertiesServiceServerPolicy::registerCallback( quickdev::auto_bind( &BanditDriverNode::getJointPropertiesCB, this ) );
        initPolicies<_GetJointPropertiesServiceServerPolicy>( "service_name_param", std::string( "get_joint_properties" ) );

        auto const num_joints = bandit_driver_.getNumJoints();
        cached_joint_properties_.id.reserve( num_joints );
        cached_joint_properties_.name.reserve( num_joints );
        cached_joint_properties_.min.reserve( num_joints );
        cached_joint_properties_.max.reserve( num_joints );
        cached_joint_properties_.pos.reserve( num_joints );

        for( auto joints_it = joints_map.cbegin(); joints_it != joints_map.cend(); ++joints_it )
        {
            auto const & joint = joints_it->second;
            cached_joint_properties_.id.push_back( joint.name );
            cached_joint_properties_.name.push_back( joint.name );
            cached_joint_properties_.min.push_back( Degree( Radian( joint.min ) ) );
            cached_joint_properties_.max.push_back( Degree( Radian( joint.max ) ) );
            cached_joint_properties_.pos.push_back( Degree( Radian( joint.getPos() ) ) );
        }

        // -------------------------------------------------------------------------------------------------------------------------------------
        // create publishers and subscribers
        multi_pub_.addPublishers<_JointStateMsg>( nh_rel, { "joint_states" } );

        multi_sub_.addSubscriber( nh_rel, "joint_array_command", &BanditDriverNode::jointArrayCommandCB, this );
        multi_sub_.addSubscriber( nh_rel, "joint_state_command", &BanditDriverNode::jointStateCommandCB, this );

        initPolicies<quickdev::policy::ALL>();

        PRINT_INFO( "Setting and checking PID configs..." );

        ros::Time target_update_time = ros::Time::now();
        do
        {
            auto const now = ros::Time::now();
            if( now >= target_update_time )
            {
                PRINT_INFO( "Sending PID info to bandit..." );
                bandit_driver_.sendAllPIDConfigs();
                target_update_time = now + ros::Duration( 1.0 );
            }

            bandit_driver_.processIO( QUICKDEV_GET_RUNABLE_POLICY()::getLoopRateSeconds() * 1000000 );
            bandit_driver_.processPackets();
        }
        while( !bandit_driver_.checkAllPIDConfigs() );

        PRINT_INFO( "Setting initial joint positions." );

        for( auto joints_it = joints_map.begin(); joints_it != joints_map.cend(); ++joints_it )
        {
            auto & joint = joints_it->second;

            joint.setPos( 0 );
        }

        bandit_driver_.sendAllJointPos();

        bandit_driver_.registerStateCB( quickdev::auto_bind( &BanditDriverNode::banditStateCB, this ) );
    }

    QUICKDEV_SPIN_ONCE()
    {
        try
        {
            bandit_driver_.processIO( QUICKDEV_GET_RUNABLE_POLICY()::getLoopRateSeconds() * 1000000 );
            bandit_driver_.processPackets();
        }
        catch( bandit::BanditException const & ex )
        {
            PRINT_ERROR( "%s", ex.what() );
        }
        catch( smartservo::SmartServoException const & ex )
        {
            PRINT_ERROR( "%s", ex.what() );
        }
        catch( ... )
        {
            PRINT_ERROR( "Unknown exception" );
        }
    }

    void banditStateCB()
    {
        bandit_state_initialized_ = true;

        auto const now = ros::Time::now();
        // let's assume we're not going to change the number of joints or their name mappings after initialization
        auto static const num_joints = bandit_driver_.getNumJoints();
        // lookup the eyebrow joint to prevent doing this for every joint every function call
        auto static const eyebrows_joint_index = bandit_driver_.getJointId( std::string( "eyebrows_joint" ) );

        _JointStateMsg joint_state_msg;

        // reserve space in name and position to make push_back more efficient
        joint_state_msg.name.reserve( num_joints );
        joint_state_msg.position.reserve( num_joints );

        // set velocity and effort to 0
        joint_state_msg.velocity.assign( num_joints, 0 );
        joint_state_msg.effort.assign( num_joints, 0 );

        joint_state_msg.header.stamp = now;
        joint_state_msg.header.frame_id = "torso_link";

        std::map<std::string, double> joint_states_map;

        auto const & joints_map = bandit_driver_.getJoints();

        for( auto joints_it = joints_map.cbegin(); joints_it != joints_map.cend(); ++joints_it )
        {
            auto const & joint = joints_it->second;

            auto const joint_name = joint.name;
            auto const joint_pos = quickdev::normalizeEuler( joint.getPos() );
            // the eyebrows are really two joints as far as robot_state_publisher is concerned
            if ( joint.name == eyebrows_joint_index )
            {
                auto l_brow_pos = -joint_pos;
                auto r_brow_pos = -joint_pos;

                std::string l_brow_name( "head_left_brow_joint" );
                std::string r_brow_name( "head_right_brow_joint" );

                joint_state_msg.name.push_back( l_brow_name );
                joint_state_msg.position.push_back( l_brow_pos );
                joint_states_map[l_brow_name] = l_brow_pos;

                joint_state_msg.name.push_back( r_brow_name );
                joint_state_msg.position.push_back( r_brow_pos );
                joint_states_map[r_brow_name] = r_brow_pos;
            }
            else
            {
                joint_state_msg.name.push_back( joint_name );
                joint_state_msg.position.push_back( joint_pos );
                joint_states_map[joint_name] = joint_pos;
            }
        }

        bandit_state_pub_ptr_->publishTransforms( joint_states_map, now );

#if QUICKDEV_ROS_VERSION >= ROS_VERSION_ELECTRIC

        bandit_state_pub_ptr_->publishFixedTransforms();

#endif

        multi_pub_.publish( "joint_states", quickdev::make_const_shared( joint_state_msg ) );
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( jointArrayCommandCB, _BanditJointArrayMsg )
    {
        if( !bandit_state_initialized_ )
        {
            PRINT_WARN( "Ignoring joint commands; driver not yet initialized." );
            return;
        }

        auto const & joints = msg->joints;

        for( auto joints_it = joints.cbegin(); joints_it != joints.cend(); ++joints_it )
        {
            bandit_driver_.setJointPos( joints_it->id, joints_it->angle );
        }

        bandit_driver_.sendAllJointPos();
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( jointStateCommandCB, _JointStateMsg )
    {
        if( !bandit_state_initialized_ )
        {
            PRINT_WARN( "Ignoring joint commands; driver not yet initialized." );
            return;
        }

        auto const & names = msg->name;
        auto const & positions = msg->position;

        auto names_it = names.cbegin();
        auto positions_it = positions.cbegin();

        for( ; names_it != names.cend(); ++names_it, ++positions_it )
        {
            bandit_driver_.setJointPos( *names_it, *positions_it );
        }

        bandit_driver_.sendAllJointPos();
    }

    QUICKDEV_DECLARE_SERVICE_CALLBACK( getJointPropertiesCB, _GetJointPropertiesSrv )
    {
        if( !bandit_state_initialized_ )
        {
            PRINT_WARN( "Ignoring GetJointProperties request; driver not yet initialized." );
            return false;
        }

        auto const & joints_map = bandit_driver_.getJoints();

        cached_joint_properties_.pos.clear();
        cached_joint_properties_.pos.reserve( joints_map.size() );

        for( auto joints_it = joints_map.cbegin(); joints_it != joints_map.cend(); ++joints_it )
        {
            cached_joint_properties_.pos.push_back( Degree( Radian( joints_it->second.getPos() ) ) );
        }

        response = cached_joint_properties_;

        return true;
    }
};

#endif // BANDITDRIVERPROTOTYPE_BANDITDRIVER_BANDITDRIVERNODE_H_


#include <math.h>
#include <cstdio>
#include <bandit/bandit.h>
#include <string>

#define DTOR( a ) ( ( a ) * M_PI / 180.0f )

namespace bandit
{

//#define DEBUG
//#define HIDE_ERRORS

// =============================================================================================================================================
Joint::Joint
(
    JointName const & name_,
    int16_t const & mod_id_,
    smartservo::WhichJoint const & which_,
    smartservo::JointType const & type_,
    int8_t const & direction_,
    double const & scale_,
    double const & offset_,
    double const & min_,
    double const & max_
)
:
    name( name_ ),
    mod_id( mod_id_ ),
    which( which_ ),
    type( type_ ),
    direction( direction_ ),
    scale( scale_ ),
    offset( offset_ ),
    min( min_ ),
    max( max_ )
{
    //
}

void Joint::setHandle( Bandit * handle )
{
    bandit_handle_ = handle;
}

double Joint::getPos() const
{
    if( !bandit_handle_ ) BANDIT_EXCEPT<BanditException>( "Joint has an invalid handle." );
    return bandit_handle_->getJointPos( name );
}

// =============================================================================================================================================
// =============================================================================================================================================

// =============================================================================================================================================
Bandit::Bandit()
:
    master_( 7 )
{
    addJoint( 0,  6, smartservo::JOINT_A, smartservo::SMART_SERVO, 1, 2*M_PI/4096.0, M_PI, DTOR( -20.0f ), DTOR( 30.0f ) );  // "head pitch"
    addJoint( 1,  2, smartservo::JOINT_B, smartservo::SMART_SERVO, 1, 2*M_PI/4096.0, M_PI, DTOR( -90.0f ), DTOR( 90.0f ) );  // "head pan"
    addJoint( 2,  5, smartservo::JOINT_A, smartservo::SMART_SERVO, 1, 2*M_PI/4096.0, M_PI, DTOR( -30.0f ), DTOR( 180.0f ) ); // "left shoulder F/B"
    addJoint( 3,  4, smartservo::JOINT_B, smartservo::SMART_SERVO, 1, 2*M_PI/4096.0, M_PI, DTOR(  0.0f ),  DTOR( 150.0f ) ); // "left shoulder I/O"
    addJoint( 4,  4, smartservo::JOINT_A, smartservo::SMART_SERVO, 1, 2*M_PI/4096.0, M_PI, DTOR( -90.0f ), DTOR( 90.0f ) );  // "left elbow twist"
    addJoint( 5,  3, smartservo::JOINT_A, smartservo::SMART_SERVO, 1, 2*M_PI/4096.0, M_PI, DTOR(  0.0f ),  DTOR( 110.0f ) ); // "left elbow"
    addJoint( 6,  3, smartservo::JOINT_B, smartservo::SMART_SERVO, 1, 2*M_PI/4096.0, M_PI, DTOR( -90.0f ), DTOR( 90.0f ) );  // "left wrist twist"
    addJoint( 7,  3, smartservo::JOINT_B, smartservo::V1_SERVO,    1, 1/255.0,       0.0,  DTOR( -90.0f ), DTOR( 90.0f ) );  // "left wrist tilt"
    addJoint( 8,  3, smartservo::JOINT_A, smartservo::V1_SERVO,    1, 1/255.0,       0.0,  -0.5,           0.0 );            // "left hand grab"
    addJoint( 9,  2, smartservo::JOINT_A, smartservo::SMART_SERVO, 1, 2*M_PI/4096.0, M_PI, DTOR( -30.0f ), DTOR( 180.0f ) ); // "right shoulder F/B"
    addJoint( 10, 1, smartservo::JOINT_B, smartservo::SMART_SERVO, 1, 2*M_PI/4096.0, M_PI, DTOR(  0.0f ),  DTOR( 150.0f ) ); // "right shoulder I/O"
    addJoint( 11, 1, smartservo::JOINT_A, smartservo::SMART_SERVO, 1, 2*M_PI/4096.0, M_PI, DTOR( -90.0f ), DTOR( 90.0f ) );  // "right elbow twist"
    addJoint( 12, 0, smartservo::JOINT_A, smartservo::SMART_SERVO, 1, 2*M_PI/4096.0, M_PI, DTOR(  0.0f ),  DTOR( 110.0f ) ); // "right elbow"
    addJoint( 13, 0, smartservo::JOINT_B, smartservo::SMART_SERVO, 1, 2*M_PI/4096.0, M_PI, DTOR( -90.0f ), DTOR( 90.0f ) );  // "right wrist twist"
    addJoint( 14, 0, smartservo::JOINT_B, smartservo::V1_SERVO,    1, 1/255.0,       0.0, -0.5,            0.5 );            // "right wrist tilt"
    addJoint( 15, 0, smartservo::JOINT_A, smartservo::V1_SERVO,    1, 1/255.0,       0.0, -0.5,            0.0 );            // "right hand grab"
    addJoint( 16, 5, smartservo::JOINT_B, smartservo::HOBBY_SERVO, 1, 1/255.0,       0.0, -0.1,            0.4 );            // "eyebrows_joint"
    addJoint( 17, 6, smartservo::JOINT_A, smartservo::HOBBY_SERVO, 1, 1/255.0,       0.0, -0.25,           0.25 );           // "mouth top_joint"
    addJoint( 18, 6, smartservo::JOINT_B, smartservo::HOBBY_SERVO, 1, 1/255.0,       0.0, -0.25,           0.25 );           // "mouth bottom_joint"
}

// =============================================================================================================================================
Bandit::~Bandit()
{

}

// =============================================================================================================================================
void Bandit::useJointLimits( bool const & use_limits )
{
    joint_limits_ = use_limits;
}

// =============================================================================================================================================
Bandit::_JointsMap::iterator Bandit::getJoint( JointName const & name )
{
    auto joint_it = joints_map_.find( name );

    if( joint_it == joints_map_.end() ) BANDIT_EXCEPT<BanditException>( "No joint with id %d", name.id_ );

    return joint_it;
}

// =============================================================================================================================================
Bandit::_JointsMap::const_iterator Bandit::getJoint( JointName const & name ) const
{
    auto joint_it = joints_map_.find( name );

    if( joint_it == joints_map_.end() ) BANDIT_EXCEPT<BanditException>( "No joint with id %d", name.id_ );

    return joint_it;
}

// =============================================================================================================================================
Bandit::_JointsMap & Bandit::getJoints()
{
    return joints_map_;
}

// =============================================================================================================================================
Bandit::_JointsMap const & Bandit::getJoints() const
{
    return joints_map_;
}

// =============================================================================================================================================
smartservo::JointType Bandit::getJointType( JointName const & name )
{
    return getJoint( name )->second.type;
}

// =============================================================================================================================================
std::string Bandit::getJointName( JointName const & name )
{
    return name;
}

// =============================================================================================================================================
int16_t Bandit::getJointId( JointName const & name )
{
    return name;
}

// =============================================================================================================================================
size_t Bandit::getNumJoints()
{
    return joints_map_.size();
}

// =============================================================================================================================================
double Bandit::getJointMin( JointName const & name )
{
    return getJoint( name )->second.min;
}

// =============================================================================================================================================
double Bandit::getJointMax( JointName const & name )
{
    return getJoint( name )->second.max;
}

// =============================================================================================================================================
double Bandit::getJointPos( JointName const & name ) const
{
    auto joint_it = getJoint( name );

    int16_t pos = 0;

    switch ( joint_it->second.type )
    {
    case smartservo::SMART_SERVO:
        pos = master_.getJointPos( joint_it->second.mod_id, joint_it->second.which );
        break;
    case smartservo::HOBBY_SERVO:
    case smartservo::V1_SERVO:
        pos = master_.getJointServoPos( joint_it->second.mod_id, joint_it->second.which );
        break;
    default:
        BANDIT_EXCEPT<BanditException>( "Tried to get a joint position for a joint with unspecified type" );
    }

    return joint_it->second.direction * ( joint_it->second.scale * ( double )( pos ) - joint_it->second.offset );
}


// =============================================================================================================================================
void Bandit::setJointDirection( JointName const & name, int8_t const & direction )
{
    if( direction != -1 && direction != 1 ) BANDIT_EXCEPT<BanditException>( "Tried to set joint direction with invalid value" );

    getJoint( name )->second.direction = direction;
}

// =============================================================================================================================================
void Bandit::setJointOffset( JointName const & name, double const & offset )
{
    auto joint_it = getJoint( name );

    if( joint_it->second.type == smartservo::SMART_SERVO ) joint_it->second.offset = offset + joint_it->second.direction * M_PI;
    else joint_it->second.offset = offset;
}

// =============================================================================================================================================
void Bandit::setJointPos( JointName const & name, double const & raw_angle )
{
    auto joint_it = getJoint( name );

    double angle = raw_angle;

    if( joint_limits_ )
    {
        if( angle > joint_it->second.max ) angle = joint_it->second.max;
        if( angle < joint_it->second.min ) angle = joint_it->second.min;
    }

    int16_t pos = ( joint_it->second.direction * angle + joint_it->second.offset ) / joint_it->second.scale;

    switch ( joint_it->second.type )
    {
    case smartservo::SMART_SERVO:
        master_.setJointPos( joint_it->second.mod_id, joint_it->second.which, pos );
        break;
    case smartservo::HOBBY_SERVO:
    case smartservo::V1_SERVO:
        printf( "setting hobby servo to: %d\n", pos );
        if( pos > 255 ) pos = 255;
        if( pos < 0 ) pos = 0;
        master_.setJointServoPos( joint_it->second.mod_id, joint_it->second.which, pos );
        break;
    default:
        BANDIT_EXCEPT<BanditException>( "Tried to set a joint position for a joint with unspecified type" );
    }
}

// =============================================================================================================================================
void Bandit::openPort( std::string const & port_name )
{
    master_.openPort( port_name.c_str() );
}

// =============================================================================================================================================
void Bandit::closePort()
{
    master_.closePort();
}

// =============================================================================================================================================
void Bandit::registerStateCB( Bandit::_StateCallback const & callback )
{
    master_.registerStateCB( callback );
}

// =============================================================================================================================================
void Bandit::processIO( uint32_t const & timeout )
{
    master_.processIO( timeout );
}

// =============================================================================================================================================
void Bandit::processPackets( bool const & wait )
{
    master_.processPackets( wait );
}

// =============================================================================================================================================
void Bandit::sendAllJointPos()
{
    master_.sendAllJointPos();
    master_.sendAllJointServoPos();
}

// =============================================================================================================================================
void Bandit::sendAllPIDConfigs()
{
    master_.sendAllPIDConfigs();
}

// =============================================================================================================================================
bool Bandit::checkAllPIDConfigs( bool const & verbose )
{
    return master_.checkAllPIDConfigs( verbose );
}

} // bandit

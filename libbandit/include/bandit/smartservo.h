#ifndef SMARTSERVO_H
#define SMARTSERVO_H

#include <stdexcept>
#include <termios.h>
#include <string>
#include <stdint.h>

#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include <queue>

#define MAX_BUF_SIZE 256

//! A namespace containing the bluesky bandit device driver
namespace smartservo
{

  //! Macro for defining an exception with a given parent (std::runtime_error should be top parent)
#define DEF_EXCEPTION(name, parent) \
  class name  : public parent { \
  public: \
    name(const char* msg) : parent(msg) {} \
  }

  //! A standard SmartServo exception
  DEF_EXCEPTION(SmartServoException, std::runtime_error);

#undef DEF_EXCEPTION

  //! Enum for specifying joint type
  enum JointType     { NO_JOINT_TYPE, SMART_SERVO, HOBBY_SERVO, V1_SERVO };
  enum WhichJoint    { JOINT_A = 0, JOINT_B = 1, NO_JOINT = 2 };
  enum PIDUpdate     { PID_NONE, PID_NEW, PID_SENT, PID_CONFIRMED_GAINS, PID_CONFIRMED_LIMITS, PID_CONFIRMED };
  enum ServoUpdate   { SERVO_NONE, SERVO_NEW, SERVO_SENT };

  //! A class representing the master module
  /*!
   *  This class represents the bridge for communicating with the joint modules
   *  It bridges the communication between serial port and the CAN bus.  This class
   *  is intended to directly represent the real data structures and communication
   *  and is expected to be wrapped by another class which handles the more
   *  robot-specific details.
   *
   * NOTE: This is not "bandit-specific" and can be pulled from this library if/when
   * appropriate.
   */
  class MasterModule
  {
  private:

    //! A class representing a smart servo joint
    /*!
     *  There are two joints per joint module.  Each is responsible
     *  for running a PID control loop on the particular joint.
     */
    class SmartServoJoint
    {
    public:
      int16_t                des_pos;
      int16_t                cur_pos;
      int8_t                 speed;
      uint8_t                current;
      int16_t                p;
      int16_t                i;
      int16_t                d;
      int16_t                i_min;
      int16_t                i_max;
      uint8_t                e_min;
      int16_t                offset;
      PIDUpdate              pid_update;

      SmartServoJoint() : des_pos(0), cur_pos(0), speed(0), current(0),
                          p(0), i(0), d(0),
                          i_min(0), i_max(0), e_min(0), offset(0),
                          pid_update(PID_NONE) { }
    };

    //! A class implementing a Servo Joint
    class ServoJoint
    {
    public:
      uint8_t      des_pos;
      ServoUpdate  servo_update;

      ServoJoint() : des_pos(128), servo_update(SERVO_NONE) { }
    };

    //! A class representing the JointModules
    /*!
     *  This represents the low-level motor control, each of which can control up to
     *  2 motors and 2 servos.  This is mostly just used for book-keeping of state.
     */
    class JointModule
    {
    public:
      uint8_t                twi_addr;
      SmartServoJoint        joint[2];
      ServoJoint             servo[2];
      JointType              joint_type;  // Determines whether S or V message is used for servos

      JointModule() : twi_addr(0), joint_type(NO_JOINT_TYPE) { }
    };

    //! A class representing a single IO packet
    class IOPacket
    {
    public:
      char contents[MAX_BUF_SIZE];
      uint32_t size;

      IOPacket(char* _buf, uint32_t _size) : size(_size)
      {
        if (_size > MAX_BUF_SIZE - 1)
          throw SmartServoException("Attempted to allocate packet size larger than max buf size");

        memcpy(contents, _buf, _size);

        contents[_size] = '\0';
      }

      //      IOPacket(const IOPacket in) : size(in.size)
      //      {
      //        memcpy(contents, in.contents, size);
      //      }

    };

    boost::mutex state_mutex_;

    uint16_t                num_modules_;
    JointModule*            modules_;

    uint32_t v_confirmations_;
    uint32_t s_confirmations_;
    uint32_t v_transmits_;
    uint32_t s_transmits_;

    boost::function<void()> state_callback_;


    int                     fd_;
    fd_set select_fds_;
    int max_fd_;

    char incoming_buf_[MAX_BUF_SIZE];
    int32_t incoming_pos_;
    boost::mutex incoming_queue_mutex_;
    boost::condition_variable incoming_queue_condition_;
    std::queue<IOPacket> incoming_queue_;

    IOPacket outgoing_packet_;
    int32_t outgoing_pos_;
    boost::mutex outgoing_queue_mutex_;
    std::queue<IOPacket> outgoing_queue_;

    int read_count_;

  public:
    //! Constructor
    MasterModule(int joints);

    //! Destructor
    ~MasterModule();

    //! Open the port
    void openPort(const char* port_name);

    //! Close the port
    void closePort();



    //! State Callback which will be triggered whenever a state update is received
    void registerStateCB(boost::function<void()> state_callback);



    //! Process incoming and outgoing serial communication
    /*!
     *  Processes pending incoming and outgoing serial messages from
     *  the master module.  This may invoke your state callback
     *  several several times.
     *
     *  \param timeout   Timeout in microseconds (0 for indefinite)
     */
    void processIO(uint32_t timeout);


    void processPackets(bool wait);



    //! Set the PID configuations for a particular joint.
    void setJointPIDConfig(uint16_t mod_id, WhichJoint which,
                        int16_t p, int16_t i, int16_t d,
                        int16_t i_min, int16_t i_max,
                        uint8_t e_min, int16_t offset);


    //! Manually send the PID configuration for a particular joint
    void sendPIDConfig(uint16_t mod_id);

    //! Sync all the joint PID configurations to the master module
    void sendAllPIDConfigs();

    bool checkAllPIDConfigs(bool verbose=false);



    //! Send all the joint positions to bandit.
    /*!
     *  This gets done with a single message and so these cannot be separated.
     */
    void sendAllJointPos();

    //! Send a particular joint servo position to bandit
    void sendJointServoPos(uint16_t mod_id);

    //! Send all (unsent) joint servo positions to bandit
    void sendAllJointServoPos();



    //! Set joint servo type
    void setJointType(uint16_t mod_id, JointType type);

    //! Set the joint position of a particular joint
    void setJointPos(uint16_t mod_id, WhichJoint which, int16_t pos);

    //! Set the servo position of a particular joint
    void setJointServoPos(uint16_t mod_id, WhichJoint which, uint8_t pos);



    //! Get the value of a joint position
    int16_t getJointPos(uint16_t mod_id, WhichJoint which) const;

    //! Set the joint servo position of a particular joint
    uint8_t getJointServoPos(uint16_t mod_id, WhichJoint which) const;


  private:

    void processIPIDGains(char* buf, uint32_t size);

    void processIPIDLimits(char* buf, uint32_t size);

    //! Process an incoming state message
    void processState(char* buf, uint32_t size);

  };

};

#endif

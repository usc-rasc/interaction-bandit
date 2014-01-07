#include <math.h>

#include <stdio.h>
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/select.h>
#include <sys/time.h>

#include <sys/signal.h>

#include "bandit/smartservo.h"

//#define DEBUG
//#define DEBUG2
//#define HIDE_ERRORS

#define SMARTSERVO_EXCEPT(except, msg, ...) \
  { \
    char buf[256]; \
    snprintf(buf, 256, "%s : %d - %s::%s: " msg, __FILE__, __LINE__, __CLASS__, __FUNCTION__,##__VA_ARGS__); \
    throw except(buf); \
  }

#define VERIFY_PORT_OPENED() \
  if (fd_ < 0) \
  { \
    char buf[256]; \
    snprintf(buf, 256, "%s : %d - %s::%s: Port has not been opened yet", __FILE__, __LINE__, __CLASS__, __FUNCTION__); \
    throw SmartServoException(buf); \
  }


// Time helper functions to go into and out of timeval
int64_t timeval_to_usec( const struct timeval* tv )
{
  return( (int64_t)tv->tv_sec * 1000000 + tv->tv_usec ) ;
}

struct timeval* usec_to_timeval( int64_t usec, struct timeval* p_tv )
{
  p_tv->tv_sec = usec / 1000000 ;
  p_tv->tv_usec = usec % 1000000 ;
  return p_tv ;
}

using namespace smartservo;

#define __CLASS__ "smartservo::MasterModule"

MasterModule::MasterModule(int num_modules) : outgoing_packet_(NULL, 0)
{
  state_callback_ = NULL;
  num_modules_ = num_modules;
  modules_ = new JointModule[num_modules];
  for (int i = 0; i < num_modules; i++)
    modules_[i].twi_addr = i+1; // For some strange reason these are 1-indexed

  outgoing_pos_ = -1;
  incoming_pos_ = 0;

  v_confirmations_ = 0;
  s_confirmations_ = 0;
  v_transmits_ = 0;
  s_transmits_ = 0;

}

MasterModule::~MasterModule()
{
  delete modules_;
}

void MasterModule::openPort(const char* port_name)
{
  fd_ = open(port_name, O_RDWR);
  if (fd_ < 0)
  {
    SMARTSERVO_EXCEPT(SmartServoException, "Could not open port %s -- %s", port_name, strerror(errno));
  }

  struct termios newtio;

  tcgetattr(fd_, &newtio);

  newtio.c_cflag = CRTSCTS | CS8 |  CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag &= ~(ICANON | ECHO);
  newtio.c_cc[VMIN]=0;
  newtio.c_cc[VTIME]=0;

  if (cfsetspeed(&newtio, B115200) < 0)
    SMARTSERVO_EXCEPT(SmartServoException, "Could not set port speed -- %s", strerror(errno));

  if (tcflush(fd_, TCIFLUSH) < 0)
    SMARTSERVO_EXCEPT(SmartServoException, "Could not flush port -- %s", strerror(errno));

  if (tcsetattr(fd_,TCSANOW,&newtio) < 0)
    SMARTSERVO_EXCEPT(SmartServoException, "Could not set port attributes -- %s", strerror(errno));


  FD_ZERO(&select_fds_);
  FD_SET(fd_, &select_fds_);
  max_fd_ = fd_ + 1;
}

void MasterModule::closePort()
{
  close(fd_);
}

void MasterModule::registerStateCB(boost::function<void()> state_callback)
{
  state_callback_ = state_callback;
}

void MasterModule::processIO(uint32_t timeout)
{
  VERIFY_PORT_OPENED();

  // Fd_sets to use
  fd_set in_fds;
  fd_set out_fds;
  fd_set error_fds;

  // Timeout to use for select
  timeval tv;
  usec_to_timeval(10000, &tv);

  // Timeout to use for overall processIO duration
  struct timeval starttime;
  gettimeofday(&starttime, NULL);
  uint64_t stoptime = timeval_to_usec(&starttime) + timeout;
  struct timeval checktime;

  do
  {
    // If outgoing buf is not empty, set the position to 0 so we know we have stuff to write
    //fprintf( stderr, "do loop" );
    // LOCK outgoing_queue
    {
      boost::mutex::scoped_lock(outgoing_mutex_);
      if (outgoing_pos_ == -1 && !outgoing_queue_.empty())
      {
        outgoing_packet_ = outgoing_queue_.front();
        outgoing_pos_ = 0;
        outgoing_queue_.pop();
      }
    }

    // Initialize our fd_sets
#ifdef __APPLE__
    FD_COPY(&select_fds_, &in_fds);
    FD_COPY(&select_fds_, &out_fds);
    FD_COPY(&select_fds_, &error_fds);
#else
    in_fds = select_fds_;
    out_fds = select_fds_;
    error_fds = select_fds_;
#endif

    int retval = 0;

    // If we're not trying to write, zero out out_fds:
    if (outgoing_pos_ == -1)
      FD_ZERO(&out_fds);

    // Select
    retval = select(max_fd_, &in_fds, &out_fds, &error_fds, &tv);

    // If retval < 0 an error occured
    if (retval < 0)
      SMARTSERVO_EXCEPT(SmartServoException, "select failed   --  error = %d: %s", errno, strerror(errno));

    // If retval > 0 we have something to do
    if (retval > 0)
    {
      // If there is something to read
      if (FD_ISSET(fd_, &in_fds))
      {
        // Read as much as possible that fits into our buffer
        int num_read = read(fd_, &(incoming_buf_[incoming_pos_]), MAX_BUF_SIZE - incoming_pos_ );

        switch (num_read)
        {
        case -1:
          SMARTSERVO_EXCEPT(SmartServoException, "read error");
        case 0:
          SMARTSERVO_EXCEPT(SmartServoException, "read EOF");
        default:

#ifdef DEBUG2
    for (int k = incoming_pos_; k < incoming_pos_ + num_read; k++)
      {
        if (incoming_buf_[k] == '<')
          {
        fprintf("< ");
        read_count_ = 0;
          }
        else if (incoming_buf_[k] == '>')
          {
        fprintf("> ");
        read_count_++;
          }
        else
          fprintf("%2d ",read_count_++);
      }

    printf("\n");
#endif

          incoming_pos_ += num_read;
        }
      }

      // Write as much as possible
      if (FD_ISSET(fd_, &out_fds))
      {
        int num_wrote = write(fd_, &outgoing_packet_.contents[outgoing_pos_], outgoing_packet_.size - outgoing_pos_);

        switch (num_wrote)
        {
        case -1:
          SMARTSERVO_EXCEPT(SmartServoException, "write error");
        case 0:
          SMARTSERVO_EXCEPT(SmartServoException, "write EOF?");
        default:
          outgoing_pos_ += num_wrote;
        }

        if (outgoing_pos_ == outgoing_packet_.size)
          outgoing_pos_ = -1;
      }

      if (FD_ISSET(fd_, &error_fds))
      {
        SMARTSERVO_EXCEPT(SmartServoException, "IO Error?");
      }
    }

    // Parse through read data to see if we have whole packets and push them into the incoming_queue.

    int skip_num = 0;
    int i = 0;
    int j = 0;

    for (;;)
    {
      //fprintf( stderr, "for ;; loop (%d<%d, %d\n",i,incoming_pos_, j );

      // Read until we find a '<' or hit the number of characters we've read
      while ( (i < incoming_pos_) && (incoming_buf_[i] != '<') )
        i++;

      // If at the end, we're done.
      if (i == incoming_pos_)
        break;

      skip_num = 0;

      if ( (i + 2) < incoming_pos_)
      {
        if (incoming_buf_[i + 1] == 'f' || incoming_buf_[i + 1] == 'F')
          skip_num = incoming_buf_[i+2];
      }

      // Read from i until we find a '>' or hit the number of characters we've read
      j = i + skip_num;

      while ( (j < incoming_pos_) && (incoming_buf_[j] != '>') )
        j++;

      // If at the end, we're done.
      if (j >= incoming_pos_)
        break;

      // Push the contents into an IOPacket
      {
        boost::mutex::scoped_lock(outgoing_mutex_);
        incoming_queue_.push(IOPacket(&incoming_buf_[i], j-i+1));
      }
      incoming_queue_condition_.notify_one();
      // Shift i up to meet j
      i = j;

      // add timeout to for loop
      gettimeofday(&checktime, NULL);
      //fprintf(stderr, "timeout: %ld\n", stoptime-timeval_to_usec(&checktime) );

    }

    // If there are characters at the front, we need to shift our buffer
    if (i > 0)
    {
      memmove(&(incoming_buf_[0]), &(incoming_buf_[i]), incoming_pos_ - i);
      incoming_pos_ -= i;
    }

    gettimeofday(&checktime, NULL);
    //fprintf(stderr, "other timeout: %ld\n", stoptime-timeval_to_usec(&checktime) );
  } while ( (timeout == 0) || timeval_to_usec(&checktime) < stoptime );

}


void MasterModule::processPackets(bool wait)
{
  boost::unique_lock<boost::mutex> lock(incoming_queue_mutex_);

  if (wait)
  {
    while(incoming_queue_.empty())
    {
      incoming_queue_condition_.wait(lock);
    }
  }

  while (!incoming_queue_.empty())
  {
    IOPacket packet = incoming_queue_.front();
    incoming_queue_.pop();

    lock.unlock();

    switch (packet.contents[1])
    {
    case 'f':
    case 'F':
      processState(packet.contents, packet.size);
      break;
    case 'i':
    case 'I':
      if (!strncmp(packet.contents, "<IPIDGains", 10))
        processIPIDGains(packet.contents, packet.size);
      else if (!strncmp(packet.contents, "<IPIDLimits", 11))
        processIPIDLimits(packet.contents, packet.size);
#ifndef HIDE_ERRORS
      else if (!strncmp(packet.contents, "<IReset", 7))
      {
        fprintf(stderr,"ERROR: %s\n", packet.contents);
      }
      else if (!strncmp(packet.contents, "<IcfgMsgRx", 10))
      {
        //Ignore it
      }
      else
        printf("ERROR: Got unknown I message: %s\n", packet.contents);
#endif
      break;
    case 'v':
    case 'V':
      v_confirmations_++;
      break;
    case 's':
    case 'S':
      s_confirmations_++;
      break;
#ifdef DEBUG
    default:
      printf("Read unhandled message size %d: %s\n", packet.size, packet.contents);
#endif
    }

    lock.lock();
  }
}

void MasterModule::processIPIDGains(char* buf, uint32_t size)
{
  int twi_addr = 0;
  int joint = 0;
  int p = 0;
  int i = 0;
  int d = 0;

  sscanf(buf, "<IPIDGains %d,%d,%d,%d,%d",&twi_addr,&joint,&p,&i,&d);

  SmartServoJoint* j = &modules_[twi_addr-1].joint[joint];

  if (j->p != p || j->i != i || j->d != d)
  {
    return;
  #ifndef HIDE_ERRORS
    fprintf(stderr,"ERROR: Returned Gains for %d/%d wrong.\n  p: %d (sent: %d)  i: %d (sent: %d)  d: %d (sent: %d)\n [%s]\n", twi_addr - 1, joint, p, j->p, i, j->i, d, j->d, buf);
  #endif
  }

  if (j->pid_update == PID_SENT)
    j->pid_update = PID_CONFIRMED_GAINS;

  if (j->pid_update == PID_CONFIRMED_LIMITS)
    j->pid_update = PID_CONFIRMED;
}

void MasterModule::processIPIDLimits(char* buf, uint32_t size)
{

  int twi_addr = 0;
  int joint = 0;
  int i_min = 0;
  int i_max = 0;
  int e_min = 0;
  int offset = 0;

  sscanf(buf, "<IPIDLimits %d,%d,%d,%d,%d,%d",&twi_addr,&joint,&i_min,&i_max,&e_min,&offset);

  SmartServoJoint* j = &modules_[twi_addr-1].joint[joint];

  if (j->i_min != i_min || j->i_max != i_max || j->e_min != e_min || j->offset != offset)
  {
    return;
  #ifndef HIDE_ERRORS

    fprintf(stderr,"ERROR: Returned Limits for %d/%d wrong.\n  i_min: %d (sent: %d)  i_max: %d (sent: %d)  e_min: %d (sent: %d)  offset: %d (sent: %d)\n [%s]\n", twi_addr - 1, joint, i_min, j->i_min, i_max, j->i_max, e_min, j->e_min, offset, j->offset, buf);
  #endif
  }

  if (j->pid_update == PID_SENT)
    j->pid_update = PID_CONFIRMED_LIMITS;

  if (j->pid_update == PID_CONFIRMED_GAINS)
    j->pid_update = PID_CONFIRMED;
}

void MasterModule::processState(char* buf, uint32_t size)
{
  if ( size != 4+buf[2])
  {
#ifndef HIDE_ERRORS
    fprintf(stderr,"Read state message with wrong length: %d/%d\n", size, 4+buf[2]);
#endif
    return;
  }

  if ( num_modules_ != buf[2]/8)
    SMARTSERVO_EXCEPT(SmartServoException, "Incompatible number of module.");

  for (int index = 0; index < buf[2]/8; index++)
  {
    modules_[index].joint[JOINT_A].cur_pos = *(int16_t*)(&buf[3 + index*8 + 0]);
    modules_[index].joint[JOINT_A].speed   =  *(int8_t*)(&buf[3 + index*8 + 2]);
    modules_[index].joint[JOINT_A].current = *(uint8_t*)(&buf[3 + index*8 + 3]);

    modules_[index].joint[JOINT_B].cur_pos = *(int16_t*)(&buf[3 + index*8 + 4]);
    modules_[index].joint[JOINT_B].speed   =  *(int8_t*)(&buf[3 + index*8 + 6]);
    modules_[index].joint[JOINT_B].current = *(uint8_t*)(&buf[3 + index*8 + 7]);
  }

  if (state_callback_)
    state_callback_();
}

void MasterModule::setJointPIDConfig(uint16_t mod_id, WhichJoint which,
                                     int16_t p, int16_t i, int16_t d,
                                     int16_t i_min, int16_t i_max,
                                     uint8_t e_min, int16_t offset)
{
  if (mod_id >= num_modules_)
    SMARTSERVO_EXCEPT(SmartServoException, "Specified module exceeded bounds");

  if (which == NO_JOINT)
    SMARTSERVO_EXCEPT(SmartServoException, "Which joint to use for module not specified");

  // Put PID values into the data structure
  modules_[mod_id].joint[which].p      = p;
  modules_[mod_id].joint[which].i      = i;
  modules_[mod_id].joint[which].d      = d;
  modules_[mod_id].joint[which].i_min  = i_min;
  modules_[mod_id].joint[which].i_max  = i_max;
  modules_[mod_id].joint[which].e_min  = e_min;
  modules_[mod_id].joint[which].offset = offset;

  modules_[mod_id].joint[which].pid_update = PID_NEW;
}


void MasterModule::sendPIDConfig(uint16_t mod_id)
{
  VERIFY_PORT_OPENED();

  // If these settings have been confirmed, we're done
  if (modules_[mod_id].joint[JOINT_A].pid_update == PID_CONFIRMED &&
      modules_[mod_id].joint[JOINT_B].pid_update == PID_CONFIRMED)
    return;

  if (modules_[mod_id].joint[JOINT_A].pid_update == PID_NONE &&
      modules_[mod_id].joint[JOINT_B].pid_update == PID_NONE)
    return;

  // Assemble and actually send out PID message
  char pid_msg[256];
  int len = 0;

  pid_msg[len++] = '<';
  *(uint8_t*)(&pid_msg[len++]) = 'C';
  *(uint8_t*)(&pid_msg[len++]) = 30;
  *(int16_t*)(&pid_msg[len]) = modules_[mod_id].twi_addr; len += 2;

  for (int joint = 0; joint < 2 ; joint++)
  {
    if (modules_[mod_id].joint[joint].pid_update == PID_NONE)
      fprintf(stderr,"WARNING: Sending PID for module %d but joint %d has not had PID values set.\n", mod_id, joint);

    *(int16_t*)(&pid_msg[len]) = modules_[mod_id].joint[joint].p;      len += 2;
    *(int16_t*)(&pid_msg[len]) = modules_[mod_id].joint[joint].i;      len += 2;
    *(int16_t*)(&pid_msg[len]) = modules_[mod_id].joint[joint].d;      len += 2;

    *(int16_t*)(&pid_msg[len]) = modules_[mod_id].joint[joint].i_min;  len += 2;
    *(int16_t*)(&pid_msg[len]) = modules_[mod_id].joint[joint].i_max;  len += 2;
    *(int16_t*)(&pid_msg[len]) = modules_[mod_id].joint[joint].e_min;  len += 2;
    *(int16_t*)(&pid_msg[len]) = modules_[mod_id].joint[joint].offset; len += 2;
  }

  pid_msg[len++] = '>';
  pid_msg[len++] = '\n';
  pid_msg[len] = '\0';

  {
    boost::mutex::scoped_lock(outgoing_mutex_);
    outgoing_queue_.push(IOPacket(pid_msg, len));
  }

  modules_[mod_id].joint[JOINT_A].pid_update = PID_SENT;
  modules_[mod_id].joint[JOINT_B].pid_update = PID_SENT;
}


void MasterModule::sendAllPIDConfigs()
{
  for (int i = 0; i < num_modules_; i++)
    sendPIDConfig(i);
}

bool MasterModule::checkAllPIDConfigs(bool verbose)
{
  bool status = true;

  for (int i = 0; i < num_modules_; i++)
  {
    for (int j = 0; j < 2; j++)
      if (modules_[i].joint[j].pid_update != PID_CONFIRMED && modules_[i].joint[j].pid_update != PID_NONE && modules_[i].joint[j].pid_update != PID_NEW)
      {
        status = false;
        if (verbose)
          fprintf(stderr,"  No confirmation for: %d/%d\n",i,j);
      }
  }
  return status;
}

void MasterModule::setJointPos(uint16_t mod_id, WhichJoint which, int16_t pos)
{
  if (mod_id >= num_modules_)
    SMARTSERVO_EXCEPT(SmartServoException, "Specified module exceeded bounds");

  if (which == NO_JOINT)
    SMARTSERVO_EXCEPT(SmartServoException, "Which joint to use for module not specified");

  modules_[mod_id].joint[which].des_pos = pos;
}

void MasterModule::sendAllJointPos()
{
  char pos_msg[256];
  int len = 0;

  pos_msg[len++] = '<';
  pos_msg[len++] = 'P';
  pos_msg[len++] = num_modules_*4;

  for (int n = 0; n < num_modules_ ; n++)
    {
      int pos_A = (modules_[n].joint[JOINT_A].des_pos+4095)%4095;
      int pos_B = (modules_[n].joint[JOINT_B].des_pos+4095)%4095;

      pos_msg[len++] = (char)((pos_A)&0x00ff);
      pos_msg[len++] = (char)((pos_A)>>8);
      pos_msg[len++] = (char)((pos_B)&0x00ff);
      pos_msg[len++] = (char)((pos_B)>>8);
    }

  pos_msg[len++] = '>';
  pos_msg[len++] - '\n';
  pos_msg[len] = '\0';

  {
    boost::mutex::scoped_lock(outgoing_mutex_);
    outgoing_queue_.push(IOPacket(pos_msg, len));
  }

}

int16_t MasterModule::getJointPos(uint16_t mod_id, WhichJoint which) const
{
  if (mod_id >= num_modules_)
    SMARTSERVO_EXCEPT(SmartServoException, "Specified module exceeded bounds");

  if (which == NO_JOINT)
    SMARTSERVO_EXCEPT(SmartServoException, "Which joint to use for module not specified");

  return modules_[mod_id].joint[which].cur_pos;
}

void MasterModule::setJointType(uint16_t mod_id, JointType type)
{
    if (mod_id >= num_modules_)
      SMARTSERVO_EXCEPT(SmartServoException, "Specified module exceeded bounds");

    if (type == NO_JOINT_TYPE)
      SMARTSERVO_EXCEPT(SmartServoException, "Tried to specify module %d to have no joint type", mod_id);

    // Don't care if it's already set to appropriate type
    if (modules_[mod_id].joint_type == type)
      return;

    // If no type was previously set, or is an upgrade then set and return
    if (modules_[mod_id].joint_type == NO_JOINT_TYPE || modules_[mod_id].joint_type == SMART_SERVO)
    {
      modules_[mod_id].joint_type = type;
      return;
    }

    // This is ok since SMART_SERVO always applicable
    if (type == SMART_SERVO)
      return;

    SMARTSERVO_EXCEPT(SmartServoException, "Tried setting joint_type to %d, but was %d", type, modules_[mod_id].joint_type);
}

void MasterModule::setJointServoPos(uint16_t mod_id, WhichJoint which, uint8_t pos)
{
    if (mod_id >= num_modules_)
      SMARTSERVO_EXCEPT(SmartServoException, "Specified module exceeded bounds");

    if (modules_[mod_id].joint_type == NO_JOINT_TYPE ||
        modules_[mod_id].joint_type == SMART_SERVO)
      SMARTSERVO_EXCEPT(SmartServoException, "Tried to set servo position on a non-servo joint");

    modules_[mod_id].servo[which].des_pos      = pos;
    modules_[mod_id].servo[which].servo_update = SERVO_NEW;
}

uint8_t MasterModule::getJointServoPos(uint16_t mod_id, WhichJoint which) const
{
    if (mod_id >= num_modules_)
      SMARTSERVO_EXCEPT(SmartServoException, "Specified module exceeded bounds");

    if (modules_[mod_id].joint_type == NO_JOINT_TYPE ||
        modules_[mod_id].joint_type == SMART_SERVO)
      SMARTSERVO_EXCEPT(SmartServoException, "Tried to set servo position on a non-servo joint");

    return modules_[mod_id].servo[which].des_pos;
}

void MasterModule::sendJointServoPos(uint16_t mod_id)
{
  VERIFY_PORT_OPENED();

  if (mod_id >= num_modules_)
    SMARTSERVO_EXCEPT(SmartServoException, "Specified module exceeded bounds");

  if (modules_[mod_id].joint_type != HOBBY_SERVO &&
      modules_[mod_id].joint_type != V1_SERVO)
    SMARTSERVO_EXCEPT(SmartServoException, "Tried to set servo position on a non-servo joint");

  // If these positions have already been sent, we are done
  if (modules_[mod_id].servo[JOINT_A].servo_update == SERVO_SENT &&
      modules_[mod_id].servo[JOINT_B].servo_update == SERVO_SENT)
    return;

  // If these positions have not been set
  if (modules_[mod_id].servo[JOINT_A].servo_update == SERVO_NONE &&
      modules_[mod_id].servo[JOINT_B].servo_update == SERVO_NONE)
    return;

  char servo_msg[256];
  int len = 0;

  servo_msg[len++] = '<';

  switch (modules_[mod_id].joint_type)
  {
  case HOBBY_SERVO:
    servo_msg[len++] = 'S';
    break;
  case V1_SERVO:
    servo_msg[len++] = 'V';
    break;
  }

  servo_msg[len++] = (uint8_t)(3); // Message length
  servo_msg[len++] = modules_[mod_id].twi_addr;

  servo_msg[len++] = modules_[mod_id].servo[JOINT_A].des_pos;
  servo_msg[len++] = modules_[mod_id].servo[JOINT_B].des_pos;

  servo_msg[len++] = '>';
  servo_msg[len++] = '\n';
  servo_msg[len] = '\0';

  {
    boost::mutex::scoped_lock(outgoing_mutex_);
    outgoing_queue_.push(IOPacket(servo_msg, len));
  }

  modules_[mod_id].servo[JOINT_A].servo_update = SERVO_SENT;
  modules_[mod_id].servo[JOINT_B].servo_update = SERVO_SENT;

  switch (modules_[mod_id].joint_type)
  {
  case HOBBY_SERVO:
    s_transmits_++;
    break;
  case V1_SERVO:
    v_transmits_++;
    break;
  }
}

void  MasterModule::sendAllJointServoPos()
{
  VERIFY_PORT_OPENED();

  if (v_confirmations_ != v_transmits_)
    fprintf(stderr,"WARNING: V servo confirmation/transmit disparity: %d/%d\n", v_confirmations_, v_transmits_);

  if (s_confirmations_ != s_transmits_)
    fprintf(stderr,"WARNING: S servo confirmation/transmit disparity: %d/%d\n", s_confirmations_, s_transmits_);

  for (int i = 0; i < num_modules_; i++)
  {
    if (modules_[i].joint_type == HOBBY_SERVO || modules_[i].joint_type == V1_SERVO)
      sendJointServoPos(i);
  }
}

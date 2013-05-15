
#include <iostream>
#include <math.h>
#include <stdio.h>

#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/thread.hpp>

#include "bandit/bandit.h"

bandit::Bandit g_bandit;
bool g_running;

boost::mutex g_bandit_mutex;

void dispState()
{
  /*
  std::cout << "Pos update: ";
  for (int i = 0; i < 19; i++)
    std::cout << g_bandit.getJointPos(i) << ", ";
  std::cout << std::endl;
  */
}

void spin()
{
  while ( g_running )
  {
    g_bandit.processIO(10000);
    g_bandit.processPackets();
  }
}

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cout << "Please specify port at the command line" << std::endl;
    return -1;
  }

  bandit::Bandit b;
  try
  {
    g_bandit.openPort(argv[1]);
    
    g_bandit.registerStateCB(boost::bind(&dispState));

    // Spin up thread to process incoming bandit messages
    g_running = true;
    boost::thread t(boost::bind(&spin));

    for (int i = 0; i < 19; i++)
    {
      if (g_bandit.getJointType(i) == smartservo::SMART_SERVO)
        g_bandit.setJointPIDConfig(i, 100, 0, 0, -4001, 4001, 50, 0);
    }

    do
    {
      std::cout << "Sending PID configurations..." << std::endl;
      g_bandit.sendAllPIDConfigs();
      usleep(5000000);
    }  while (!g_bandit.checkAllPIDConfigs(true));
    
    // Set bandit to "neutral position:
    g_bandit.setJointPos(0,  0.0); //"head pitch",        
    g_bandit.setJointPos(1,  0.0); //"head pan",          
    g_bandit.setJointPos(2,  0.0); //"left shoulder F/B", 
    g_bandit.setJointPos(3,  0.0); //"left shoulder I/O", 
    g_bandit.setJointPos(4,  0.0); //"left elbow twist",  
    g_bandit.setJointPos(5,  0.0); //"left elbow",        
    g_bandit.setJointPos(6,  0.0); //"left wrist twist",  
    g_bandit.setJointPos(7,  0.5);  //"left wrist tilt",   
    g_bandit.setJointPos(8,  0.5);  //"left hand grab",    
    g_bandit.setJointPos(9,  0.0); //"right shoulder F/B",
    g_bandit.setJointPos(10, 0.0); // "right shoulder I/O"
    g_bandit.setJointPos(11, 0.0); // "right elbow twist",
    g_bandit.setJointPos(12, 0.0); // "right elbow",      
    g_bandit.setJointPos(13, 0.0); // "right wrist twist",
    g_bandit.setJointPos(14, 0.5);  // "right wrist tilt", 
    g_bandit.setJointPos(15, 0.5);  // "right hand grab",  
    g_bandit.setJointPos(16, 0.5);  // "eyebrows",         
    g_bandit.setJointPos(17, 0.5);  // "mouth top",        
    g_bandit.setJointPos(18, 0.5);  // "mouth bottom", 

    // Send bandit position commands:
    g_bandit.sendAllJointPos();

    g_running = true;



    std::string input;

    std::cout << "Verify bandit is in neutral position and hit enter to continue.\n";
    getchar();


    for (int i = 0; i < 19; i++)
    {

      std::cout << "Bandit is about to move his " << g_bandit.getJointName(i) << ".  Please hit enter to continue.";
      getchar();

      double nominal;
      double high;
      double low;

      if (g_bandit.getJointType(i) == smartservo::SMART_SERVO)
      {
        nominal = 0;
        high = 0.3;
        low = -0.3;
      } 
      else
      {
        nominal = 0.5;
        high = 0.6;
        low = 0.4;
      }

      g_bandit_mutex.lock();
      g_bandit.setJointPos(i,  low);
      g_bandit.sendAllJointPos();
      g_bandit_mutex.unlock();
      usleep(500000);
      g_bandit_mutex.lock();
      g_bandit.setJointPos(i,  high);
      g_bandit.sendAllJointPos();
      g_bandit_mutex.unlock();
      usleep(500000);
      g_bandit.setJointPos(i,  nominal);
      g_bandit.sendAllJointPos();

    }



    g_running = false;

    t.join();
        
  } catch (bandit::BanditException e)
  {
    std::cout << "Bandit threw exception: " << e.what() << std::endl;
    g_bandit.closePort();
  }
}

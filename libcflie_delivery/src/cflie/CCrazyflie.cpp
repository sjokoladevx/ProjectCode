// Copyright (c) 2013, Jan Winkler <winkler@cs.uni-bremen.de>
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Universität Bremen nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "cflie/CCrazyflie.h"


void CCrazyflieConstructor(CCrazyRadio *crRadio,CCrazyflie* crFile) {
  crFile->m_crRadio = crRadio;
  
  // Review these values
  crFile->m_fMaxAbsRoll = 45.0f;
  crFile->m_fMaxAbsPitch = crFile->m_fMaxAbsRoll;
  crFile->m_fMaxYaw = 2 * M_PI;
  crFile->m_nMaxThrust = 60000;
  crFile->m_nMinThrust = 0;//15000;

  crFile->m_fRoll = 0;
  crFile->m_fPitch = 0;
  crFile->m_fYaw = 0;
  crFile->m_nThrust = 0;
  
  crFile->m_bSendsSetpoints = false;
  crFile->m_setHoverPoint = false;
  
  crFile->m_tocParameters = new CTOC;
  CTOCConstructor(crFile->m_tocParameters,crFile->m_crRadio , 2);
  crFile->m_tocLogs = new CTOC; 
  CTOCConstructor(crFile->m_tocLogs,crFile->m_crRadio, 5);

  crFile->m_enumState = STATE_ZERO;
  
  crFile->m_dSendSetpointPeriod = 0.01; // Seconds
  crFile->m_dSetpointLastSent = 0;

}

void CCrazyflieDestructor(CCrazyflie* crFile) {
  stopLogging(crFile);
}

bool readTOCParameters(CCrazyflie* crFile) {
  if(requestMetaData(crFile->m_tocParameters)) {
    if(requestItems(crFile->m_tocParameters)) {
      return true;
    }
  }
  
  return false;
}

bool readTOCLogs(CCrazyflie* crFile) {
  if(requestMetaData(crFile->m_tocLogs)) {
    if(requestItems(crFile->m_tocLogs)) {
      return true;
    }
  }
  
  return false;
}



bool sendParam(CCrazyflie* crFile,int8_t althold) {
   unsigned char varid = idForName(crFile->m_tocParameters,"flightmode.althold");
      //CS50_TODO
      //*pseudocode*
        /*
            Checkout sendSetpoint function, it is similiar to the sendSetpoint function
            But the char cBuffer[nSize]; should have the an "unsigned char" type
            varid (as the id of the flightmode.althold) and followed by a "int8_t"
            variable (1 means enable the hover mode and 0 meas disable the hover mode)
            Then initilaize it with Port 2, Channel 2 packets send this packet to 
            the copter
        */
    //*pseudocode* 

//int8_t althold=1;this will send out a packet which will turn hover on, on its current altitude
//int8_t althold=0;this will send out a packet which will turn it off
 memcpy(&cBuffer[0], &varid, sizeof(unsigned char));
 memcpy(&cBuffer[1 * sizeof(unsigned char)], &althold, sizeof(int8_t));


  CCRTPPacket *crtpPacket = new CCRTPPacket;
  CCRTPPacketInit3(crtpPacket,cBuffer, nSize, 2);

// PortTargetUsed for
// 0 Console Read console text that is printed to the console on the Crazyflie using consoleprintf
// 2 Parameters  Get/set parameters from the Crazyflie. Parameters are defined using a macro in the Crazyflie source-code
// 3 Commander Sending control set-points for the roll/pitch/yaw/thrust regulators
// 5 Log Set up log blocks with variables that will be sent back to the Crazyflie at a specified period. Log variables are defined using a macro in the Crazyflie source-code
// 14  Client-side debugging Debugging the UI and exists only in the Crazyflie Python API and not in the Crazyflie itself.
// 15  Link layer  Used to control and query the communication link


  setPort(crtpPacket, 2);
  setChannel(crtpPacket, 2);

  CCRTPPacket *crtpReceived = sendPacket(crFile->m_crRadio,crtpPacket);
  
  delete crtpPacket;
  if(crtpReceived != NULL) {
    delete crtpReceived;
    return true;
  } else {
    return false;
  }
}

bool sendSetpoint(CCrazyflie* crFile,float fRoll, float fPitch, float fYaw, short sThrust) {
  fPitch = -fPitch;
  
  int nSize = 3 * sizeof(float) + sizeof(short);
  char cBuffer[nSize];
  memcpy(&cBuffer[0 * sizeof(float)], &fRoll, sizeof(float));
  memcpy(&cBuffer[1 * sizeof(float)], &fPitch, sizeof(float));
  memcpy(&cBuffer[2 * sizeof(float)], &fYaw, sizeof(float));
  memcpy(&cBuffer[3 * sizeof(float)], &sThrust, sizeof(short));
  
  CCRTPPacket *crtpPacket = new CCRTPPacket;
  CCRTPPacketInit3(crtpPacket,cBuffer, nSize, 3);

  CCRTPPacket *crtpReceived = sendPacket(crFile->m_crRadio,crtpPacket);
  
  delete crtpPacket;
  if(crtpReceived != NULL) {
    delete crtpReceived;
    return true;
  } else {
    return false;
  }
}


//The main interface of setting
void setThrust(CCrazyflie* crFile,int nThrust) {
  crFile->m_nThrust = nThrust;
  if(crFile->m_nThrust < crFile->m_nMinThrust) {
    crFile->m_nThrust = crFile->m_nMinThrust;
  } else if(crFile->m_nThrust > crFile->m_nMaxThrust) {
    crFile->m_nThrust = crFile->m_nMaxThrust;
  }
}

//cycle through each state
bool cycle(CCrazyflie* crFile) {
  double dTimeNow = currentTime();
  
  switch(crFile->m_enumState) {
  case STATE_ZERO: {
    crFile->m_enumState = STATE_READ_PARAMETERS_TOC;
  } break;
    
  case STATE_READ_PARAMETERS_TOC: {
    if(readTOCParameters(crFile)) {
      crFile->m_enumState = STATE_READ_LOGS_TOC;
    }
  } break;
    
  case STATE_READ_LOGS_TOC: {
    if(readTOCLogs(crFile)) {
      crFile->m_enumState = STATE_START_LOGGING;
    }
  } break;
    
  case STATE_START_LOGGING: {
    if(startLogging(crFile)) {
      crFile->m_enumState = STATE_ZERO_MEASUREMENTS;
    }
  } break;
    
  case STATE_ZERO_MEASUREMENTS: {
   processPackets(crFile->m_tocLogs,popLoggingPackets(crFile->m_crRadio),false);
    
    // NOTE(winkler): Here, we can do measurement zero'ing. This is
    // not done at the moment, though. Reason: No readings to zero at
    // the moment. This might change when altitude becomes available.
    
    crFile->m_enumState = STATE_NORMAL_OPERATION;
  } break;
    
  case STATE_NORMAL_OPERATION: {
    // Shove over the sensor readings from the radio to the Logs TOC.
    processPackets(crFile->m_tocLogs,popLoggingPackets(crFile->m_crRadio),crFile->want_set);
    crFile->want_set=false;
    
    if(crFile->m_bSendsSetpoints) {
      // Check if it's time to send the setpoint
      if(dTimeNow - crFile->m_dSetpointLastSent > crFile->m_dSendSetpointPeriod) {
	   //CS50_TODO
     //*pseudocode*
      sendParam(crFile, hoverFlag);
        /*
            You could call the setParam function you write to enable/disable the 
            Hover mode here
        */
    //*pseudocode* 

	   sendSetpoint(crFile,crFile->m_fRoll, crFile->m_fPitch, crFile->m_fYaw, crFile->m_nThrust);
	}
  crFile->m_dSetpointLastSent = dTimeNow;
      
    } else {
      // Send a dummy packet for keepalive
      sendDummyPacket(crFile->m_crRadio);
    }
  } break;
    
  default: {
  } break;
  }
  
  if(ackReceived(crFile->m_crRadio)) {
    crFile->m_nAckMissCounter = 0;
  } else {
    crFile->m_nAckMissCounter++;
  }
  
  return usbOK(crFile->m_crRadio);
}

bool copterInRange(CCrazyflie* crFile) {
  return crFile->m_nAckMissCounter < crFile->m_nAckMissTolerance;
}

//The main interface for input
void setRoll(CCrazyflie* crFile,float fRoll) {
  crFile->m_fRoll = fRoll;
  
  if(fabs(crFile->m_fRoll) > crFile->m_fMaxAbsRoll) {
    crFile->m_fRoll = copysign(crFile->m_fMaxAbsRoll, crFile->m_fRoll);
  }
}


void setPitch(CCrazyflie* crFile,float fPitch) {
  crFile->m_fPitch = fPitch;
  
  if(fabs(crFile->m_fPitch) > crFile->m_fMaxAbsPitch) {
    crFile->m_fPitch = copysign(crFile->m_fMaxAbsPitch, crFile->m_fPitch);
  }
}

double currentTime() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000;
}

bool isInitialized(CCrazyflie* crFile) {
  return crFile->m_enumState == STATE_NORMAL_OPERATION;
}

//CS50_TODO
//*pseudocode*
        /*
            The following five functions will give you idea how to get 
            parameter/log from the copter

            Use the python client connect to the copter to checkout what else parameter 
            you can get from it
        */
//*pseudocode* 
bool startLogging(CCrazyflie* crFile) {
  enableBatteryLogging(crFile);
  return true;
}

bool stopLogging(CCrazyflie* crFile) {
  disableBatteryLogging(crFile);
  return true;
}

void setSendSetpoints(CCrazyflie* crFile,bool bSendSetpoints) {
  crFile->m_bSendsSetpoints = bSendSetpoints;
}

bool sendsSetpoints(CCrazyflie* crFile) {
  return crFile->m_bSendsSetpoints;
}

double sensorDoubleValue(CCrazyflie* crFile,string strName) {
  return doubleValue(crFile->m_tocLogs,strName);
}

void disableLogging(CCrazyflie* crFile) {
  unregisterLoggingBlock(crFile->m_tocLogs,"high-speed");
  unregisterLoggingBlock(crFile->m_tocLogs,"low-speed");
}

void enableBatteryLogging(CCrazyflie* crFile) {
  registerLoggingBlock(crFile->m_tocLogs,"battery", 1000);
  startLogging(crFile->m_tocLogs,"pm.vbat", "battery");
}

double batteryLevel(CCrazyflie* crFile) {
  return sensorDoubleValue(crFile,"pm.vbat");
}

void disableBatteryLogging(CCrazyflie* crFile) {
  unregisterLoggingBlock(crFile->m_tocLogs,"battery");
}


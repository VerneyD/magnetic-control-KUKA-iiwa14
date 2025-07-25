/**

The following license terms and conditions apply, unless a redistribution
agreement or other license is obtained by KUKA Deutschland GmbH, Augsburg, Germany.

SCOPE

The software �KUKA Sunrise.FRI Client SDK� is targeted to work in
conjunction with the �KUKA Sunrise.FRI� toolkit.
In the following, the term �software� refers to all material directly
belonging to the provided SDK �Software development kit�, particularly source
code, libraries, binaries, manuals and technical documentation.

COPYRIGHT

All Rights Reserved
Copyright (C)  2014-2022 
KUKA Deutschland GmbH
Augsburg, Germany

LICENSE 

Redistribution and use of the software in source and binary forms, with or
without modification, are permitted provided that the following conditions are
met:
a) The software is used in conjunction with KUKA products only. 
b) Redistributions of source code must retain the above copyright notice, this
list of conditions and the disclaimer.
c) Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the disclaimer in the documentation and/or other
materials provided with the distribution. Altered source code of the
redistribution must be made available upon request with the distribution.
d) Modification and contributions to the original software provided by KUKA
must be clearly marked and the authorship must be stated.
e) Neither the name of KUKA nor the trademarks owned by KUKA may be used to
endorse or promote products derived from this software without specific prior
written permission.

DISCLAIMER OF WARRANTY

The Software is provided "AS IS" and "WITH ALL FAULTS," without warranty of
any kind, including without limitation the warranties of merchantability,
fitness for a particular purpose and non-infringement. 
KUKA makes no warranty that the Software is free of defects or is suitable for
any particular purpose. In no event shall KUKA be responsible for loss or
damages arising from the installation or use of the Software, including but
not limited to any indirect, punitive, special, incidental or consequential
damages of any character including, without limitation, damages for loss of
goodwill, work stoppage, computer failure or malfunction, or any and all other
commercial damages or losses. 
The entire risk to the quality and performance of the Software is not borne by
KUKA. Should the Software prove defective, KUKA is not liable for the entire
cost of any service and repair.



\file
\version {2.5}
*/
#ifndef _KUKA_FRI_LBR_CLIENT_H
#define _KUKA_FRI_LBR_CLIENT_H

#include "friClientIf.h"
#include "friLBRState.h"
#include "friLBRCommand.h"

/** Kuka namespace */
namespace KUKA
{
namespace FRI
{

   /**
    * \brief Implementation of the IClient interface for the KUKA LBR (lightweight) robots.
    * 
    * Provides access to the current LBR state and the possibility to send new 
    * commands to the LBR.  
    */
   class LBRClient : public IClient
   {
   
   public:
      
      /** \brief Constructor. */
      LBRClient();
      
      /** \brief Destructor. */
      ~LBRClient();
         
      /** 
       * \brief Callback that is called whenever the FRI session state changes.
       * 
       * @param oldState previous FRI session state
       * @param newState current FRI session state
       */
      virtual void onStateChange(ESessionState oldState, ESessionState newState);
   
      /**
       * \brief Callback for the FRI session states 'Monitoring Wait' and 'Monitoring Ready'.
       */
      virtual void monitor();
   
      /**
       * \brief Callback for the FRI session state 'Commanding Wait'.
       */
      virtual void waitForCommand();
   
      /**
       * \brief Callback for the FRI session state 'Commanding'.
       */
      virtual void command();
   
      /**
       * \brief Provide read access to the current robot state.
       * 
       * @return Reference to the LBRState instance
       */
      const LBRState& robotState() const { return _robotState; }
      
      /**
       * \brief Provide write access to the robot commands.
       * 
       * @return Reference to the LBRCommand instance
       */
      LBRCommand& robotCommand() { return _robotCommand; }
      
   private:
      
      LBRState _robotState;      //!< wrapper class for the FRI monitoring message
      LBRCommand _robotCommand;  //!< wrapper class for the FRI command message
      
      /**
       * \brief Method to create and initialize the client data structure (used internally).
       * 
       * @return newly allocated client data structure
       */
      virtual ClientData* createData();
      
   };
   
}
}


#endif // _KUKA_FRI_LBR_CLIENT_H

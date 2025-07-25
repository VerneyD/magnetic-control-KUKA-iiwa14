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
#include "friLBRState.h"
#include "friLBRCommand.h"
#include "friClientData.h"
#include "pb_frimessages_callbacks.h"
#include "friDataHelper.h"

using namespace KUKA::FRI;

//******************************************************************************
void LBRCommand::setJointPosition(const double* const values)
{
   _cmdMessage->has_commandData = true;
   _cmdMessage->commandData.has_jointPosition = true;
   const tRepeatedDoubleArguments* const dest =
            static_cast<tRepeatedDoubleArguments*>(_cmdMessage->commandData.jointPosition.value.arg);
   memcpy(dest->value, values, LBRState::NUMBER_OF_JOINTS * sizeof(double));
}

//******************************************************************************
void LBRCommand::setWrench(const double* const wrench)
{
   _cmdMessage->has_commandData = true;
   _cmdMessage->commandData.has_cartesianWrenchFeedForward = true;

   double* const dest = _cmdMessage->commandData.cartesianWrenchFeedForward.element;
   memcpy(dest, wrench, 6 * sizeof(double));
}

//******************************************************************************
void LBRCommand::setTorque(const double* const torques)
{
   _cmdMessage->has_commandData = true;
   _cmdMessage->commandData.has_jointTorque= true;

   const tRepeatedDoubleArguments* const dest = static_cast<tRepeatedDoubleArguments*>(_cmdMessage->commandData.jointTorque.value.arg);
   memcpy(dest->value, torques, LBRState::NUMBER_OF_JOINTS * sizeof(double));
}

//******************************************************************************
void LBRCommand::setCartesianPose(const double* const cartesianPoseQuaternion,
    double const *const redundancyValue)
{
   _cmdMessage->has_commandData = true;
   _cmdMessage->commandData.has_cartesianPose = true;
   _cmdMessage->commandData.cartesianPose.name[0] = '\0';
   memcpy(_cmdMessage->commandData.cartesianPose.element, cartesianPoseQuaternion, 7 * sizeof(double));

   _cmdMessage->commandData.has_redundancyInformation = true;
   _cmdMessage->commandData.redundancyInformation.strategy = 
       _monMessage->monitorData.measuredRedundancyInformation.strategy;
 
   if (NULL != redundancyValue)
   {
      //set value if provided
      
      _cmdMessage->commandData.redundancyInformation.value = *redundancyValue;
   }
   else
   {
      // use interpolated redundancy value if no value is commanded
      _cmdMessage->commandData.redundancyInformation.value = _monMessage->ipoData.redundancyInformation.value;
   }
}

//******************************************************************************
void LBRCommand::setCartesianPoseAsMatrix(const double(&measuredCartesianPose)[3][4],
    double const *const redundancyValue)
{
   double quaternion[7];
   DataHelper::convertTrafoMatrixToQuaternion(measuredCartesianPose, quaternion);
   setCartesianPose(quaternion, redundancyValue);
}

//******************************************************************************
void LBRCommand::setBooleanIOValue(const char* const name, const bool value)
{
   ClientData::setBooleanIOValue(_cmdMessage, name, value, _monMessage);
}

//******************************************************************************
void LBRCommand::setAnalogIOValue(const char* const name, const double value)
{
   ClientData::setAnalogIOValue(_cmdMessage, name, value, _monMessage);
}

//******************************************************************************
void LBRCommand::setDigitalIOValue(const char* const name, const unsigned long long value)
{
   ClientData::setDigitalIOValue(_cmdMessage, name, value, _monMessage);
}

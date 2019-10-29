/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OI.h"

#include <frc/WPILib.h>

#include <commands/sequenceLoad.h>
#include <commands/wristGoToLevel.h>
#include <commands/intakeRollersStop.h>
#include <commands/intakeRollersIn.h>
#include <commands/intakeRollersOut.h>
#include <commands/sequenceLeveOneScore.h>
#include <commands/sequenceLevelTwoScore.h>
#include <commands/sequenceLevelThreeScore.h>
#include <commands/baseLockRotation.h>
#include <commands/baseControl.h>
#include <commands/baseNerfControl.h>
#include <commands/sequenceBackOff.h>
#include <commands/sequenceBackOffMedium.h>
#include <commands/armPneuControlUp.h>
#include <commands/armPneuControlDown.h>
#include <commands/armControl.h>
#include <commands/intakeProbe.h>
#include <commands/armPneuControlDown.h>
#include <commands/armPneuControlUp.h>
#include <commands/armGoToTwo.h>
#include <commands/armGoToZero.h>
#include <commands/limelightModeDrive.h>
#include <commands/limelightModeTrack.h>
#include <commands/liftFrontSet.h>
#include <commands/liftRearSet.h>

OI::OI()
{
    x->WhenReleased(new armControl());
    lb->WhenReleased(new baseNerfControl());
    rb->WhenReleased(new baseControl());

    load->WhenReleased(new sequenceLoad());
    lv1->WhenReleased(new sequenceLeveOneScore());
    in->WhenReleased(new intakeRollersIn());
    out->WhenReleased(new intakeRollersOut());
    pneuUp->WhenReleased(new armPneuControlUp());
    pneuDown->WhenReleased(new armPneuControlDown());
    mid->WhenReleased(new wristGoToLevel(1));
    down->WhenReleased(new wristGoToLevel(2));
    stp->WhenReleased(new intakeRollersStop());
    armUp->WhenReleased(new armGoToTwo());
    armDown->WhenReleased(new armGoToZero());
    reset->WhenReleased(new sequenceBackOff());
    probe->WhenReleased(new intakeProbe());
    tog->WhenPressed(new limelightModeDrive());
    tog->WhenReleased(new limelightModeTrack());
    FRplus->WhenReleased(new liftFrontSet(1));
    FRminus->WhenReleased(new liftFrontSet(0));
    REplus->WhenReleased(new liftRearSet(1));
    REminus->WhenReleased(new liftRearSet(0));

    
    
}

package org.firstinspires.ftc.teamcode.Robot;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Robot;

public class SolRobot extends Robot {

    // ... in MyRobot.java (extends Robot)

    // enum to specify opmode type
    public enum OpModeType {
        TELEOP, AUTO
    }

    // the constructor with a specified opmode type
    public SolRobot(OpModeType type) {
        if (type == OpModeType.TELEOP) {
            initTele();
        } else {
            initAuto();
        }
    }

    /*
     * Initialize teleop or autonomous, depending on which is used
     */
    public void initTele() {
        // initialize teleop-specific scheduler
        CommandScheduler.getInstance().enable();
    }

    public void initAuto() {
        // initialize auto-specific scheduler
        CommandScheduler.getInstance().enable();
    }
}

package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot.BetterDashboard;

public class Shooter extends SubsystemBase {
    DcMotor shooter;
    public Shooter(HardwareMap hardwareMap){
        shooter = hardwareMap.get(DcMotor.class,"shooter");
    }

    public void resetShooter(){
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void shooterTelem(BetterDashboard betterDashboard) {
        betterDashboard.sendTelem("shooterpow",shooter.getPower());
    }

    public enum ShooterStates {
        MAXSPEED,
        HALFSPEED,
        STOP
    }
}

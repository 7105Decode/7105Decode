package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot.BetterDashboard;

public class Shooter extends SubsystemBase {
    public DcMotor shooter;
    public static double MaxSpinSpeed = 1, HalfSpinSpeed = .5;
    public Shooter(HardwareMap hardwareMap){
        shooter = hardwareMap.get(DcMotor.class,"shooter");
    }
    public void resetShooter(){
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void shooterTelem() {
        BetterDashboard.sendTelem("shooterpower",shooter.getPower());
    }
    public void setShooterStates(ShooterStates shooterStates){
        switch (shooterStates){
            case MAXSPEED:
                shooter.setPower(MaxSpinSpeed);
                break;
            case HALFSPEED:
                shooter.setPower(HalfSpinSpeed);
                break;
            case STOP:
                shooter.setPower(0);
                break;
        }
    }
    public enum ShooterStates {
        MAXSPEED,
        HALFSPEED,
        STOP
    }
}

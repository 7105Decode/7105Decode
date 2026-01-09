package org.firstinspires.ftc.teamcode.Robot.Subsystems;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Robot.BetterDashboard;

//import org.firstinspires.ftc.teamcode.Robot.BetterDashboard;

@Configurable
public class Shooter extends SubsystemBase {
    public DcMotor shooter;
    public static double MaxSpinSpeed = 1, HalfSpinSpeed = .5;
    public Shooter(HardwareMap hardwareMap){
        shooter = hardwareMap.get(DcMotor.class,"rightshooter");
    }
    public void resetShooter(){
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {
        shooterTelem();
    }

    public void shooterTelem() {
        BetterDashboard.addtelem("shooterpower", shooter.getPower());
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

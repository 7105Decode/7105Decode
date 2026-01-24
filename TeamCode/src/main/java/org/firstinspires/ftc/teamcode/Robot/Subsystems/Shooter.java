package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import org.firstinspires.ftc.teamcode.Robot.BetterPanels;
@Configurable
public class Shooter extends Subsystem {
    public static final Shooter INSTANCE = new Shooter();
    private Shooter() { }
    public MotorEx rightshooter,leftshooter;
//    MotorGroup shooterGroup;
    public static double MaxSpinSpeed = 1, HalfSpinSpeed = .5;
    public String rightshootername = "rightshooter", leftshootername = "leftshooter";

    @Override
    public void initialize() {
        rightshooter = new MotorEx(rightshootername);
        leftshooter = new MotorEx(leftshootername);
//        shooterGroup = new MotorGroup(leftshootername, rightshootername);
    }
    @Override
    public void periodic() {
        shooterTelem();
    }
    public void resetShooter(){
        leftshooter.resetEncoder();
    }

    public double shooterVel(){
        return leftshooter.getVelocity();
    }


    public void shooterTelem() {
        BetterPanels.addtelem("shootervel", shooterVel());
    }
    public void setShooterStates(ShooterStates shooterStates){
        switch (shooterStates){
            case MAXSPEED:
                rightshooter.setPower(MaxSpinSpeed);
                break;
            case HALFSPEED:
                rightshooter.setPower(HalfSpinSpeed);
                break;
            case STOP:
                rightshooter.setPower(0);
                break;
        }
    }
    public enum ShooterStates {
        MAXSPEED,
        HALFSPEED,
        STOP
    }
}

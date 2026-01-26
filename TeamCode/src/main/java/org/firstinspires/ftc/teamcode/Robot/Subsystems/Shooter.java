package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.bylazar.configurables.annotations.Configurable;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import org.firstinspires.ftc.teamcode.Robot.BetterPanels;
@Configurable
public class Shooter extends Subsystem {
    public static final Shooter INSTANCE = new Shooter();
    private Shooter() { }
    public MotorEx rightshooter,leftshooter;

    BasicPID shooterpid;
    PIDCoefficients coefficients;
    public static double MaxSpinSpeed = 1, HalfSpinSpeed = .5, kp = 0.03, ki = 0, kd = 0, targetvel = -2280;
    public String rightshootername = "rightshooter", leftshootername = "leftshooter";

    @Override
    public void initialize() {
        coefficients = new PIDCoefficients(kp,ki,kd);
        shooterpid = new BasicPID(coefficients);
        rightshooter = new MotorEx(rightshootername);
        leftshooter = new MotorEx(leftshootername);
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

    public double setControllerValue(double value){
        kp = value;
        return kp;
    }
    public double setTargetVel(double referencevel){
        targetvel = referencevel;
        return targetvel;
    }
    public void runPController(){
        rightshooter.setPower(-1*shooterpid.calculate(targetvel,shooterVel()));
        leftshooter.setPower(-1*shooterpid.calculate(targetvel,shooterVel()));
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

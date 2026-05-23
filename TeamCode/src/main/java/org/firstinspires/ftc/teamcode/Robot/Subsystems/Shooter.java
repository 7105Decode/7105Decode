package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

@Configurable
public class Shooter extends Subsystem {
    public static final Shooter INSTANCE = new Shooter();
    private Shooter() { }
    public MotorEx rightshooter,leftshooter;
    public Servo hood;
    public static double hoodUp = .965,hoodDown = .055,MaxSpinSpeed = 1, HalfSpinSpeed = .5,
            feedforwardlong = .88,feedforwardshort = .62,kp = 0.002, targetvel = -2280, feedforward = 0;
    public String rightshootername = "rightshooter", leftshootername = "leftshooter";
    public static boolean runShooter = false, slowerSpeed = false;

    @Override
    public void initialize() {
        rightshooter = new MotorEx(rightshootername);
        leftshooter = new MotorEx(leftshootername);
        leftshooter.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightshooter.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hood = OpModeData.INSTANCE.getHardwareMap().get(Servo.class,"hood");
        runShooter = false;
    }
    @Override
    public void periodic() {
        if (runShooter) {
            calculatePF();
        } else if (slowerSpeed) {
            leftshooter.setPower(.6);
            rightshooter.setPower(.6);
        } else {
            setPower(0);
        }
    }
    public void resetShooter(){
        leftshooter.resetEncoder();
    }

    public double shooterVel(){
        return leftshooter.getVelocity();
    }

    public double shooterVelError(double targetvel){
        return (targetvel - shooterVel()) *-1;
    }

    public double setControllerValue(double value){
        kp = value;
        return kp;
    }
    public double setTargetVel(double referencevel){
        targetvel = referencevel;
        return targetvel;
    }
    public void setTargetFeedForward(double feedforward){
        this.feedforward = feedforward;
    }
    public void setPower(double power){
        rightshooter.setPower(power);
        leftshooter.setPower(power);
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

    public void calculatePF(){
         setPower((shooterVelError(targetvel) * kp) + feedforward);
    }
    public enum ShooterStates {
        MAXSPEED,
        HALFSPEED,
        STOP
    }
}

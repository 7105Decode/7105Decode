package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

import org.firstinspires.ftc.teamcode.Robot.BetterPanels;
@Configurable
public class Intake extends Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake() { }
    public MotorEx intake;
    public static double intakeSpeed = 1;
    public String intakename = "frontintake";
    @Override
    public void initialize() {
        intake = new MotorEx(intakename);
    }
    @Override
    public void periodic() {
    }
    public void intakeTelem() {
    }
    public void runIntakeTele(Gamepad gamepad1){

    }
}

package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

@Configurable
public class Intake extends Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake() { }
    public static MotorEx intake;
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
    public static void runIntakeTele(Gamepad gamepad1){
        if (gamepad1.left_trigger > .4){
            intake.setPower(-intakeSpeed);
        } else if (gamepad1.right_trigger > .4) {
            intake.setPower(intakeSpeed);
        } else {
            intake.setPower(0);
        }
    }
}

package org.firstinspires.ftc.teamcode.Opmodes;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import org.firstinspires.ftc.teamcode.Robot.MoreConvenientTelemetry;
import org.firstinspires.ftc.teamcode.Robot.Commands.RunDriveTrainTeleop;
import org.firstinspires.ftc.teamcode.Robot.Commands.RunIntakeTele;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

@TeleOp
@Configurable
public class CommandSchedulerTeleop extends PedroOpMode {

    public CommandSchedulerTeleop() {
        super(Transfer.INSTANCE, Turret.INSTANCE, DriveTrain.INSTANCE, MoreConvenientTelemetry.INSTANCE, Shooter.INSTANCE, Intake.INSTANCE);
        MoreConvenientTelemetry.INSTANCE.setTelemetry(telemetry);
//        DriveTrain.INSTANCE.createFollower(hardwareMap);
    }
    Command runRobot;
    @Override
    public void onInit() {
        DriveTrain.INSTANCE.createFollower(hardwareMap);
    }

    @Override
    public void onWaitForStart() {
        runRobot = new ParallelGroup(

                new RunDriveTrainTeleop(DriveTrain.INSTANCE,gamepad1),

                new RunIntakeTele(gamepad1)
        );
    }

    @Override
    public void onStartButtonPressed() {
        runRobot.invoke();
//        gamepadManager.getGamepad2().getDpadUp().setReleasedCommand(Claw.INSTANCE::open);
    }

    @Override
    public void onUpdate() {
        MoreConvenientTelemetry.telemetryM.update();
        DriveTrain.INSTANCE.updateFollower();
    }
}

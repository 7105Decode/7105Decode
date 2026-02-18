package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

public class ReadObelisk extends Command {
    Telemetry telemetry;
    int pipeline;
    public ReadObelisk(int pipeline, Telemetry telemetry){
        this.pipeline = pipeline;
        this.telemetry = telemetry;
    }

    @Override
    public void start() {
        Turret.INSTANCE.limelight.pipelineSwitch(pipeline);
        Turret.INSTANCE.limelight.start();
        Turret.GPP = false;
        Turret.PPG = false;
        Turret.PGP = false;
    }

    @Override
    public void update() {
        Turret.INSTANCE.readObelisk(telemetry);
    }

    @Override
    public boolean isDone() {
        return true;
    }
}

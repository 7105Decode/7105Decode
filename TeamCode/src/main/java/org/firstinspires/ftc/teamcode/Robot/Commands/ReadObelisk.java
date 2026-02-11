package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

public class ReadObelisk extends Command {
    Turret turret;
    Telemetry telemetry;
    public ReadObelisk(Turret turret, Telemetry telemetry){
        this.turret = turret;
        this.telemetry = telemetry;
    }

    @Override
    public void start() {
        turret.limelight.start();
        Turret.GPP = false;
        Turret.PPG = false;
        Turret.PGP = false;
    }

    @Override
    public void update() {
        turret.readObelisk(telemetry);
    }

    @Override
    public boolean isDone() {
        return true;
    }
}

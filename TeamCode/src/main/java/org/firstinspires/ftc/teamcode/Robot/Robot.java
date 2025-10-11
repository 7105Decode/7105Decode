package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

public class Robot extends com.arcrobotics.ftclib.command.Robot {
    public Shooter shooter;
    public DriveTrain driveTrain;
    public BetterDashboard dashboard;
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Pose2d pose2d){
        dashboard = new BetterDashboard(telemetry);
        driveTrain = new DriveTrain(hardwareMap,pose2d,dashboard);
        shooter = new Shooter();
    }

    @Override
    public void register(Subsystem... subsystems) {
        dashboard.register();
        driveTrain.register();
        shooter.register();
    }


}

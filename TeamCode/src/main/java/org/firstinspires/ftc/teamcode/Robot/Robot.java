package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

public class Robot {
    public Shooter shooter;
    public DriveTrain driveTrain;
    public BetterDashboard dashboard;
    public Robot(HardwareMap hardwareMap, Pose pose){
        dashboard = new BetterDashboard();
        driveTrain = new DriveTrain(hardwareMap,pose);
        shooter = new Shooter(hardwareMap);
    }

    public void updateRobotInit(){
        driveTrain.driveTrainTelem();
        shooter.shooterTelem();
        dashboard.initloopupdate();
    }
    public void updateRobotRun(){
        driveTrain.driveTrainTelem();
        shooter.shooterTelem();
        dashboard.initrunupdate();
    }
}

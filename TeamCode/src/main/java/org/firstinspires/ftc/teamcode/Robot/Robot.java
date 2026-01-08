package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

public class Robot {
    public Shooter shooter;
    public DriveTrain driveTrain;
//    public boolean tele = false;
    public BetterDashboard dashboard;
    public Robot(OpModeType type,HardwareMap hardwareMap, Pose pose){
        if (type == OpModeType.TELEOP) {
            initTele();
//            tele = true;
        } else {
            initAuto();
//            tele = false;
        }
        dashboard = new BetterDashboard();
        driveTrain = new DriveTrain(hardwareMap,pose);
        shooter = new Shooter(hardwareMap);
    }
    public void initTele() {
        driveTrain.initloop();
//        shooter.shooterTelem();
        dashboard.initloopupdate();
    }

    public void initAuto() {
        driveTrain.initloop();
//        shooter.shooterTelem();
        dashboard.initloopupdate();
    }

    public void updateRobotRun(){
        driveTrain.driveTrainTelem();
        shooter.shooterTelem();
        dashboard.initrunupdate();
    }

    public enum OpModeType {
        TELEOP, AUTO
    }
}

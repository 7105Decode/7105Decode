package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;

public class Robot {
    public Shooter shooter;
    public DriveTrain driveTrain;
    public BetterDashboard dashboard;
    public Transfer transfer;
    Telemetry telemetry;
    public Robot(OpModeType type,HardwareMap hardwareMap, Pose pose,Telemetry telemetry){
        dashboard = new BetterDashboard();
        driveTrain = new DriveTrain(hardwareMap,pose);
        shooter = new Shooter(hardwareMap);
        transfer = new Transfer(hardwareMap);
        this.telemetry = telemetry;
        if (type == OpModeType.TELEOP) {
            initLoopTele();
        } else {
            initLoopAuto();
        }
    }
    public void initLoopTele() {
        driveTrain.initloop();
        shooter.shooterTelem();
        dashboard.dashboardTelem();
        BetterDashboard.telemetryM.update(telemetry);
    }

    public void initLoopAuto() {
        driveTrain.initloop();
        shooter.shooterTelem();
        dashboard.dashboardTelem();
        BetterDashboard.telemetryM.update(telemetry);
    }

    public void updateRobotRun(){
        driveTrain.driveTrainTelem();
        shooter.shooterTelem();
        dashboard.dashboardTelem();
        BetterDashboard.telemetryM.update(telemetry);
    }

    public enum OpModeType {
        TELEOP, AUTO
    }
}

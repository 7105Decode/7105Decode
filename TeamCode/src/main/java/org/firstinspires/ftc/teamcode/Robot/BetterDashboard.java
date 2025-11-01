package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BetterDashboard extends SubsystemBase {
    Telemetry telemetry;
    public double hello = 0;
    public static TelemetryPacket telemetryPacket;
    public BetterDashboard(Telemetry telemetry){
        this.telemetry = telemetry;
        hello = 0;
    }
    public void sendTelem(String name, Object packet){
        telemetry.addData(name,packet);
        telemetryPacket.put(name,packet);
    }
    public void dashboardTelem(){
        sendTelem("double",hello);
    }
    public void updateTelemetry(){
        telemetryPacket = new TelemetryPacket();
        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
        telemetry.update();
    }
    public void streamLimelight(Limelight3A limelight3A, int maxfps){
        FtcDashboard.getInstance().startCameraStream(limelight3A,maxfps);
    }
    public void stopLimelightStream(){
        FtcDashboard.getInstance().stopCameraStream();
    }
}

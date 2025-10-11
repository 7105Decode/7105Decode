package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BetterDashboard extends SubsystemBase {
    Telemetry telemetry;
    public TelemetryPacket telemetryPacket;
    public BetterDashboard(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        telemetryPacket = new TelemetryPacket();
        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
        telemetry.update();
    }

    public void sendTelem(String name, Object packet){
        telemetry.addData(name,packet);
        telemetryPacket.put(name,packet);
    }
    public void streamLimelight(Limelight3A limelight3A, int maxfps){
        FtcDashboard.getInstance().startCameraStream(limelight3A,maxfps);
    }
    public void stopLimelightStream(){
        FtcDashboard.getInstance().stopCameraStream();
    }
}

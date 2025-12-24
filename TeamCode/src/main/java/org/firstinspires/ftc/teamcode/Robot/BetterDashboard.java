//package org.firstinspires.ftc.teamcode.Robot;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//public class BetterDashboard extends SubsystemBase {
//    static Telemetry telemetry;
//    public static TelemetryPacket telemetryPacket;
//    public double loopTime;
//    public BetterDashboard(Telemetry telemetry){
//        BetterDashboard.telemetry = telemetry;
//    }
//    public static void sendTelem(String name, Object packet){
//        telemetry.addData(name,packet);
//        telemetryPacket.put(name,packet);
//    }
//    public void dashboardTelem(){
//        double loop = System.nanoTime();
//        sendTelem("Loop Time ", 1000000000 / (loop - loopTime));
//        loopTime = loop;
//    }
//    public void updateTelemetry(){
//        telemetryPacket = new TelemetryPacket();
//        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
//        telemetry.update();
//    }
//    public void streamLimelight(Limelight3A limelight3A, int maxfps){
//        FtcDashboard.getInstance().startCameraStream(limelight3A,maxfps);
//    }
//    public void stopLimelightStream(){
//        FtcDashboard.getInstance().stopCameraStream();
//    }
//}

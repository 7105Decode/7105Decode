package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Tuning.follower;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class BetterDashboard extends SubsystemBase {
    @IgnoreConfigurable
    static PoseHistory poseHistory;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @IgnoreConfigurable
    static ArrayList<String> changes = new ArrayList<>();
//    public static TelemetryPacket telemetryPacket;
    public double loopTime;
    public BetterDashboard(Follower follower){
        poseHistory = follower.getPoseHistory();
    }
    public static void sendTelem(String name, Object packet){
    }
    public void dashboardTelem(){
        double loop = System.nanoTime();
        sendTelem("Loop Time ", 1000000000 / (loop - loopTime));
        loopTime = loop;
    }
    public void updateTelemetry(){

    }
}

package org.firstinspires.ftc.teamcode.Robot;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.DrawingCopy;
import org.firstinspires.ftc.teamcode.Tuning;


public class BetterDashboard extends SubsystemBase {
    @IgnoreConfigurable
    static TelemetryManager telemetryM;
    public double loopTime;
    public BetterDashboard(){
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        DrawingCopy.init();
    }
    public void initloopupdate(){
        Tuning.drawOnlyCurrent();
        dashboardTelem();
        telemetryM.update();
    }
    public void initrunupdate(){
        Tuning.draw();
        dashboardTelem();
        telemetryM.update();
    }

    public static void addtelem(String name, Object packet){
        telemetryM.addData(name,packet);
    }
    public void dashboardTelem(){
        double loop = System.nanoTime();
        addtelem("Loop Time ", 1000000000 / (loop - loopTime));
        loopTime = loop;
    }
}

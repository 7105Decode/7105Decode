package org.firstinspires.ftc.teamcode.Robot;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.rowanmcalpin.nextftc.core.Subsystem;

public class BetterDashboard extends Subsystem {
    public static final BetterDashboard INSTANCE = new BetterDashboard();
    private BetterDashboard() { }
    @IgnoreConfigurable
    public static TelemetryManager telemetryM;
    public double loopTime;
//    public BetterDashboard(){
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//        DrawingCopy.init();
//    }
//    public void initloopupdate(){
//        dashboardTelem();
//        telemetryM.update();
//    }
//    public void initrunupdate(){
//        dashboardTelem();
//        telemetryM.update();
//    }

//    @Override
//    public void periodic() {
//        dashboardTelem();
//    }

    public static void addtelem(String name, Object packet){
        telemetryM.addData(name,packet);
    }
    public void dashboardTelem(){
        double loop = System.nanoTime();
        addtelem("Loop Time ", 1000000000 / (loop - loopTime));
        loopTime = loop;
    }
}

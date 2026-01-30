package org.firstinspires.ftc.teamcode.Robot;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.rowanmcalpin.nextftc.core.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class MoreConvenientTelemetry extends Subsystem {
    public static final MoreConvenientTelemetry INSTANCE = new MoreConvenientTelemetry();
    private MoreConvenientTelemetry() { }
    @IgnoreConfigurable
    public static TelemetryManager telemetryM;
    public static Telemetry telemetry;
    public double loopTime;

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        dashboardTelem();
    }

    @Override
    public void periodic() {
        dashboardTelem();
    }
    public void setTelemetry(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public static void addtelem(String name, Object packet){
        telemetryM.addData(name,packet);
        telemetry.addData(name,packet);
    }
    public void dashboardTelem(){
        double loop = System.nanoTime();
        addtelem("Loop Time ", 1000000000 / (loop - loopTime));
        loopTime = loop;
    }
}

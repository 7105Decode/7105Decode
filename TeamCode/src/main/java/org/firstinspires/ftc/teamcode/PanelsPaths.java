package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class PanelsPaths {
    public static double x,y,heading;
    Pose pose;
    public PanelsPaths(double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
}

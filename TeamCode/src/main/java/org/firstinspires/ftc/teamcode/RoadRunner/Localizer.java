package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;

/**
 * Interface for localization methods.
 */
public interface Localizer {
    /**
     * Updates the Localizer's pose estimate.
     *
     * @return the Localizer's current velocity estimate
     */
    Twist2dDual<Time> update();
}

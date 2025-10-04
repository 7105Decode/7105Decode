package org.firstinspires.ftc.teamcode;

public class Apriltag {

    double aprilTagHeading = 0,aprilTagX = 0,aprilTagY = 0;

    public double getX(){
        return aprilTagX;
    }
    public double getY(){
        return aprilTagY;
    }
    public double getHeading(){
        return aprilTagHeading;
    }    public void setX(double x){
        aprilTagX = x;
    }
    public void setY(double y){
        aprilTagY = y;
    }
    public void setHeading(double heading){
        aprilTagHeading = heading;
    }
}

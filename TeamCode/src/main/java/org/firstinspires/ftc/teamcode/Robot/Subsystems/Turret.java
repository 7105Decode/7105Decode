//package org.firstinspires.ftc.teamcode.Robot.Subsystems;
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.teamcode.Robot.BetterDashboard;
//public class Turret extends SubsystemBase{
//    public static double target = 0;
//    DcMotor turret;
//    public Turret(HardwareMap hardwareMap){
//        turret = hardwareMap.get(DcMotor.class,"turret");
//    }
//    public void resetTurret(){
//        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }
//    public void turretTelemetry() {
////        BetterDashboard.sendTelem("turretpower",getPower());
////        BetterDashboard.sendTelem("turretposition",getPosition());
//    }
//    public double getPosition(){
//        return turret.getCurrentPosition();
//    }
//    public double getPower(){
//        return turret.getPower();
//    }
//    public double getError(){
//        return target-getPosition();
//    }
//}

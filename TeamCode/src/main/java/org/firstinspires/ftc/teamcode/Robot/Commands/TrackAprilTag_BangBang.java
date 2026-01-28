package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.rowanmcalpin.nextftc.core.command.Command;
//
//public class TrackAprilTag_BangBang extends Command {
//    Limelight3A limelight;
//    public TrackAprilTag_BangBang(Limelight3A limelight){
//        this.result = result;
//        this.limelight = limelight;
//    }
//    @Override
//    public void start() {
//        result = limelight.getLatestResult();
//    }
//    @Override
//    public void update() {
//        result = limelight.getLatestResult();
////        if (result.isValid()) {
//        if (ty <= -8.5 && result.isValid()) {
//            topturret.setPower(-.3);
//        } else if (ty > -8.5 && ty < .3) {
//            topturret.setPower(-.09);
//        } else if (ty >= 9.5) {
//            topturret.setPower(.3);
//        } else if (ty > .7) {
//            topturret.setPower(.09);
//        } else {
//            topturret.setPower(0);
//        }
////    }
//    @Override
//    public boolean isDone() {
//        return false;
//    }
//}

package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

@Configurable
public class Transfer extends Subsystem {
    public static final Transfer INSTANCE = new Transfer();
    private Transfer() { }
    RevColorSensorV3 leftsensor, rightsensor,midsensor;
    public static double leftdownpos = 0.07, rightreadytransferpos,midreadytransferpos = .7,leftreadytransferpos,lefttransferpos = .7, midfurtherback = 0.083,middownpos = 0.095,midtransferpos = .7, rightdownpos = .11, righttransferpos = .7;
    public Servo righttransfer,lefttransfer,midtransfer,rightled,midled,leftled;
    public DetectLeftArt detectLeftArt = DetectLeftArt.OPTICAL;
    public DetectMidArt detectMidArt = DetectMidArt.OPTICAL;
    public DetectRightArt detectRightArt = DetectRightArt.OPTICAL;
    ElapsedTime midtimer = new ElapsedTime(), lefttimer = new ElapsedTime(), righttimer = new ElapsedTime();
    @Override
    public void initialize() {
        righttransfer = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "righttransfer");
        lefttransfer = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "lefttransfer");
        midtransfer = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "midtransfer");
        rightsensor = OpModeData.INSTANCE.getHardwareMap().get(RevColorSensorV3.class,"rightcolorsensor");
        leftsensor = OpModeData.INSTANCE.getHardwareMap().get(RevColorSensorV3.class,"leftcolorsensor");
        midsensor = OpModeData.INSTANCE.getHardwareMap().get(RevColorSensorV3.class,"middlecolorsensor");
        rightled = OpModeData.INSTANCE.getHardwareMap().get(Servo.class,"rightled");
        midled = OpModeData.INSTANCE.getHardwareMap().get(Servo.class,"midled");
        leftled = OpModeData.INSTANCE.getHardwareMap().get(Servo.class,"leftled");
        righttransfer.setDirection(Servo.Direction.REVERSE);
        midtransfer.setPosition(midfurtherback);
        detectLeftArt = DetectLeftArt.OPTICAL;
        detectMidArt = DetectMidArt.OPTICAL;
        detectRightArt = DetectRightArt.OPTICAL;
    }
    public void detectRightArtifact(){
        switch (detectRightArt){
            case OPTICAL:
                if(rightsensor.rawOptical() >= 300){
                    detectRightArt = DetectRightArt.GET_COLOR;
                } else if (rightsensor.rawOptical() > 180) {
                   detectRightArt = DetectRightArt.GET_COLOR_MID;
                }else if (rightsensor.rawOptical() > 130) {
                    detectRightArt = DetectRightArt.GET_COLOR_LOW;
                }
                break;
            case GET_COLOR:
                if (rightsensor.red() >= 80){
                    detectRightArt = DetectRightArt.SET_LED_P;
                } else if (rightsensor.red() >= 60) {
                    detectRightArt = DetectRightArt.SET_LED_G;
                }
                break;
            case GET_COLOR_MID:
                if (rightsensor.red() >= 47){
                    detectRightArt = DetectRightArt.SET_LED_P;
                } else if (rightsensor.red() >= 26) {
                    detectRightArt = DetectRightArt.SET_LED_G;
                }
                break;
            case GET_COLOR_LOW:
                if (rightsensor.red() >= 37){
                    detectRightArt = DetectRightArt.SET_LED_P;
                } else if (rightsensor.red() >= 26) {
                    detectRightArt = DetectRightArt.SET_LED_G;
                }
                break;
            case SET_LED_G:
                rightled.setPosition(.5);
                if (lefttransfer.getPosition() == .7){
                    detectRightArt = DetectRightArt.RESETTING;
                }
                break;
            case SET_LED_P:
                rightled.setPosition(.722);
                if (righttransfer.getPosition() == .7){
                    detectRightArt = DetectRightArt.RESETTING;
                }
                break;
            case RESETTING:
                rightled.setPosition(0);
                if (righttransfer.getPosition() == rightdownpos){
                    righttimer.reset();
                    detectRightArt = DetectRightArt.ENCORE;
                }
                break;
            case ENCORE:
                if(righttimer.seconds()> .75){
                    detectRightArt = DetectRightArt.OPTICAL;
                }
                break;
        }
    }
    public void detectLeftArtifact(){
        switch (detectLeftArt){
            case OPTICAL:
                if(leftsensor.rawOptical() >= 300){
                    detectLeftArt = DetectLeftArt.GET_COLOR;
                } else if (leftsensor.rawOptical() > 180) {
                    detectLeftArt = DetectLeftArt.GET_COLOR_MID;
                }else if (leftsensor.rawOptical() > 130) {
                    detectLeftArt = DetectLeftArt.GET_COLOR_LOW;
                }
                break;
            case GET_COLOR:
                if (leftsensor.red() >= 80){
                    detectLeftArt = DetectLeftArt.SET_LED_P;
                } else if (leftsensor.red() >= 60) {
                    detectLeftArt = DetectLeftArt.SET_LED_G;
                }
                break;
            case GET_COLOR_MID:
                if (leftsensor.red() >= 47){
                    detectLeftArt = DetectLeftArt.SET_LED_P;
                } else if (leftsensor.red() >= 26) {
                    detectLeftArt = DetectLeftArt.SET_LED_G;
                }
                break;
            case GET_COLOR_LOW:
                if (leftsensor.red() >= 37){
                    detectLeftArt = DetectLeftArt.SET_LED_P;
                } else if (leftsensor.red() >= 26) {
                    detectLeftArt = DetectLeftArt.SET_LED_G;
                }
                break;
            case SET_LED_G:
                leftled.setPosition(.5);
                if (lefttransfer.getPosition() == .7){
                    detectLeftArt = DetectLeftArt.RESETTING;
                }
                break;
            case SET_LED_P:
                leftled.setPosition(.722);
                if (lefttransfer.getPosition() == .7){
                    detectLeftArt = DetectLeftArt.RESETTING;
                }
                break;
            case RESETTING:
                leftled.setPosition(0);
                if (lefttransfer.getPosition() == leftdownpos){
                    lefttimer.reset();
                    detectLeftArt = DetectLeftArt.ENCORE;
                }
                break;
            case ENCORE:
                if(lefttimer.seconds()> .75){
                    detectLeftArt = DetectLeftArt.OPTICAL;
                }
                break;
        }
    }
    public void detectMidArtifact(){
        switch (detectMidArt){
            case OPTICAL:
                if(midsensor.rawOptical() >= 135){
                    detectMidArt = DetectMidArt.GET_COLOR;
                } else if (leftsensor.rawOptical() > 112) {
                    detectMidArt = DetectMidArt.GET_COLOR_MID;
                }else if (leftsensor.rawOptical() > 90) {
                    detectMidArt = DetectMidArt.GET_COLOR_LOW;
                }
                break;
            case GET_COLOR:
                if (midsensor.green() >= 90){
                    detectMidArt = DetectMidArt.SET_LED_P;
                } else if (midsensor.green() >= 60){
                    detectMidArt = DetectMidArt.SET_LED_G;
                }
                break;
            case GET_COLOR_MID:
                if (midsensor.green() >= 80){
                    detectMidArt = DetectMidArt.SET_LED_P;
                } else if (midsensor.green() >=50)  {
                    detectMidArt = DetectMidArt.SET_LED_G;
                }
                break;
            case GET_COLOR_LOW:
                if (midsensor.green() >= 75) {
                    detectMidArt = DetectMidArt.SET_LED_P;
                } else if (midsensor.green() >=50) {
                    detectMidArt = DetectMidArt.SET_LED_G;
                }
                break;
            case SET_LED_G:
                midled.setPosition(.5);
                if (midtransfer.getPosition() == .7){
                    detectMidArt = DetectMidArt.RESETTING;
                }
                break;
            case SET_LED_P:
                midled.setPosition(.722);
                if (midtransfer.getPosition() == .7){
                    detectMidArt = DetectMidArt.RESETTING;
                }
                break;
            case RESETTING:
                midled.setPosition(0);
                if (midtransfer.getPosition() == middownpos || midtransfer.getPosition() == midfurtherback){
                    midtimer.reset();
                    detectMidArt = DetectMidArt.ENCORE;
                }
                break;
            case ENCORE:
                if(midtimer.seconds()> .75){
                    detectMidArt = DetectMidArt.OPTICAL;
                }
                break;
        }
    }
    public Command transfer_RightArtifact() {
        return new ServoToPosition(righttransfer, // SERVO TO MOVE
                righttransferpos, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command transfer_LeftArtifact() {
        return new ServoToPosition(lefttransfer, // SERVO TO MOVE
                lefttransferpos, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command transfer_MidArtifact() {
        return new ServoToPosition(midtransfer, // SERVO TO MOVE
                midtransferpos, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command down_RightArtifact() {
        return new ServoToPosition(righttransfer, // SERVO TO MOVE
                rightdownpos, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command down_LeftArtifact() {
        return new ServoToPosition(lefttransfer, // SERVO TO MOVE
                leftdownpos, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command down_MidArtifact() {
        return new ServoToPosition(midtransfer, // SERVO TO MOVE
                middownpos, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command readyForTransfer_RightArtifact() {
        return new ServoToPosition(righttransfer, // SERVO TO MOVE
                rightreadytransferpos, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command readyForTransfer_LeftArtifact() {
        return new ServoToPosition(lefttransfer, // SERVO TO MOVE
                leftreadytransferpos, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command readyForTransfer_MidArtifact() {
        return new ServoToPosition(midtransfer, // SERVO TO MOVE
                midreadytransferpos, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    public enum DetectMidArt{
        OPTICAL,
        GET_COLOR,
        GET_COLOR_MID,
        GET_COLOR_LOW,
        SET_LED_G,
        SET_LED_P,
        RESETTING,
        ENCORE
    }
    public enum DetectLeftArt{
        OPTICAL,
        GET_COLOR,
        GET_COLOR_MID,
        GET_COLOR_LOW,
        SET_LED_G,
        SET_LED_P,
        RESETTING,
        ENCORE
    }
    public enum DetectRightArt{
        OPTICAL,
        GET_COLOR,
        GET_COLOR_MID,
        GET_COLOR_LOW,
        SET_LED_G,
        SET_LED_P,
        ENCORE,
        RESETTING
    }
}

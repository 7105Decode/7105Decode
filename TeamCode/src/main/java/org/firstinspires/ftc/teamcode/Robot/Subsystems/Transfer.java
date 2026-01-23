package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

@Configurable
public class Transfer extends Subsystem {
    public static final Transfer INSTANCE = new Transfer();
    private Transfer() { }
    public static double  lefttransferservopos = 0.04, midtransferservopos = .13,righttransferservopos = 0.095;
    Servo righttransfer,lefttransfer,midtransfer;
    public String righttransfername = "righttransfer", lefttransfername = "lefttransfer",midtransfername = "midtransfer";
    @Override
    public void initialize() {
        righttransfer = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, righttransfername);
        lefttransfer = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, lefttransfername);
        midtransfer = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, midtransfername);
    }
    public Command transfer_RightArtifact() {
        return new ServoToPosition(righttransfer, // SERVO TO MOVE
                0.9, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command transfer_LeftArtifact() {
        return new ServoToPosition(lefttransfer, // SERVO TO MOVE
                0.7, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command transfer_MidArtifact() {
        return new ServoToPosition(midtransfer, // SERVO TO MOVE
                0.9, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command down_RightArtifact() {
        return new ServoToPosition(righttransfer, // SERVO TO MOVE
                0.9, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command down_LeftArtifact() {
        return new ServoToPosition(lefttransfer, // SERVO TO MOVE
                lefttransferservopos, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command down_MidArtifact() {
        return new ServoToPosition(midtransfer, // SERVO TO MOVE
                0.9, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command readyForTransfer_RightArtifact() {
        return new ServoToPosition(righttransfer, // SERVO TO MOVE
                0.9, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command readyForTransfer_LeftArtifact() {
        return new ServoToPosition(lefttransfer, // SERVO TO MOVE
                0.9, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command readyForTransfer_MidArtifact() {
        return new ServoToPosition(midtransfer, // SERVO TO MOVE
                0.9, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    public void setRightTransfer(RightTransferStates rightTransferStates){
        switch (rightTransferStates){
            case TRANSFER:
                righttransfer.setPosition(.6);
                break;
            case DOWN:
                righttransfer.setPosition(righttransferservopos);
                break;
        }
    }
    public void setLeftTransfer(LeftTransferStates leftTransferStates){
        switch (leftTransferStates){
            case TRANSFER:
                lefttransfer.setPosition(.625);
                break;
            case DOWN:
                lefttransfer.setPosition(lefttransferservopos);
                break;
        }
    }
    public void setMidTransfer(MidTransferStates midTransferStates){
        switch (midTransferStates){
            case TRANSFER:
                midtransfer.setPosition(.6);
                break;
            case DOWN:
                midtransfer.setPosition(midtransferservopos);
                break;
        }
    }
    public enum RightTransferStates{
        TRANSFER,
        DOWN
    }
    public enum LeftTransferStates{
        TRANSFER,
        DOWN
    }
    public enum MidTransferStates{
        TRANSFER,
        DOWN
    }
}

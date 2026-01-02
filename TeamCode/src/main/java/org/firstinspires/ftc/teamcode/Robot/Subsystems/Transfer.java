package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Configurable
public class Transfer extends SubsystemBase {

    public static double lefttransferservopos = .155, midtransferservopos = .13,righttransferservopos = .13;
    Servo righttransfer,lefttransfer,midtransfer;
    public Transfer(HardwareMap hardwareMap){
        righttransfer = hardwareMap.get(Servo.class,"righttransfer");
        midtransfer = hardwareMap.get(Servo.class,"midtransfer");
        lefttransfer = hardwareMap.get(Servo.class,"lefttransfer");
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

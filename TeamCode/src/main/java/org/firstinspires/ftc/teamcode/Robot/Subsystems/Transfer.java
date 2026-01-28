package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

@Configurable
public class Transfer extends Subsystem {
    public static final Transfer INSTANCE = new Transfer();
    private Transfer() { }
    RevColorSensorV3 leftsensor, rightsensor,midsensor;
    public static double leftdownpos = 0.065, rightreadytransferpos,midreadytransferpos,leftreadytransferpos,lefttransferpos = 1, middownpos = 0.09,midtransferpos = .7, rightdownpos = .11, righttransferpos = .7;
    Servo righttransfer,lefttransfer,midtransfer;
    @Override
    public void initialize() {
        righttransfer = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "righttransfer");
        lefttransfer = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "lefttransfer");
        midtransfer = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "midtransfer");
        rightsensor = OpModeData.INSTANCE.getHardwareMap().get(RevColorSensorV3.class,"rightcolorsensor");
        leftsensor = OpModeData.INSTANCE.getHardwareMap().get(RevColorSensorV3.class,"leftcolorsensor");
        midsensor = OpModeData.INSTANCE.getHardwareMap().get(RevColorSensorV3.class,"midcolorsensor");
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
}

package org.firstinspires.ftc.teamcode.Robot.Commands;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.leftreadytransferpos;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.midreadytransferpos;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.righttransferpos;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;

public class MoveTransferCheckforPark extends Command {
    ElapsedTime timer = new ElapsedTime();
    boolean rightservo, leftservo, midservo;
    double targetpos, time,parktimer,parktimethreshold;;
    Transfer transfer;
    public MoveTransferCheckforPark(Transfer transfer,boolean rightservo, boolean leftservo, boolean midservo, double targetpos, double time, double parktimer,double parktimethreshold){
        this.leftservo = leftservo;
        this.transfer= transfer;
        this.midservo = midservo;
        this.rightservo = rightservo;
        this.targetpos = targetpos;
        this.time = time;
        this.parktimer = parktimer;
        this.parktimethreshold = parktimethreshold;
    }

    @Override
    public void start() {
        timer.reset();
            if (rightservo) {
                transfer.righttransfer.setPosition(targetpos);
            } else if (leftservo) {
                transfer.lefttransfer.setPosition(targetpos);
            } else if (midservo) {
                transfer.midtransfer.setPosition(targetpos);
            }
    }

    @Override
    public boolean isDone() {
        return timer.seconds() > time || parktimer>= parktimethreshold;
    }
}

package org.firstinspires.ftc.teamcode.Robot.Commands;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.leftreadytransferpos;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.midreadytransferpos;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.righttransferpos;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;

public class MoveTransfer extends Command {
    ElapsedTime timer = new ElapsedTime();
    boolean rightservo, leftservo, midservo;
    double targetpos;
    public MoveTransfer(boolean rightservo, boolean leftservo, boolean midservo, double targetpos){
        this.leftservo = leftservo;
        this.midservo = midservo;
        this.rightservo = rightservo;
        this.targetpos = targetpos;
    }

    @Override
    public void start() {
        timer.reset();
        if (rightservo){
            Transfer.INSTANCE.righttransfer.setPosition(targetpos);
        } else if (leftservo) {
            Transfer.INSTANCE.lefttransfer.setPosition(targetpos);
        } else if (midservo) {
            Transfer.INSTANCE.midtransfer.setPosition(targetpos);
        }
    }
    @Override
    public boolean isDone() {
        return timer.seconds() > .3;
    }

    @Override
    public void stop(boolean interrupted) {
        timer.reset();
    }
}

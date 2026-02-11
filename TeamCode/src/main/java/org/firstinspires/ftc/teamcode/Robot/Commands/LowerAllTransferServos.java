package org.firstinspires.ftc.teamcode.Robot.Commands;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.leftdownpos;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.middownpos;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.rightdownpos;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;
public class LowerAllTransferServos extends Command {
    ElapsedTime timer = new ElapsedTime();
    Transfer transfer;
    public LowerAllTransferServos(Transfer transfer){
        this.transfer = transfer;
    }

    @Override
    public void start() {
        transfer.midtransfer.setPosition(middownpos);
        transfer.lefttransfer.setPosition(leftdownpos);
        transfer.righttransfer.setPosition(rightdownpos);
    }

    @Override
    public void update() {
        transfer.midtransfer.setPosition(middownpos);
        transfer.lefttransfer.setPosition(leftdownpos);
        transfer.righttransfer.setPosition(rightdownpos);
    }

    @Override
    public boolean isDone() {
        return timer.seconds() > .6;
    }
}

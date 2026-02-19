package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;

public class ArtifactShaker extends Command {
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void update() {
        if (timer.seconds() <= .25){
            Transfer.INSTANCE.righttransfer.setPosition(Transfer.rightdownpos + .1);
            Transfer.INSTANCE.lefttransfer.setPosition(Transfer.leftdownpos + .1);
        } else if (timer.seconds() <= .5) {
            Transfer.INSTANCE.righttransfer.setPosition(Transfer.rightdownpos );
            Transfer.INSTANCE.lefttransfer.setPosition(Transfer.leftdownpos );
        }else if (timer.seconds() <= .75) {
            Transfer.INSTANCE.righttransfer.setPosition(Transfer.rightdownpos + .1);
            Transfer.INSTANCE.lefttransfer.setPosition(Transfer.leftdownpos + .1);
        } else {
            Transfer.INSTANCE.righttransfer.setPosition(Transfer.rightdownpos );
            Transfer.INSTANCE.lefttransfer.setPosition(Transfer.leftdownpos );
        }
    }

    @Override
    public boolean isDone() {
        return timer.seconds() >= 1;
    }
}

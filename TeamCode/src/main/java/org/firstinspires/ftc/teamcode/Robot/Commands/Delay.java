package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.rowanmcalpin.nextftc.core.command.Command;

public class Delay extends Command {
    ElapsedTime timer = new ElapsedTime();
    double time;
    public Delay(double time){
        this.time = time;
    }
    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public boolean isDone() {
        return timer.seconds() > time;
    }
}

package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.rowanmcalpin.nextftc.core.command.Command;

public class DoneChanger extends Command {

    public DoneChanger(boolean done){
     if (done){

     } else {

     }
    }

    @Override
    public boolean isDone() {
        return true;
    }
}

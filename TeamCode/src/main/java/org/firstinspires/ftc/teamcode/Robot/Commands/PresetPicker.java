package org.firstinspires.ftc.teamcode.Robot.Commands;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.middownpos;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.midreadytransferpos;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.righttransferpos;

import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

public class PresetPicker extends Command {

    boolean done = false;

    @Override
    public void start() {
        if (Turret.PPG){
            new SequentialGroup(new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.lefttransferpos,.9),
                    new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.leftdownpos,.45),
                    new MoveTransfer(Transfer.INSTANCE,true,false,false,righttransferpos,.9),
                    new MoveTransfer(Transfer.INSTANCE,true,false,false,Transfer.rightdownpos,.45),
                    new MoveTransfer(Transfer.INSTANCE,false,false,true, midreadytransferpos,.9),
                    new MoveTransfer(Transfer.INSTANCE,false,false,true, middownpos,.45)
                    );
        }else if (Turret.PGP){
            new SequentialGroup(
                    new MoveTransfer(Transfer.INSTANCE,true,false,false,righttransferpos,.9),
                    new MoveTransfer(Transfer.INSTANCE,true,false,false,Transfer.rightdownpos,.45),
                    new MoveTransfer(Transfer.INSTANCE,false,false,true, midreadytransferpos,.9),
                    new MoveTransfer(Transfer.INSTANCE,false,false,true, middownpos,.45),
                    new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.lefttransferpos,.9),
                    new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.leftdownpos,.45)
            );
        } else if (Turret.GPP) {
            new SequentialGroup(
                    new MoveTransfer(Transfer.INSTANCE,false,false,true, midreadytransferpos,.9),
                    new MoveTransfer(Transfer.INSTANCE,false,false,true, middownpos,.45),
                    new MoveTransfer(Transfer.INSTANCE,true,false,false,righttransferpos,.9),
                    new MoveTransfer(Transfer.INSTANCE,true,false,false,Transfer.rightdownpos,.45),
                    new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.lefttransferpos,.9),
                    new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.leftdownpos,.45),
                    new DoneChanger(true)
            );
        }
    }

    @Override
    public boolean isDone() {
        return done;
    }
}

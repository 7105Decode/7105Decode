package org.firstinspires.ftc.teamcode.Robot.Commands;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.middownpos;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.midreadytransferpos;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer.righttransferpos;

import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

public class AutoArtifactPicker extends Command {
    Command command;
    boolean firstOne, midOne,lastOne, done;
    public AutoArtifactPicker(Command command){
        this.command = command;
    }
    @Override
    public void start() {
        lastOne = false;
        firstOne = false;
        midOne = false;
        done = false;
    }

    @Override
    public void update() {
        if (Turret.PPG){
            if (Transfer.INSTANCE.detectRightArt == Transfer.DetectRightArt.SET_LED_P || lastOne &&
            Transfer.INSTANCE.detectRightArt != Transfer.DetectRightArt.RESETTING){
                new SequentialGroup( new MoveTransfer(Transfer.INSTANCE,true,false,false,righttransferpos,.95),
                        new MoveTransfer(Transfer.INSTANCE,true,false,false,Transfer.rightdownpos,.45));
                Transfer.INSTANCE.detectRightArt = Transfer.DetectRightArt.RESETTING;
            } else if (Transfer.INSTANCE.detectLeftArt == Transfer.DetectLeftArt.SET_LED_P || lastOne &&
                    Transfer.INSTANCE.detectLeftArt != Transfer.DetectLeftArt.RESETTING) {
                new SequentialGroup(  new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.lefttransferpos,.95),
                        new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.leftdownpos,.45));
                Transfer.INSTANCE.detectLeftArt = Transfer.DetectLeftArt.RESETTING;
            } else if (Transfer.INSTANCE.detectMidArt == Transfer.DetectMidArt.SET_LED_P || lastOne &&
                    Transfer.INSTANCE.detectMidArt != Transfer.DetectMidArt.RESETTING){
                new SequentialGroup(new MoveTransfer(Transfer.INSTANCE,false,false,true, midreadytransferpos,.95),
                        new MoveTransfer(Transfer.INSTANCE,false,false,true, middownpos,.45));
                Transfer.INSTANCE.detectMidArt = Transfer.DetectMidArt.RESETTING;
            } else if (lastOne) {
                done = true;
            } else {
                lastOne = true;
            }
        } else if (Turret.PGP ) {
            if ((Transfer.INSTANCE.detectRightArt == Transfer.DetectRightArt.SET_LED_P || midOne) &&
                    Transfer.INSTANCE.detectRightArt != Transfer.DetectRightArt.RESETTING && !firstOne){
                firstOne = true;
                new SequentialGroup( new MoveTransfer(Transfer.INSTANCE,true,false,false,righttransferpos,.95),
                        new MoveTransfer(Transfer.INSTANCE,true,false,false,Transfer.rightdownpos,.45));
                Transfer.INSTANCE.detectRightArt = Transfer.DetectRightArt.RESETTING;
            } else if ((Transfer.INSTANCE.detectLeftArt == Transfer.DetectLeftArt.SET_LED_P || midOne) &&
                    Transfer.INSTANCE.detectLeftArt != Transfer.DetectLeftArt.RESETTING && !firstOne) {
                firstOne = true;
                new SequentialGroup(  new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.lefttransferpos,.95),
                        new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.leftdownpos,.45));
                Transfer.INSTANCE.detectLeftArt = Transfer.DetectLeftArt.RESETTING;
            } else if ((Transfer.INSTANCE.detectMidArt == Transfer.DetectMidArt.SET_LED_P || midOne) &&
                    Transfer.INSTANCE.detectMidArt != Transfer.DetectMidArt.RESETTING && !firstOne){
                firstOne= true;
                new SequentialGroup(new MoveTransfer(Transfer.INSTANCE,false,false,true, midreadytransferpos,.95),
                        new MoveTransfer(Transfer.INSTANCE,false,false,true, middownpos,.45));
                Transfer.INSTANCE.detectMidArt = Transfer.DetectMidArt.RESETTING;
            } else if (!firstOne) {
                firstOne = true;
                midOne = true;
            } else if (midOne) {
                done = true;
            } else {
                midOne = true;
            }
        } else if (Turret.GPP) {
            if (Transfer.INSTANCE.detectRightArt == Transfer.DetectRightArt.SET_LED_G || firstOne &&
                    Transfer.INSTANCE.detectRightArt != Transfer.DetectRightArt.RESETTING){
                new SequentialGroup( new MoveTransfer(Transfer.INSTANCE,true,false,false,righttransferpos,.95),
                        new MoveTransfer(Transfer.INSTANCE,true,false,false,Transfer.rightdownpos,.45));
                Transfer.INSTANCE.detectRightArt = Transfer.DetectRightArt.RESETTING;
            } else if (Transfer.INSTANCE.detectLeftArt == Transfer.DetectLeftArt.SET_LED_G || firstOne &&
                    Transfer.INSTANCE.detectLeftArt != Transfer.DetectLeftArt.RESETTING) {
                new SequentialGroup(  new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.lefttransferpos,.95),
                        new MoveTransfer(Transfer.INSTANCE,false,true,false,Transfer.leftdownpos,.45));
                Transfer.INSTANCE.detectLeftArt = Transfer.DetectLeftArt.RESETTING;
            } else if (Transfer.INSTANCE.detectMidArt == Transfer.DetectMidArt.SET_LED_G || firstOne &&
                    Transfer.INSTANCE.detectMidArt != Transfer.DetectMidArt.RESETTING){
                new SequentialGroup(new MoveTransfer(Transfer.INSTANCE,false,false,true, midreadytransferpos,.95),
                        new MoveTransfer(Transfer.INSTANCE,false,false,true, middownpos,.45));
                Transfer.INSTANCE.detectMidArt = Transfer.DetectMidArt.RESETTING;
            } else if (firstOne) {
                done = true;
            } else {
                firstOne = true;
            }
        }
    }

    @Override
    public boolean isDone() {
        return done;
    }
}

package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;

public class ColorSenseArtifact extends Command {
    Subsystem subsystem;
    public ColorSenseArtifact(Subsystem subsystem){
        this.subsystem = subsystem;
    }
    @Override
    public void start() {

    }

    @Override
    public void update() {
        Transfer.INSTANCE.detectRightArtifact();
        Transfer.INSTANCE.detectLeftArtifact();
        Transfer.INSTANCE.detectMidArtifact();
    }

    @Override
    public boolean isDone() {
        return false;
    }
}

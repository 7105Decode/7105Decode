//package org.firstinspires.ftc.teamcode.Robot.Commands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;
//
//public class RunRightTransferServo extends CommandBase {
//
//    Transfer transfer;
//    Transfer.RightTransferStates rightTransferStates;
//    ElapsedTime timer = new ElapsedTime();
//    public RunRightTransferServo(Transfer transfer, Transfer.RightTransferStates rightTransferStates){
//        this.transfer = transfer;
//        this.rightTransferStates = rightTransferStates;
//    }
//
//    @Override
//    public void initialize() {
//        timer.reset();
//        transfer.setRightTransfer(rightTransferStates);
//    }
//
//    @Override
//    public boolean isFinished() {
//        return timer.seconds() > 1.5;
//    }
//}

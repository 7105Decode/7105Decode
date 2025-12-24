//package org.firstinspires.ftc.teamcode.Robot.Commands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;
//
//public class RunLeftTransferServo extends CommandBase {
//
//    Transfer transfer;
//    Transfer.LeftTransferStates leftTransferStates;
//    ElapsedTime timer = new ElapsedTime();
//    public RunLeftTransferServo(Transfer transfer, Transfer.LeftTransferStates leftTransferStates){
//        this.transfer = transfer;
//        this.leftTransferStates = leftTransferStates;
//    }
//
//    @Override
//    public void initialize() {
//        timer.reset();
//        transfer.setLeftTransfer(leftTransferStates);
//    }
//
//    @Override
//    public boolean isFinished() {
//        return timer.seconds() > 1.5;
//    }
//}

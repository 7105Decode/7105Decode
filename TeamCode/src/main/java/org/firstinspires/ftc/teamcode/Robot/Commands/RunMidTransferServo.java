//package org.firstinspires.ftc.teamcode.Robot.Commands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Robot.Subsystems.Transfer;
//
//public class RunMidTransferServo extends CommandBase {
//
//    Transfer transfer;
//    Transfer.MidTransferStates midTransferStates;
//    ElapsedTime timer = new ElapsedTime();
//    public RunMidTransferServo(Transfer transfer, Transfer.MidTransferStates midTransferStates){
//        this.transfer = transfer;
//        this.midTransferStates = midTransferStates;
//    }
//
//    @Override
//    public void initialize() {
//        timer.reset();
//        transfer.setMidTransfer(midTransferStates);
//    }
//
//    @Override
//    public boolean isFinished() {
//        return timer.seconds() > 1.5;
//    }
//}

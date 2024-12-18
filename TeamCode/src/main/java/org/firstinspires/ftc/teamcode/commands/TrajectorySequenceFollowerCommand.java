package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TrajectorySequenceFollowerCommand extends CommandBase {
    private SampleMecanumDrive drive;
    private TrajectorySequence trajectory;

    public TrajectorySequenceFollowerCommand(SampleMecanumDrive drive, TrajectorySequence trajectory) {
        this.drive = drive;
        this.trajectory = trajectory;

    }

    @Override
    public void execute() {
        drive.followTrajectorySequence(trajectory);
        drive.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.interruptTrajectorySequence();
        }
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }
}
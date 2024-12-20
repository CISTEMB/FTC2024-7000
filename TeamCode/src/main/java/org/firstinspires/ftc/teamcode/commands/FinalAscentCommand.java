package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorV2;
import org.firstinspires.ftc.teamcode.subsystems.Worm;

public class FinalAscentCommand extends CommandBase {
    private Worm worm;
    private ElevatorV2 elevatorV2;
    public FinalAscentCommand(Worm worm, ElevatorV2 elevatorV2) {
        this.worm = worm;
        this.elevatorV2 = elevatorV2;
        addRequirements(worm, elevatorV2);
    }

    @Override
    public void execute() {
        ParallelCommandGroup c = new ParallelCommandGroup();
        c.addCommands(new RunCommand(() -> worm.lower(-1), worm));
        c.addCommands(new RunCommand(() -> elevatorV2.setPower(-1), elevatorV2));
        c.execute();
    }

    @Override
    public void end(boolean interrupted) {
        //need to set counter to zero after end or we can't keep running it
        worm.brake();
        elevatorV2.setBrake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

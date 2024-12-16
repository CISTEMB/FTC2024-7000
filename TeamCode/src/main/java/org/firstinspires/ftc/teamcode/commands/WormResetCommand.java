package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Worm;

public class WormResetCommand extends CommandBase {
    private Worm worm;
    private int counter;
    private int target;

    public WormResetCommand(Worm worm) {
        this.worm = worm;
        addRequirements(worm);
    }

    @Override
    public void execute() {
        worm.lower(-0.5);
    }

    @Override
    public void end(boolean interrupted) {
        //need to set counter to zero after end or we can't keep running it
        worm.brake();
    }

    @Override
    public boolean isFinished() {
        return counter >= target;
    }
}

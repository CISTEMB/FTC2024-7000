package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Wrist;

public class WristDownCommand extends CommandBase {

    private Wrist wrist;

    public WristDownCommand(Wrist wrist) {
        this.wrist = wrist;
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.Active = true;
    }

    @Override
    public void execute() {
        wrist.RemoveDegree();
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}

package team.ironriders.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.ironriders.lib.Utils;
import team.ironriders.subsystems.ArmSubsystem;


public class PivotClimberMoveCommand extends CommandBase {
    double pivotTarget;
    double climberTarget;
    ArmSubsystem arm;

    public PivotClimberMoveCommand(double pivotTarget, double climberTarget, ArmSubsystem arm) {
        this.pivotTarget = pivotTarget;
        this.climberTarget = climberTarget;
        this.arm = arm;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setPivot(pivotTarget);
        arm.setClimber(climberTarget);
    }

    @Override
    public boolean isFinished() {
        return Utils.isWithinTolerance(arm.getPivotPos(), pivotTarget, 0.05) &&
                Utils.isWithinTolerance(arm.getClimberPos(), climberTarget, 2);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopPivot();
        arm.stopClimber();
    }
}

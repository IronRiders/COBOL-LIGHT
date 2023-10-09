package team.ironriders.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.ironriders.lib.Utils;
import team.ironriders.subsystems.ArmSubsystem;


public class PivotClimberMoveCommand extends CommandBase {
    public enum Preset {
        L1,
        L2,
        L3,
        HP,
        R
    }

    double pivotTarget;
    double climberTarget;
    ArmSubsystem arm;

    public PivotClimberMoveCommand(double pivotTarget, double climberTarget, ArmSubsystem arm) {
        this.pivotTarget = pivotTarget;
        this.climberTarget = climberTarget;
        this.arm = arm;

        addRequirements(arm);
    }

    public PivotClimberMoveCommand(Preset preset, ArmSubsystem arm) {
        switch (preset) {
            case L1 -> {
                this.pivotTarget = 0.6;
                this.climberTarget = 30;
            }
            case L2 -> {
                this.pivotTarget = 1.2;
                this.climberTarget = 80;
            }
            case L3 -> {
                this.pivotTarget = 1.5;
                this.climberTarget = 220;
            }
            case HP -> {
                this.pivotTarget = 1.5;
                this.climberTarget = 50;
            }
            case R -> {
                this.pivotTarget = 0;
                this.climberTarget = 0;
            }
        }

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

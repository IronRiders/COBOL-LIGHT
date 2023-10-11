package team.ironriders.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.ironriders.lib.Utils;
import team.ironriders.subsystems.ArmSubsystem;


public class PivotCommand extends CommandBase {
   double target;
   ArmSubsystem arm;

   public PivotCommand(double target, ArmSubsystem arm) {
       this.target = target;
       this.arm = arm;

       addRequirements(arm);
   }

   @Override
   public void initialize() {
       arm.setPivot(target);
   }

   @Override
   public boolean isFinished() {
       return Utils.isWithinTolerance(arm.getPivotPos(), target, 0.05);
   }

   @Override
   public void end(boolean interrupted) {
       arm.stopPivot();
   }
}

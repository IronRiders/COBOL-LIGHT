package team.ironriders.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.ironriders.lib.Utils;
import team.ironriders.subsystems.ArmSubsystem;


public class ClimberCommand extends CommandBase {
   double target;
   ArmSubsystem arm;

   public ClimberCommand(double target, ArmSubsystem arm) {
       this.target = target;
       this.arm = arm;

       addRequirements(arm);
   }

   @Override
   public void initialize() {
       arm.setClimber(target);
   }

   @Override
   public boolean isFinished() {
       return Utils.isWithinTolerance(arm.getClimberPos(), target, 2);
   }

   @Override
   public void end(boolean interrupted) {
       arm.stopClimber();
   }
}

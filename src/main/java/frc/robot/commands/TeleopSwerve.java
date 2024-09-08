package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.CenterOfRotation;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TeleopSwerve extends Command {
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  Supplier<CenterOfRotation> centerSup;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(6);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(6);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(6);

  public TeleopSwerve(
      Swerve s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup,
      Supplier<CenterOfRotation> centerSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.centerSup = centerSup;
    
    
  }

  @Override
  public void execute() {
    /* Get Values, DeadBand */
    double translationVal = translationLimiter
        .calculate(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.STICK_DEAD_BAND));
    double strafeVal = strafeLimiter
        .calculate(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.STICK_DEAD_BAND));
    double rotationVal = rotationLimiter
        .calculate(MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.STICK_DEAD_BAND));

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED),
        rotationVal * Constants.Swerve.MAX_ANGULAR_VELOCITY,
        !robotCentricSup.getAsBoolean(), true,centerSup.get().value);
  }
}

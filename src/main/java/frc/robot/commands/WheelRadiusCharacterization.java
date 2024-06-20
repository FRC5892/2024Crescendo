package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import monologue.Logged;
import frc.robot.Constants;

public class WheelRadiusCharacterization extends Command implements Logged {
  private static final double characterizationSpeed = 0.25;
  private static final double swerveRadius = Constants.Swerve.WHEEL_BASE/2;
  private final DoubleSupplier gyroYawRadsSupplier;
      // () -> RobotState.getInstance().getOdometryPose().getRotation().getRadians();

  
  public enum Direction {
    CLOCKWISE(-1),
    COUNTER_CLOCKWISE(1);

    private final int value;
    Direction (int value) {
      this.value = value;
    }
  }

  private final Swerve swerve;

  private final Direction omegaDirection;
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

  private double lastGyroYawRads = 0.0;
  private double accumGyroYawRads = 0.0;

  private double[] startWheelPositions;

  private double currentEffectiveWheelRadius = 0.0;

  public WheelRadiusCharacterization(Swerve swerve, Direction omegaDirection) {
    
    this.swerve = swerve;
    this.omegaDirection = omegaDirection;
    this.gyroYawRadsSupplier = () -> swerve.getYaw().getRadians();
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    // Reset
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    accumGyroYawRads = 0.0;

    startWheelPositions = swerve.getModuleDistances();

    omegaLimiter.reset(0);
  }

  @Override
  public void execute() {
    // Run Swerve at velocity
    swerve.runWheelRadiusCharacterization(
        omegaLimiter.calculate(omegaDirection.value * characterizationSpeed));

    // Get yaw and wheel positions
    accumGyroYawRads += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    double averageWheelPosition = 0.0;
    double[] wheelPositiions = swerve.getModuleDistances();
    for (int i = 0; i < 4; i++) {
      averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelPositions[i]);
    }
    averageWheelPosition /= 4.0;

    currentEffectiveWheelRadius = (accumGyroYawRads * swerveRadius) / averageWheelPosition;
    this.log("SwervePosition", averageWheelPosition);
    this.log("AccumGyroYawRads", accumGyroYawRads);
    this.log(
        "CurrentWheelRadiusInches",
        Units.metersToInches(currentEffectiveWheelRadius));
  }

  @Override
  public void end(boolean interrupted) {
    if (accumGyroYawRads <= Math.PI * 2.0) {
      System.out.println("Not enough data for characterization");
    } else {
      System.out.println(
          "Effective Wheel Radius: "
              + Units.metersToInches(currentEffectiveWheelRadius)
              + " inches");
      this.log("effectiveWheelRadiusIn", currentEffectiveWheelRadius);
    }
  }
}

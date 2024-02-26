// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    CANSparkMax clawMotor;
    DigitalInput toplimitSwitch = new DigitalInput(5);
    DigitalInput bottomlimitSwitch = new DigitalInput(6);
    
    public Claw() {
        this.clawMotor = new CANSparkMax(Constants.ClawConstants.clawMotor,MotorType.kBrushless);
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
    
    public void closeClaw(){
        if (bottomlimitSwitch.get()){
            clawMotor.set(0);
        }
        else{
            clawMotor.set(-1);
        }
    };

    public void openClaw(){
        if (toplimitSwitch.get()) {
            clawMotor.set(0);
        }
        else{
            clawMotor.set(1);
        }
        
    }

    public void stopClaw(){
        clawMotor.set(0);
    }


    public Command openClawCommand() {
      return runEnd(this::openClaw, this::stopClaw);
    }

    
    public Command closeClawCommand() {
      return runEnd(this::openClaw, this::stopClaw);
    }

}


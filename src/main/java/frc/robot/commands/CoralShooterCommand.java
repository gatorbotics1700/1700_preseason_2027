// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralShooterCommand extends Command {
  // The command needs to have an instance of the CoralShooterSubsystem, 
  // a variable to keep track of our voltage, and one to track the start time of the command
  /* TODO: declare the CoralShooterSubsystem, voltage, and start time variables here */
    private double voltage;
    private long startTime;
    private CoralShooterSubsystem coralShooterSubsystem;


  /** Creates a new CoralShooterCommand. */
  public CoralShooterCommand(CoralShooterSubsystem coralShooterSubsystem, double voltage) {
    /* TODO: assign the values of the parameters to the variables you instantiated */
    addRequirements(coralShooterSubsystem);
    this.coralShooterSubsystem = coralShooterSubsystem;
    this.voltage = voltage;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /* TODO: initialize the start time variable with the current time */
      // HINT: System.currentTimeMillis() gets you the current time
      startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // We want to set the motor voltage of the Coral Shooter Subsystem to the voltage we passed into the command
    // It's also helpful to have print statements to indicate what the shooter is doing
      // if our voltage is positive, we're intaking
      if (voltage > 0) {
        System.out.println("intaking");
      }
      // if our voltage is equal to the L4 shooting voltage, we're shooting l4
      else if (voltage == Constants.l4ShootingVoltage) {
        System.out.println("Shooting l4");
      }
      // if our voltage is equal to the trough shooting voltage, we're shooting trough
      else if (voltage == Constants.troughShootingVoltage) {
        System.out.println("Shooting Trough");
      }

    /* TODO: set motor voltage */
    coralShooterSubsystem.setMotorVoltage(voltage);
    

    /* TODO: add print statements depending on the voltage to indicate the state the shooter is currently in */
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //TODO from Vivien: make this variable a double
    private long elapsedTime = System.currentTimeMillis() - startTime;
    if (elapsedTime > 5000 && voltage > 0) {
      System.out.println("finished intaking because 5 seconds have passed");
      coralShooterSubsystem.setMotorVoltage(0);
      return true; 
    }
    //TODO from Vivien: your command doesn't know what your limit switch is! instead, use the method that you wrote
    if (limitSwitch.get() && voltage > 0) {
      System.out.println("limit switch triggered");
      coralShooterSubsystem.setMotorVoltage(0);
      return true; 
    }
    if (elapsedTime > 1500 && voltage < 0) {
      System.out.println("finished outtaking because 1.5 seconds have passed");
      coralShooterSubsystem.setMotorVoltage(0);
      return true;
    }
      
    return false;
    }


    
      
    // if we're intaking, we want to end either if it's been 5 seconds or the limit switch is triggered
      // the limit switch is triggered if the value it's getting is true
    // if our voltage is zero, we can just end, but to be safe also set speed to zero
    // if we're outtaking, we want to end if it's been 1.5 seconds
    // in each case, we want to print what's causing the command to end, 
      // set the motor voltage to 0, and return true
    // otherwise, we want to return false

    /* calculate how much time has passed and store it in a variable */
      // you have the start time and can get the current time
      // this value is stored in milliseconds

    /* write the logic for when to end the command */
    
    
      // refer to the descriptions above to write this part
    
  
}

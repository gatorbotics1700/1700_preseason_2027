package frc.robot.subsystems.mech;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//import com.revrobotics.ColorMatch;
//import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;


import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

public class ElevatorSubsystem {
    //commands; go to set position, speed
    //subsystems; methods, limit switch, voltage
    //methods; current limits --> x speed, set position --> rotating motor --> voltage, limit switch --> stops x position
  private TalonFX elevatorMotor;
  private static final double kP = 0.0005;
private static final double kI = 0.0;
private static final double kD = 0.0002;

public ElevatorSubsystem() {
 elevatorMotor = new TalonFX(0);
  elevatorMotor = new TalonFX(Constants.ElevatorSubsystemConstants.ELEVATOR_MOTOR_CAN_ID, Constants.CANIVORE_BUS_NAME);
  elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
  elevatorPIDController = new PIDController(kP, kI, kD);
  topLimitSwitch = new DigitalInput(Constants.TOP_LIMIT_SWITCH_POSITION);
   bottomLimitSwitch = new DigitalInput(Constants.BOTTOM_LIMIT_SWITCH_POSITION);
}
}

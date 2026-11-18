// TODO intended to start pitch at loc 0


package frc.robot.subsystems.mech;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.controller.PIDController; // pid (converts position to voltage)
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class WristSubsystem extends SubsystemBase {
  // roll and pitch PID (data type is PIDController)
  // motors!
  // encoders(make them pitch an roll encoders)
  // variables for pitch and roll positions(used to contain real position of the wrist)datatype Rotation2d


  public WristSubsystem() {
    //motor configs(check w/ build with motor inversions)

  }

  @Override
  public void periodic() {
   /* pitchPos = Rotation2d.fromDegrees(pitchEncoder.getPosition().getValueAsADouble)
   do the same for roll */

  }

  public void setPostion(Rotation2d pitchPos, double rollPos) {
    //pitchVoltage(pitchPID.calculate(pitchPos)), rollVoltage, leftVoltage, rightVoltage
    //safetyPrecautions
    //call setVoltage
  }

  public void setZero () {
    // set pitch and roll encoders to 0
  }

  public void setVoltage(double leftVoltage, double rightVoltage) {

  }
  

}
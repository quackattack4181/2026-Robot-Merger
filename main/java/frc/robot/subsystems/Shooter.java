package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase implements AutoCloseable {
  private final CANSparkFlex topMotor;
  private final CANSparkFlex middleMotor;
  private final CANSparkFlex bottomMotor;

  private final RelativeEncoder topEncoder;
  private final SparkPIDController topPid;

  public Shooter() {
    topMotor = new CANSparkFlex(ShooterConstants.TOP_MOTOR_ID, MotorType.kBrushless);
    middleMotor = new CANSparkFlex(ShooterConstants.MIDDLE_MOTOR_ID, MotorType.kBrushless);
    bottomMotor = new CANSparkFlex(ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);

    topMotor.restoreFactoryDefaults();
    middleMotor.restoreFactoryDefaults();
    bottomMotor.restoreFactoryDefaults();

    topMotor.setIdleMode(IdleMode.kCoast);
    middleMotor.setIdleMode(IdleMode.kCoast);
    bottomMotor.setIdleMode(IdleMode.kCoast);

    topMotor.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS);
    middleMotor.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS);
    bottomMotor.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS);

    topMotor.setInverted(ShooterConstants.TOP_INVERTED);
    middleMotor.follow(topMotor, ShooterConstants.MIDDLE_INVERTED);
    bottomMotor.follow(topMotor, ShooterConstants.BOTTOM_INVERTED);

    topEncoder = topMotor.getEncoder();
    topPid = topMotor.getPIDController();
    topPid.setP(ShooterConstants.KP);
    topPid.setI(ShooterConstants.KI);
    topPid.setD(ShooterConstants.KD);
    topPid.setFF(ShooterConstants.KF);
  }

  public void stop() {
    topMotor.stopMotor();
  }

  public void setShooterVoltage(double voltage) {
    topMotor.setVoltage(voltage);
  }

  public void setShooterRpm(double rpm) {
    topPid.setReference(rpm, ControlType.kVelocity);
  }

  public double getShooterRpm() {
    return topEncoder.getVelocity();
  }

  public boolean atSpeed() {
    return Math.abs(getShooterRpm() - ShooterConstants.SHOOTER_RPM)
        <= ShooterConstants.VELOCITY_TOLERANCE_RPM;
  }

  public Command runShooterRpm(double rpm) {
    return runEnd(() -> setShooterRpm(rpm), this::stop);
  }

  public Command runShooterRpm() {
    return runShooterRpm(ShooterConstants.SHOOTER_RPM);
  }

  @Override
  public void close() {
    topMotor.close();
    middleMotor.close();
    bottomMotor.close();
  }
}

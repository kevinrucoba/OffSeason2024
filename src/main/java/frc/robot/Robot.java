// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.ResourceBundle.Control;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private AHRS navx;
  XboxController control1;
  private PowerDistribution pdh;
  //Module Front Right
  public double kPFR, kIFR, kDFR, kIzFR, kFFFR, kMaxOutputFR, kMinOutputFR;
  private RelativeEncoder encoderFR;
  private CANSparkMax motorAngleFR;
  private CANSparkMax motorSpeedFR;
  private SparkPIDController pidControllerFR;
  //Module BackRight
  public double kPBR, kIBR, kDBR, kIzBR, kFFBR, kMaxOutputBR, kMinOutputBR;
  private RelativeEncoder encoderBR;
  private CANSparkMax motorAngleBR;
  private CANSparkMax motorSpeedBR;
  private SparkPIDController pidControllerBR;
  //Module Back Left
  public double kPBL, kIBL, kDBL, kIzBL, kFFBL, kMaxOutputBL, kMinOutputBL;
  private RelativeEncoder encoderBL;
  private CANSparkMax motorAngleBL;
  private CANSparkMax motorSpeedBL;
  private SparkPIDController pidControllerBL;
  //Module Back Right
  public double kPFL, kIFL, kDFL, kIzFL, kFFFL, kMaxOutputFL, kMinOutputFL;
  private RelativeEncoder encoderFL;
  private CANSparkMax motorAngleFL;
  private CANSparkMax motorSpeedFL;
  private SparkPIDController pidControllerFL;

  //Intake
  private CANSparkMax motorIntake;
  private SparkPIDController pidControllerIntake;
  public double kPIntake, kIIntake, kDIntake, kIzIntake, kFFIntake, kMaxOutputIntake, kMinOutputIntake;

  //Shooter
  private CANSparkMax motorShooterR;
  private SparkPIDController pidControllerShooterR;
  public double kPShooter, kIShooter, kDShooter, kIzShooter, kFFShooter, kMaxOutputShooter, kMinOutputShooter;
  private CANSparkMax motorShooteL;
  private SparkPIDController pidControllerShooterL;
  private DigitalInput limitL;
  private DigitalInput limitR;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    UsbCamera cam1 = CameraServer.startAutomaticCapture();
    UsbCamera cam2 = CameraServer.startAutomaticCapture();
    cam1.setResolution(10, 10);
    cam2.setResolution(10, 10);
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    navx = new AHRS(SPI.Port.kMXP);
    control1 = new XboxController(0);
    pdh = new PowerDistribution(1, ModuleType.kRev);

    //MODULO FR
    motorAngleFR = new CANSparkMax(1, MotorType.kBrushless);
    motorSpeedFR = new CANSparkMax(5, MotorType.kBrushless);
    motorAngleFR.setInverted(false);
    motorAngleFR.setInverted(false);
    motorAngleFR.setIdleMode(IdleMode.kBrake);
    motorSpeedFR.setIdleMode(IdleMode.kBrake);
    encoderFR = motorAngleFR.getEncoder();
    
    kPFR = 0.1; 
    kIFR = 0;
    kDFR = 0.1; 
    kIzFR = 0;  
    kMaxOutputFR = 1; 
    kMinOutputFR = -1;
    pidControllerFR = motorAngleFR.getPIDController();

    pidControllerFR.setP(kPFR);
    pidControllerFR.setI(kIFR);
    pidControllerFR.setD(kDFR);
    pidControllerFR.setIZone(kIzFR);
    pidControllerFR.setOutputRange(kMinOutputFR, kMaxOutputFR);

    //MODULO BR
    motorAngleBR = new CANSparkMax(20, MotorType.kBrushless);
    motorSpeedBR = new CANSparkMax(4, MotorType.kBrushless);
    motorAngleBR.setInverted(false);
    motorAngleBR.setInverted(false);
    motorAngleBR.setIdleMode(IdleMode.kBrake);
    motorSpeedBR.setIdleMode(IdleMode.kBrake);
    encoderBR = motorAngleBR.getEncoder();

    kPBR = 0.1; 
    kIBR = 0;
    kDBR = 0.1; 
    kIzBR = 0;  
    kMaxOutputBR = 1; 
    kMinOutputBR = -1;
    pidControllerBR = motorAngleBR.getPIDController();

    pidControllerBR.setP(kPFR);
    pidControllerBR.setI(kIFR);
    pidControllerBR.setD(kDFR);
    pidControllerBR.setIZone(kIzFR);
    pidControllerBR.setOutputRange(kMinOutputFR, kMaxOutputFR);

    //MODULO BL
    motorAngleBL = new CANSparkMax(11, MotorType.kBrushless);
    motorSpeedBL = new CANSparkMax(6, MotorType.kBrushless);
    motorAngleBL.setInverted(false);
    motorAngleBL.setInverted(false);
    motorAngleBL.setIdleMode(IdleMode.kBrake);
    motorSpeedBL.setIdleMode(IdleMode.kBrake);
    encoderBL = motorAngleBL.getEncoder();

    kPBL = 0.1; 
    kIBL = 0;
    kDBL = 0.1; 
    kIzBL = 0;  
    kMaxOutputBL = 1; 
    kMinOutputBL = -1;
    pidControllerBL = motorAngleBL.getPIDController();

    pidControllerBL.setP(kPFR);
    pidControllerBL.setI(kIFR);
    pidControllerBL.setD(kDFR);
    pidControllerBL.setIZone(kIzFR);
    pidControllerBL.setOutputRange(kMinOutputFR, kMaxOutputFR);
    //MODULO FL
    motorAngleFL = new CANSparkMax(22, MotorType.kBrushless);
    motorSpeedFL = new CANSparkMax(7, MotorType.kBrushless);
    motorAngleFL.setInverted(false);
    motorAngleFL.setInverted(false);
    motorAngleFL.setIdleMode(IdleMode.kBrake);
    motorSpeedFL.setIdleMode(IdleMode.kBrake);
    encoderFL = motorAngleFL.getEncoder();

    kPFL = 0.1; 
    kIFL = 0;
    kDFL = 0.1; 
    kIzFL = 0;  
    kMaxOutputFL = 1; 
    kMinOutputFL = -1;
    pidControllerFL = motorAngleFL.getPIDController();

    pidControllerFL.setP(kPFL);
    pidControllerFL.setI(kIFL);
    pidControllerFL.setD(kDFL);
    pidControllerFL.setIZone(kIzFL);
    pidControllerFL.setOutputRange(kMinOutputFL, kMaxOutputFL);  
    
    //Inake
    motorIntake = new CANSparkMax(0, MotorType.kBrushless);
    motorIntake.setInverted(false);

    kPIntake = 0.1; 
    kIIntake = 0;
    kDIntake = 0.1; 
    kIzIntake = 0;  
    kMaxOutputIntake = 1; 
    kMinOutputIntake = -1;
    pidControllerIntake = motorIntake.getPIDController();

    pidControllerIntake.setP(kPIntake);
    pidControllerIntake.setI(kIIntake);
    pidControllerIntake.setD(kDIntake);
    pidControllerIntake.setIZone(kIzIntake);
    pidControllerIntake.setOutputRange(kMinOutputIntake, kMaxOutputIntake);  

    //Shooter
    motorShooterR = new CANSparkMax(0, MotorType.kBrushless);
    motorShooterR.setInverted(false);

    kPShooter = 0.1; 
    kIShooter = 0;
    kDShooter = 0.1; 
    kIzShooter = 0;  
    kMaxOutputShooter = 1; 
    kMinOutputShooter = -1;
    pidControllerShooterR = motorShooterR.getPIDController();
    pidControllerShooterL = motorShooteL.getPIDController();

    pidControllerShooterL.setP(kPShooter);
    pidControllerShooterL.setI(kIShooter);
    pidControllerShooterL.setD(kDShooter);
    pidControllerShooterL.setIZone(kIzShooter);
    pidControllerShooterL.setOutputRange(kMinOutputShooter, kMaxOutputShooter);  

    pidControllerShooterR.setP(kPShooter);
    pidControllerShooterR.setI(kIShooter);
    pidControllerShooterR.setD(kDShooter);
    pidControllerShooterR.setIZone(kIzShooter);
    pidControllerShooterR.setOutputRange(kMinOutputShooter, kMaxOutputShooter); 

    limitL = new DigitalInput(0);
    limitR = new DigitalInput(1);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    encoderFR.setPosition(0);
    encoderBR.setPosition(0);
    encoderBL.setPosition(0);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Drive();

    if(control1.getAButtonPressed() && !control1.getBButtonPressed() && !control1.getXButtonPressed() && !limitL.get() && !limitR.get()){
      intake();
    }

    if(control1.getBButtonPressed() && !control1.getAButtonPressed() && !control1.getXButtonPressed()){
      shooter();
    }
    if(control1.getXButtonPressed() && !control1.getBButtonPressed() && !control1.getAButtonPressed() ){
      takeIn();
    }
  }
  private void Drive(){
    int L = 1, W = 1;
    double x1 = control1.getLeftX();
    double y1 = control1.getLeftY();
    double x2 = control1.getRightX();
    double r = Math.sqrt((L*L)+(W*W));
    y1 *= -1;

    if (Math.abs(x1) < 0.1 && Math.abs(y1) < 0.1 && Math.abs(x2) < 0.1) {
        x1 = 0; y1 = 0; x2 = 0;
    }

    double angleNaVX = Math.toRadians(navx.getAngle()+180);

    double matrizR[][] = new double[2][2];
    double ejesR[][] = new double[1][2];

    matrizR[0][0] = Math.cos(angleNaVX);
    matrizR[0][1] = -Math.sin(angleNaVX);
    matrizR[1][0] = Math.sin(angleNaVX);
    matrizR[1][1] = Math.cos(angleNaVX);
    ejesR[0][0] = ((matrizR[0][0]*x1)+(matrizR[0][1]*y1));     //X
    ejesR[0][1] = ((matrizR[1][0]*x1)+(matrizR[1][1]*y1));    // Y
    double a = ejesR[0][0] - x2 * (L / r);
    double b = ejesR[0][0] + x2 * (L / r);
    double c = ejesR[0][1] - x2 * (W / r);
    double d = ejesR[0][1] + x2 * (W / r);
    // Cálculo de las velocidades
    double backLeftSpeed = Math.sqrt((a * a) + (d * d));
    double backRightSpeed = Math.sqrt((a * a) + (c * c));
    double frontLeftSpeed = Math.sqrt((b * b) + (d * d));
    double frontRightSpeed = Math.sqrt((b * b) + (c * c));

    // Cálculo de los ángulos en radianes y normalización entre -0.5 y 0.5
    double backLeftAngle = normalizeAngle(Math.atan2(a, d) / (2 * Math.PI));
    double backRightAngle = normalizeAngle(Math.atan2(a, c) / (2 * Math.PI));
    double frontLeftAngle = normalizeAngle(Math.atan2(b, d) / (2 * Math.PI));
    double frontRightAngle = normalizeAngle(Math.atan2(b, c) / (2 * Math.PI));

    // Optimización de ángulos y ajuste de velocidades
    if (Math.abs(backRightAngle) > 0.25) {
        backRightSpeed = -backRightSpeed;
        backRightAngle = normalizeAngle(backRightAngle + 0.5);
    }
    if (Math.abs(backLeftAngle) > 0.25) {
        backLeftSpeed = -backLeftSpeed;
        backLeftAngle = normalizeAngle(backLeftAngle + 0.5);
    }
    if (Math.abs(frontRightAngle) > 0.25) {
        frontRightSpeed = -frontRightSpeed;
        frontRightAngle = normalizeAngle(frontRightAngle + 0.5);
    }
    if (Math.abs(frontLeftAngle) > 0.25) {
        frontLeftSpeed = -frontLeftSpeed;
        frontLeftAngle = normalizeAngle(frontLeftAngle + 0.5);
    } 
    double reduceSpeed = 0.2;

    double ticksFR = 21.499897* frontRightAngle;
    pidControllerFR.setReference(ticksFR, ControlType.kPosition);
    motorSpeedFR.set(frontRightSpeed*reduceSpeed);

    double ticksBR = 21.499897* backRightAngle;
    pidControllerBR.setReference(ticksBR, ControlType.kPosition);
    motorSpeedBR.set(backRightSpeed*reduceSpeed);
    
    double ticksBL = 21.499897* backLeftAngle;
    pidControllerBL.setReference(ticksBL, ControlType.kPosition);
    motorSpeedBL.set(backLeftSpeed*reduceSpeed);

    double ticksFL = 21.499897* frontLeftAngle;
    pidControllerFL.setReference(ticksFL, ControlType.kPosition);
    motorSpeedFL.set(frontLeftSpeed*reduceSpeed);
  }

  private double normalizeAngle(double angle) {
    angle %= 1;
    if (angle > 0.5) angle -= 1.0;
    if (angle < -0.5) angle += 1.0;
    return angle;
}
private void shooter(){
  pidControllerShooterR.setReference(-5108, ControlType.kVelocity);
  pidControllerShooterL.setReference(5108, ControlType.kVelocity);
}
private void takeIn(){
  pidControllerShooterR.setReference(5108, ControlType.kVelocity);
  pidControllerShooterL.setReference(-5108, ControlType.kVelocity);
  pidControllerIntake.setReference(5108, ControlType.kVelocity);
}
private void intake(){
  pidControllerIntake.setReference(-5108, ControlType.kVelocity);
}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

}


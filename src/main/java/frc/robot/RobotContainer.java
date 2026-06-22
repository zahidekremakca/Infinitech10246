// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
// RobotContainer'ın üst kısmında


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.AutoAimAndShootCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class RobotContainer {
    // RobotContainer'ın üst kısmında
    // Değişken isminin m_shooter olduğundan emin ol
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final VisionSubsystem m_vision = new VisionSubsystem();
    // --- KİLİTLİ HEDEF HAFIZASI ---
    private double m_lockedTagX = 0;
    private double m_lockedTagY = 0;
    private boolean m_isTagLocked = false;

    private final double LENS_HEIGHT = 0.5; // Kameranın yerden yüksekliği (Metre)
    private final double TARGET_HEIGHT = 1.2573; // AprilTag'in yerden yüksekliği (Metre)
    private final double CAMERA_MOUNT_ANGLE = 30.0; // Kameranın montaj açısı (Derece)

    // NetworkTable for the vision system (e.g., Limelight)
    private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final com.ctre.phoenix6.Orchestra m_orchestra = new com.ctre.phoenix6.Orchestra();
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // NamedCommands kayıtlarını PathPlanner konfigürasyonundan önce yapmak en güvenli yoldur
        
        NamedCommands.registerCommand("shooter", new AutoAimAndShootCommand(drivetrain, m_shooter));
        NamedCommands.registerCommand("shooterbitir", m_shooter.shooterKapatKomutu());
        NamedCommands.registerCommand("feeder", m_shooter.feederAcKomutu());
        NamedCommands.registerCommand("hiza", drivetrain.applyRequest(() -> 
            drive.withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(limelight_aim_proportional())
        ));

        // Dosya adının tam doğru olduğundan emin ol
       
       for (int i = 0; i < 4; i++) {
        // Her modüldeki Drive (Sürüş) motorunu ekle
        m_orchestra.addInstrument(drivetrain.getModule(i).getDriveMotor());
        // Her modüldeki Steer (Dönüş/Açı) motorunu ekle
        m_orchestra.addInstrument(drivetrain.getModule(i).getSteerMotor());
    }   
    
       
   

        

        


        drivetrain.configurePathPlanner();
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Otonom Secici", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() ->
            drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        joystick.start().onTrue(Commands.runOnce(() -> {
        String musicPath = Filesystem.getDeployDirectory().getAbsoluteFile() + "/output.chrp";
        m_orchestra.loadMusic(musicPath);
        m_orchestra.play();
        }));

        joystick.back().onTrue(Commands.runOnce(m_orchestra::stop));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));


        joystick.rightBumper().whileTrue(new AutoAimAndShootCommand(drivetrain, m_shooter));

        joystick.rightTrigger().whileTrue(
            drivetrain.applyRequest(() ->
            drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                .withRotationalRate(limelight_aim_proportional())
            )
        );
        //humanfeeder
        joystick.y().whileTrue(m_shooter.humanfeederCommand());
        // Sol tetiğe basılı tutunca shooter ve feeder'ı geri döndür
        joystick.leftTrigger().toggleOnTrue(
            m_shooter.feederSequenceCommand().finallyDo(() -> m_shooter.feederStop())

        );      
        // Sol tetiğe basılı tutunca shooter ve feeder'ı geri döndür
          //tersshooter
        joystick.x().whileTrue(m_shooter.reverseShooterCommand()); 
          
         //tersshooter
        // Otomatik hedefleme ve mesafeye göre atış
        

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.





        
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // SmartDashboard/Shuffleboard üzerinden seçtiğin PathPlanner otonomunu döndürür
        return autoChooser.getSelected();
    }

     double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .023;

    double tx = m_vision.getTargetYaw();
    double ty = m_table.getEntry("ty").getDouble(0.0);
    double tid = m_vision.getTargetID();
    var currentPose = drivetrain.getState().Pose;

    // Sadece 9, 10, 25 ve 26 numaralı ID'leri kabul et
    boolean isCorrectTag = (tid == 9 || tid == 10 || tid == 25 || tid == 26);

    // Geçerli bir tag gördüğümüzde ve istediğimiz ID ise konumu kilitle
    if (m_vision.hasTarget() && isCorrectTag) {
        // 1. Hedefe olan mesafeyi hesapla
        double angleToGoalRad = Math.toRadians(CAMERA_MOUNT_ANGLE + ty);
        double distance = (TARGET_HEIGHT - LENS_HEIGHT) / Math.tan(angleToGoalRad);

        // 2. Hedefin saha üzerindeki mutlak açısını hesapla (Robot açısı - tx)
        // Not: Limelight tx sağa (+) olduğu için robot açısından çıkarıyoruz
        double angleToTagRad = currentPose.getRotation().getRadians() - Math.toRadians(tx);

        // 3. Hedefin saha koordinatlarını hesapla ve hafızaya al
        m_lockedTagX = currentPose.getX() + distance * Math.cos(angleToTagRad);
        m_lockedTagY = currentPose.getY() + distance * Math.sin(angleToTagRad);
        m_isTagLocked = true;

        // Eğer hedef belirlenen tolerans (0.5 derece) içindeyse hizalanmış kabul et ve dur
        if (Math.abs(tx) < VisionConstants.kTolerance) {
            SmartDashboard.putBoolean("Vision/AlignedToTarget", true);
            return 0.0;
        }

        SmartDashboard.putBoolean("Vision/AlignedToTarget", false);
        // Görüş varken doğrudan Limelight tx ile hassas hizalan
        double targetingAngularVelocity = tx * kP * MaxAngularRate * -1.0;
        return targetingAngularVelocity;
    } 
    
    // Eğer hedef görünmüyorsa ama daha önce bir yer kilitlendiyse odometri ile oraya dön
    else if (m_isTagLocked) {
        // Robot ile kilitli nokta arasındaki açıyı hesapla
        double targetAngleRad = Math.atan2(m_lockedTagY - currentPose.getY(), m_lockedTagX - currentPose.getX());
        double currentAngleRad = currentPose.getRotation().getRadians();

        double rotationError = MathUtil.angleModulus(targetAngleRad - currentAngleRad);
        return rotationError * 4.0; // kP = 4.0 odometri dönüşü için
    }

    return 0.0;
  }

    public void periodic() {
        // Debug için verileri ekrana basalım (ham NT durumunu da yaz)
        boolean ntConnected = NetworkTableInstance.getDefault().isConnected();
        SmartDashboard.putBoolean("Vision/NTConnected", ntConnected);

        double rawTx = m_table.getEntry("tx").getDouble(Double.NaN);
        double rawTv = m_table.getEntry("tv").getDouble(Double.NaN);
        SmartDashboard.putNumber("Vision/Raw_tx", rawTx);
        SmartDashboard.putNumber("Vision/Raw_tv", rawTv);

        if (Double.isNaN(rawTx) || Double.isNaN(rawTv)) {
            SmartDashboard.putString("Vision/Status", "NT keys missing or not connected");
        } else {
            SmartDashboard.putString("Vision/Status", "OK");
        }

        SmartDashboard.putNumber("Vision/TargetX", m_vision.getTargetYaw());
        SmartDashboard.putBoolean("Vision/HasTarget", m_vision.hasTarget());
    }

}

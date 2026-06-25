package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;


public class ShooterSubsystem extends SubsystemBase {
    // Motorlar TunerConstants içindeki "CANivore" ismiyle tanımlanan hatta bağlanır
    private final TalonFX shooterMotor = new TalonFX(10, TunerConstants.kCANBus);
private final TalonFX feederMotor = new TalonFX(9, TunerConstants.kCANBus);

    // 5000 RPM'i Rotations Per Second'a (RPS) çeviriyoruz: 5000 / 60 ≈ 83.3 RPS
    private double m_defaultTargetRPS = 4500.0 / 60.0;
    private final double kShooterTolerance = 0.5; // Hedefe ne kadar yakın olmalı (RPS cinsinden)
    private double m_activeTargetRPM = 0;

    public ShooterSubsystem() {
        TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();
        TalonFXConfiguration feederConfigs = new TalonFXConfiguration();

        // Shooter Ayarları
        shooterConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        // Shooter Akım ve Voltaj Ayarları
        // Sorun teşhis edilene kadar sınırı 60A'e düşürüyoruz (Güvenlik için)
        shooterConfigs.CurrentLimits.StatorCurrentLimit = 60.0; 
        shooterConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterConfigs.CurrentLimits.SupplyCurrentLimit = 40.0; // Aküden çekilecek maksimum akım (Sigorta koruması)
        shooterConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        shooterConfigs.Voltage.PeakForwardVoltage = 12.0;
        shooterConfigs.Voltage.PeakReverseVoltage = -12.0;

        // Ramp (Hızlanma Süresi): Akım patlamasını önlemek için 0.1 saniyede tam güce çıkar
        shooterConfigs.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;

        // Yüksek hızlar için PID değerlerinde kV (Feedforward) kritik rol oynar
        shooterConfigs.Slot0.kP = 0.05; // 0.11'den daha yumuşak bir tepki için 0.05'e düşürüldü
        shooterConfigs.Slot0.kV = 0.011; // 0.115'den 0.011'e düzeltildi (Hız kontrolü için kritik)

        // Feeder Ayarları (Shooter'ın zıttı yöne dönecek şekilde ayarlandı)
        feederConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        feederConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Feeder Akım Ayarları (Sıkışma durumunda motoru korumak için daha düşük limit)
        feederConfigs.CurrentLimits.StatorCurrentLimit = 60.0;
        feederConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        feederConfigs.CurrentLimits.SupplyCurrentLimit = 30.0;
        feederConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        feederConfigs.Voltage.PeakForwardVoltage = 12.0;
        feederConfigs.Voltage.PeakReverseVoltage = -12.0;

        shooterMotor.getConfigurator().apply(shooterConfigs);
        feederMotor.getConfigurator().apply(feederConfigs);
    }

    /**
     * Shooter'ı çalıştıran ve hızlandığında feeder'ı açan komut.
     */
    public Command shootSequenceCommand() {
        return run(() -> {
            runShooterWithRPM(m_defaultTargetRPS * 60.0);
        }).finallyDo(() -> {
            // Komut bittiğinde (tuş bırakıldığında) her şeyi durdur
            stop();
        });
    }

    /**
     * Motorları verilen RPM değerine göre sürer.
     * AutoAim komutu mesafeyi hesaplayıp bu metodu çağıracaktır.
     */
    public void runShooterWithRPM(double targetRPM) {
        this.m_activeTargetRPM = targetRPM;
        double targetRPS = targetRPM / 60.0;
        shooterMotor.setControl(new VelocityDutyCycle(targetRPS));

        double currentRPS = shooterMotor.getVelocity().getValueAsDouble();
    
        if (Math.abs(currentRPS) >= Math.abs(targetRPS) - kShooterTolerance) {
            feederMotor.setControl(new DutyCycleOut(0.6));
        } else {
            feederMotor.setControl(new DutyCycleOut(0));
        }
    }

    /**
     * Shooter ve Feeder'ı ters yönde çalıştıran komut (Sıkışmaları gidermek için).
     */
    public Command reverseShooterCommand() {
        return run(() -> {
            // İki motoru da düşük güçte ters yöne döndür
            shooterMotor.setControl(new DutyCycleOut(-0.9));
            feederMotor.setControl(new DutyCycleOut(-0.9));
        }).finallyDo(this::stop);
    }

    /**
     * Feeder komutu (shootSequenceCommand'e benzer).
     * Shooter hedef hıza ulaştığında hem shooter hem feeder saat yönünün tersine döner.
     */
    public Command feederSequenceCommand() {
        return run(() -> {
            // Shooter'ı hedef hıza sür (CCW pozitif olacak şekilde)
            shooterMotor.setControl(new VelocityDutyCycle(m_defaultTargetRPS));
            feederMotor.setControl(new DutyCycleOut(-0.7)); // Feeder'ı sürekli çalıştır (tuş bırakılana kadar)
        }).finallyDo(this::stop);

    }

    public Command humanfeederCommand() {
        return run(() -> {
            // Shooter'ı hedef hıza sür (CCW pozitif olacak şekilde)
            shooterMotor.setControl(new VelocityDutyCycle(-m_defaultTargetRPS));
            feederMotor.setControl(new DutyCycleOut(0.8)); // Feeder'ı sürekli çalıştır (tuş bırakılana kadar)
        }).finallyDo(this::stop);
    }
    public Command feederStop() {
        return run(() -> {
            // Shooter'ı hedef hıza sür (CCW pozitif olacak şekilde)
            shooterMotor.setControl(new VelocityDutyCycle(0)); // Hedef hıza ulaşmak için ters yönde çalıştır));
            feederMotor.setControl(new DutyCycleOut(0)); // Feeder'ı sürekli çalıştır (tuş bırakılana kadar)
        });
        
    }

    public void stop() {
        shooterMotor.setControl(new DutyCycleOut(0));
        feederMotor.setControl(new DutyCycleOut(0));
    }
    

    

    public void shooterKapat() {
        // motor.set(0);
        shooterMotor.setControl(new DutyCycleOut(0));
        feederMotor.setControl(new DutyCycleOut(0));

    }

   
    public Command shooterKapatKomutu() {
        return new InstantCommand(() -> this.shooterKapat(), this);
    }
    
    public Command feederAcKomutu() {
        return feederSequenceCommand();
    }
        


    @Override
    public void periodic() {
        // Shooter Telemetri Verileri
        double currentRPM = shooterMotor.getVelocity().getValueAsDouble() * 60.0;
        SmartDashboard.putNumber("Shooter/RPM", currentRPM);
        SmartDashboard.putBoolean("Shooter/Ready", currentRPM >= (m_defaultTargetRPS * 60.0 - 10));
        
        // Paylaştığın verilere göre Shooter (ID 10) detayları
        SmartDashboard.putNumber("Shooter/TorqueCurrent_Amps", shooterMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/SupplyCurrent_Amps", shooterMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/DutyCycle", shooterMotor.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/SupplyVoltage", shooterMotor.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/MotorVoltage", shooterMotor.getMotorVoltage().getValueAsDouble());
        
        // Kritik Uyarı: Akım çok yüksekse Dashboard'da kullanıcıyı uyar
        boolean isStalling = shooterMotor.getStatorCurrent().getValueAsDouble() > 50.0 && currentRPM < 500;
        SmartDashboard.putBoolean("Shooter/MECHANICAL_STALL_WARNING", isStalling);

        // Feeder (ID 9) detayları
        SmartDashboard.putNumber("Feeder/TorqueCurrent_Amps", feederMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Feeder/SupplyCurrent_Amps", feederMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Feeder/DutyCycle", feederMotor.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("Feeder/SupplyVoltage", feederMotor.getSupplyVoltage().getValueAsDouble());
    }
}

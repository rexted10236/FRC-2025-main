package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMax.MotorType;
import com.revrobotics.spark.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax.IdleMode;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
    // ✅ SparkMax motorlarını tanımla (2025 REVLib)
    private SparkMax rightFront;
    private SparkMax rightRear;
    private SparkMax leftFront;
    private SparkMax leftRear;

    // ✅ Xbox kumandası
    private XboxController controller;

    @Override
    public void robotInit() {
        // ✅ Motorları CAN ID'lerine göre başlat
        rightFront = new SparkMax(2, MotorType.kBrushless);
        rightRear = new SparkMax(3, MotorType.kBrushless);
        leftFront = new SparkMax(4, MotorType.kBrushless);
        leftRear = new SparkMax(5, MotorType.kBrushless);

        // ✅ Motorları yapılandır
        configureMotor(rightFront);
        configureMotor(rightRear);
        configureMotor(leftFront);
        configureMotor(leftRear);

        // ✅ Xbox Controller'ı başlat
        controller = new XboxController(0);

        SmartDashboard.putString("Motor Status", "Initialized");
    }

    private void configureMotor(SparkMax motor) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.setIdleMode(IdleMode.kBrake); // Motorlar fren modunda olacak
        config.setSmartCurrentLimit(40); // Maksimum akım limiti 40A
        motor.configure(config);
    }

    @Override
    public void teleopPeriodic() {
        // ✅ Joystick kontrolü ile hız ayarla
        double forward = -controller.getLeftY(); // İleri-Geri
        double turn = controller.getRightX(); // Sağa-Sola Dönüş

        // ✅ Tank Drive hesaplama
        double leftSpeed = forward + turn;
        double rightSpeed = forward - turn;

        // ✅ Motorlara hız değerlerini gönder
        leftFront.set(leftSpeed);
        leftRear.set(leftSpeed);
        rightFront.set(rightSpeed);
        rightRear.set(rightSpeed);

        // ✅ SmartDashboard'ta bilgileri göster
        SmartDashboard.putNumber("Left Speed", leftSpeed);
        SmartDashboard.putNumber("Right Speed", rightSpeed);
    }
}

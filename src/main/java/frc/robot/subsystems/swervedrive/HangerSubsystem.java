package frc.robot.subsystems.swervedrive;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class HangerSubsystem extends SubsystemBase{
    private CANSparkMax hangingMotor = new CANSparkMax(16, MotorType.kBrushless);

    public HangerSubsystem(){
        hangingMotor.setSmartCurrentLimit(80);
    }

    public void pullUP(){
        hangingMotor.set(0.4);
    }

    public void pullDown(){
        hangingMotor.set(-0.4);
    }

    public void stopPull(){
        hangingMotor.set(0);
    }

}

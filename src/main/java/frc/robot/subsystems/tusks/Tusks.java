package frc.robot.subsystems.tusks;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TuskConstants;
import frc.robot.subsystems.utils.tusks.tuskPositions;

public class Tusks extends SubsystemBase{

    private SparkFlex tuskRollerMotor;
    private SparkMax tuskPivotMotor;
    private PIDController tuskPivotPIDController;
    private tuskPositions currentState;

    public Tusks(){
        tuskRollerMotor = new SparkFlex(TuskConstants.kTuskRollerMotorId, MotorType.kBrushless);
        tuskPivotMotor = new SparkMax(TuskConstants.kTuskPivotMotorId, MotorType.kBrushless);
        tuskPivotPIDController = new PIDController(TuskConstants.kPivotP, TuskConstants.kPivotI, TuskConstants.kPivotD);
        currentState = tuskPositions.IN;

    }

    public void setRollerPower(double power){
        tuskRollerMotor.set(power);
    }

    public void setPivotPower(double power){
        tuskPivotMotor.set(power);
    }

    public void stopRoller(){
        tuskRollerMotor.set(0);
    }

    public void stopPivot(){
        tuskPivotMotor.set(0);
    }

    public void stopAll(){
        stopRoller();
        stopPivot();
    }

    public PIDController getPIDController(){
        return tuskPivotPIDController;
    }

    public double getPivotPosition(){
        return tuskPivotMotor.getEncoder().getPosition();
    }

    @Override
    public void periodic(){
        //System.out.println("Pivot Position: " + tuskPivotMotor.getEncoder().getPosition());
        if (currentState == tuskPositions.IN){
            SmartDashboard.putString("Tusk Position", "IN");
        } else if (currentState == tuskPositions.OUT){
            SmartDashboard.putString("Tusk Position", "OUT");
        }
    }

    public tuskPositions getCurrentState(){
        return currentState;
    }

    public void setCurrentState(tuskPositions state){
        currentState = state;
    }


    
}

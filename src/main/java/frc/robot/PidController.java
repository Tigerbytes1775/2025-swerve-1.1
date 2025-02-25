package frc.robot;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkFlex;

public class PidController {

    private  final double p;
    private final double i;
    private final double d;

    private double targetPoint;
    private double currentPoint;
    private double currentVelocity;
    private double error;

    private double integral;

    private final DoubleSupplier motorEncoder;
    private final DoubleSupplier motorVelocity;

    private final double restingError = 0.1;
    private final double restingVelocity = 0.01;

    public PidController(double p, double i, double d, SparkMax motor) {

        this.p = p;
        this.i = i;
        this.d = d;
        
        this.motorEncoder = () -> motor.getAbsoluteEncoder().getPosition();
        this.motorVelocity = () -> motor.getAbsoluteEncoder().getVelocity();
    }

    public PidController(double p, double i, double d, SparkFlex motor) {
        this.p = p;
        this.i = i;
        this.d = d;

        this.motorEncoder = () -> motor.getAbsoluteEncoder().getPosition();
        this.motorVelocity = () -> motor.getAbsoluteEncoder().getVelocity();
    }

    

    public void setTargetPoint(double targetPoint) {
        this.targetPoint = targetPoint;

        integral = 0;
    }

    public void update() {
        currentPoint = motorEncoder.getAsDouble();
        currentVelocity = motorVelocity.getAsDouble();

        error = targetPoint - currentPoint;

        integral += error / 50;
    }
    
    public double GetForce() {

        double force = p * error + i * integral + d * currentVelocity;
        return force;
    }

    public boolean IsAtTarget() {
        return error < restingError && currentVelocity < restingVelocity;
    }
}

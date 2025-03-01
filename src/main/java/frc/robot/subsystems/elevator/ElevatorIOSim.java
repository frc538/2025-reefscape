package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;

public class ElevatorIOSim implements ElevatorIO {
        // Use only the left side, leaving the DCMotor model handle twice the effort
        SparkMax mLeft;

        SparkMaxConfig mLeftConfig = new SparkMaxConfig();

        double mReferencePosition = 0.0;
        double mArbFF = 0.0;

        private final SparkClosedLoopController leftController;

        /*
         * Gear 1 - 2106 MAXPlanetary Universal Input Stage
         * Gear 2 - 2129 MAXPlanetary 9:1 Cartridge
         * Gear 3 - 2104 MAXPlanetary 1/2" Hex Socket Output
         * AM4779 - 26T #25 SERIES - pitch diameter 2.074
         * 
         * Idler sprockets - 19T - 41mm OD
         * 
         */

        // This gearbox represents a gearbox containing 2 NEOs.
        private final DCMotor m_elevatorGearbox = DCMotor.getNEO(2);
        private final double kElevatorGearing = 9.0;
        private final double kCarriageMass = 10.0;
        private final double kElevatorDrumRadius = Units.inchesToMeters(2.074);
        private final double kMinElevatorHeightMeters = 0.0;
        private final double kMaxElevatorHeightMeters = Units.feetToMeters(6.0);

        // Simulation classes help us simulate what's going on, including gravity.
        private final ElevatorSim m_elevatorSim = new ElevatorSim(
                        m_elevatorGearbox,
                        kElevatorGearing,
                        kCarriageMass,
                        kElevatorDrumRadius,
                        kMinElevatorHeightMeters,
                        kMaxElevatorHeightMeters,
                        true,
                        0,
                        0.01,
                        0.0);

        // create the Spark MAX sim object
        SparkMaxSim mSimLeft;

        public ElevatorIOSim(int leftId) {
                mLeft = new SparkMax(leftId, MotorType.kBrushless);

                mSimLeft = new SparkMaxSim(mLeft, m_elevatorGearbox);

                mLeftConfig
                                .idleMode(IdleMode.kBrake)
                                .inverted(false)
                                .smartCurrentLimit(Constants.ElevatorConstants.ElevatorCurrentLimit);
                mLeftConfig.encoder
                                .positionConversionFactor(Constants.ElevatorConstants.ElevatorPositionConversionFactor)
                                .velocityConversionFactor(Constants.ElevatorConstants.ElevatorVelocityConversionFactor);
                mLeftConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pid(1, 0, 0)
                                .outputRange(-1, 1);
                mLeftConfig.closedLoop.maxMotion
                                .allowedClosedLoopError(Constants.ElevatorConstants.allowedClosedLoopError)
                                .maxAcceleration(Constants.ElevatorConstants.maxAcceleration)
                                .maxVelocity(Constants.ElevatorConstants.maxVelocity);

                mLeft.configure(mLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

                leftController = mLeft.getClosedLoopController();
                setReference(0.0);
        }

        // ¯\_(ツ)_/¯\\

        public void updateInputs(ElevatorIOInputs inputs) {
                m_elevatorSim.setInput(mSimLeft.getAppliedOutput() * RoboRioSim.getVInVoltage());
                m_elevatorSim.update(0.02);

                mSimLeft.iterate(mSimLeft.getVelocity() / Constants.ElevatorConstants.ElevatorPositionConversionFactor,
                                RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
                                0.02);

                // SimBattery estimates loaded battery voltages
                // This should include all motors being simulated
                RoboRioSim.setVInVoltage(
                                BatterySim.calculateDefaultBatteryLoadedVoltage(mSimLeft.getMotorCurrent()));

                inputs.leftAppliedBusVoltage = mSimLeft.getAppliedOutput() * RoboRioSim.getVInVoltage();
                inputs.leftAppliedCurrent = mSimLeft.getMotorCurrent();
                inputs.leftAppliedOutput = mSimLeft.getAppliedOutput();
                inputs.height = m_elevatorSim.getPositionMeters();
        }

        public void setReference(double position) {
                mReferencePosition = position;
                Logger.recordOutput("Sim/Commanded Position", position);
                System.out.println(position);
                leftController.setReference(mReferencePosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, mArbFF);
        }
        public void setConfiguration(double arbFF) {
                mArbFF = arbFF;
                leftController.setReference(mReferencePosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, mArbFF);
        }
}

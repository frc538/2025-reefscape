package frc.robot.subsystems.Intake;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.spark.config.LimitSwitchConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeIOImplementation {
        private static Servo IntakeServo;
        private final DigitalInput LimitSwitchCoral;
        private final DigitalInput LimitSwitchAlgae;
        
        public IntakeIOImplementation(int ServoChannel, int dioCoralDetected, int dioAlgaeDetected){
            IntakeServo = new Servo(ServoChannel);
        LimitSwitchAlgae = new DigitalInput(dioAlgaeDetected);
        LimitSwitchCoral = new DigitalInput(dioCoralDetected);
   } 
   public static Command coralIntake(){
        IntakeServo.set(0.5);
                return null;
   }
   public static Command AlgaeIntake(){
        IntakeServo.set(-0.5);
                return null;
}
}

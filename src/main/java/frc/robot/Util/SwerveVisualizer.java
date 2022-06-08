package frc.robot.Util;

import javax.xml.crypto.dsig.Transform;
import javax.xml.namespace.QName;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveVisualizer {
    Field2d field;
    FieldObject2d robotBase;
    FieldObject2d frontLeftWheel;
    FieldObject2d frontRightWheel;
    FieldObject2d backLeftWheel;
    FieldObject2d backRightWheel;
    double robotWidth;
    double robotLength;

    public SwerveVisualizer(double robotWidth, double robotLength) {
        field = new Field2d();

        this.robotWidth = robotWidth;
        this.robotLength = robotLength;

        robotBase = field.getObject("robotBase");
        robotBase.setPose(new Pose2d(5,5,new Rotation2d()));

        frontLeftWheel = field.getObject("frontLeftWheel");
        Pose2d frontLeftOffset = new Pose2d(robotLength/2, robotWidth/2, new Rotation2d());
        System.out.println(robotBase.getPose().toString());
        Transform2d frontLeftTransform = new Transform2d(robotBase.getPose(), frontLeftOffset);
        frontLeftWheel.setPose(robotBase.getPose().plus(frontLeftTransform));
        // Transform2d temp0 = new Transform2d(frontLeftWheel.getPose().getTranslation(), robotBase.getPose().getRotation());
        // frontLeftWheel.setPose(robotBase.getPose().plus(temp0));

        frontRightWheel = field.getObject("frontRightWheel");
        Pose2d frontRightOffset = new Pose2d(robotLength/2, -robotWidth/2, new Rotation2d());
        Transform2d frontRightTransform = new Transform2d(robotBase.getPose(), frontRightOffset);
        frontRightWheel.setPose(robotBase.getPose().plus(frontRightTransform));
        // Transform2d temp1 = new Transform2d(frontRightWheel.getPose().getTranslation(), robotBase.getPose().getRotation());
        // frontLeftWheel.setPose(robotBase.getPose().plus(temp1));

        backLeftWheel = field.getObject("backLeftWheel");
        Pose2d backLeftOffset = new Pose2d(-robotLength/2, robotWidth/2, new Rotation2d());
        Transform2d backLeftTransform = new Transform2d(robotBase.getPose(), backLeftOffset);
        backLeftWheel.setPose(robotBase.getPose().plus(backLeftTransform));
        // Transform2d temp2 = new Transform2d(backLeftWheel.getPose().getTranslation(), robotBase.getPose().getRotation());
        // frontLeftWheel.setPose(robotBase.getPose().plus(temp2));

        backRightWheel = field.getObject("backRightWheel");
        Pose2d backRightOffset = new Pose2d(-robotLength/2, -robotWidth/2, new Rotation2d());
        Transform2d backRightTransform = new Transform2d(robotBase.getPose(), backRightOffset);
        backRightWheel.setPose(robotBase.getPose().plus(backRightTransform));
        // Transform2d temp3 = new Transform2d(backRightWheel.getPose().getTranslation(), robotBase.getPose().getRotation());
        // frontLeftWheel.setPose(robotBase.getPose().plus(temp3));

        SmartDashboard.putData(field);
    }

    public void update(Rotation2d fLWR, Rotation2d fRWR, Rotation2d bLWR, Rotation2d bRWR, Pose2d rBase) {
        frontLeftWheel.setPose(new Pose2d(frontLeftWheel.getPose().getTranslation(), fLWR));
        frontRightWheel.setPose(new Pose2d(frontRightWheel.getPose().getTranslation(), fLWR));
        backLeftWheel.setPose(new Pose2d(backLeftWheel.getPose().getTranslation(), fLWR));
        backRightWheel.setPose(new Pose2d(backRightWheel.getPose().getTranslation(), fLWR));

        robotBase.setPose(rBase);
    }


}

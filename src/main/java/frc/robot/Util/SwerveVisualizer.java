package frc.robot.Util;

import javax.xml.crypto.dsig.Transform;
import javax.xml.namespace.QName;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    Translation2d frontLeftRelativeLocation;
    Translation2d frontRightRelativeLocation;
    Translation2d backLeftRelativeLocation;
    Translation2d backRightRelativeLocation;
    double robotWidth;
    double robotLength;

    public SwerveVisualizer(double robotWidth, double robotLength) {
        field = new Field2d();

        this.robotWidth = robotWidth;
        this.robotLength = robotLength;

        robotBase = field.getObject("robotBase");
        robotBase.setPose(new Pose2d());

        frontLeftWheel = field.getObject("frontLeftWheel");
        frontLeftRelativeLocation = new Translation2d(this.robotLength/2, this.robotWidth/2);

        frontRightWheel = field.getObject("frontRightWheel");
        frontRightRelativeLocation = new Translation2d(this.robotLength/2, -this.robotWidth/2);

        backLeftWheel = field.getObject("backLeftWheel");
        backLeftRelativeLocation = new Translation2d(-this.robotLength/2, this.robotWidth/2);

        backRightWheel = field.getObject("backRightWheel");
        backRightRelativeLocation = new Translation2d(-this.robotLength/2, -this.robotWidth/2);

        this.update(new Rotation2d(), new Rotation2d(), new Rotation2d(), new Rotation2d(), 
            new Pose2d(new Translation2d(5, 4), Rotation2d.fromDegrees(45)));

        SmartDashboard.putData("f2", field);
    }

    /**
     * Updates each of the four swerve wheels as well as 'robotBase', rotating each of the wheels
     * about the robot center.
     * 
     * @param flWheelRot The rotation of the front left wheel
     * @param frWheelRot The rotation of the front left wheel
     * @param blWheelRot The rotation of the front left wheel
     * @param brWheelRot The rotation of the front left wheel
     * @param rBase The Pose2d object representing the robot base, i.e. the chasse
     */
    public void update(Rotation2d flWheelRot, Rotation2d frWheelRot, Rotation2d blWheelRot, Rotation2d brWheelRot, Pose2d rBase) {
        frontLeftWheel.setPose(revolveObject(frontLeftRelativeLocation, rBase, flWheelRot));
        frontRightWheel.setPose(revolveObject(frontRightRelativeLocation, rBase, frWheelRot));
        backLeftWheel.setPose(revolveObject(backLeftRelativeLocation, rBase, blWheelRot));
        backRightWheel.setPose(revolveObject(backRightRelativeLocation, rBase, brWheelRot));

        robotBase.setPose(rBase);
    }

    /**
     * Revolves a Pose2d object about another Pose2d object, perserving their relative angles.
     * 
     * @param relativePos The position of the Pose2d to be rotated relative to the Pose2d origin
     * @param rotateOrigin The Pose2d storing the origin
     * @param rotateAngle The angle to rotate about the origin
     * @return Pose2d
     */
    private Pose2d revolveObject(Translation2d relativePos, Pose2d rotateOrigin, Rotation2d rotateAngle) {
        Transform2d rotDelta = new Transform2d(relativePos, rotateAngle);
        return new Pose2d(rotDelta.getTranslation().rotateBy(rotateOrigin.getRotation()).plus(rotateOrigin.getTranslation()), 
            rotateOrigin.getRotation().plus(rotDelta.getRotation()));
    }
}

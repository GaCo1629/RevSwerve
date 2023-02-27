package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;

public final class Shared {
    public static boolean liftDown = false ;
    public static int gridNumber = 0;
    public static int gridLvl = 0;
    public static boolean cone = false;
    public static double armSetpoint = 0;
    public static boolean armInPosition = true;
    public static Pose2d currentPose = new Pose2d();
    public static Pose2d targetPose  = new Pose2d();

    public static BooleanSupplier inPosition = () -> armInPosition;
    
}

//file is used to get the information from the rpi running tensorflow for ball detection
package frc.team5973.robot.rapidreact;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

import frc.team5973.robot.RobotBase;
import frc.team5973.robot.subsystems.SubsystemBase;
public class DetectionData extends SubsystemBase {
    
    public NetworkTable resolution = NetworkTableInstance.getDefault().getTable("Resolution");

    public NetworkTable blueBalls = NetworkTableInstance.getDefault().getTable("Blue Balls");
    public NetworkTable redBalls  = NetworkTableInstance.getDefault().getTable("Red Balls");
    public NetworkTable robots    = NetworkTableInstance.getDefault().getTable("Robots");

    public NetworkTableEntry resX;
	public NetworkTableEntry resY;

    public NetworkTableEntry blueBallCenterX;
	public NetworkTableEntry blueBallCenterY;

    public NetworkTableEntry redBallCenterX;
	public NetworkTableEntry redBallCenterY;

    public NetworkTableEntry blueBallBoxWidth;
	public NetworkTableEntry blueBallBoxHeight;

    public NetworkTableEntry redBallBoxWidth;
	public NetworkTableEntry redBallBoxHeight;

    public NetworkTableEntry isBlueBallDetected;
    public NetworkTableEntry isRedBallDetected;

    private double distanceFromTarget;

    private Timer timer = new Timer();

    public DetectionData(RobotBase robot) {
        super(robot);
	}
    
    public double distanceFromTarget(String ballType) {
		// // distance constant divided by length between centers of contours
		// distanceFromTarget = DISTANCE_CONSTANT / getCenterX();
		
		double width = 9.5; //inches // width of the ball

        if(ballType.equals("blue")) {

            blueBallBoxWidth = blueBalls.getEntry("Box Width");
            double boxWidthentry = blueBallBoxWidth.getDouble(-1);
            distanceFromTarget = (width * 333.82) / boxWidthentry; // 333.82 is the focal length of the microsoft lifecam in px
            
        } else if(ballType.equals("red")) {

            redBallBoxWidth = redBalls.getEntry("Box Width");
            double boxWidthentry = redBallBoxWidth.getDouble(-1);
            distanceFromTarget = (width * 333.82) / boxWidthentry; // 333.82 is the focal length of the microsoft lifecam in px
            
        }

        return distanceFromTarget;

	}
    
    public double getResX() {
		
        resX = resolution.getEntry("Width");
		
		return resX.getDouble(-1);

	}

	public double getResY() {
		
        resX = resolution.getEntry("Height");
		
		return resY.getDouble(-1);

	}

    public double getBlueBallCenterX() {
        
        blueBallCenterX = blueBalls.getEntry("centerX");
        
        return blueBallCenterX.getDouble(-1);

    }

    public double getBlueBallCenterY() {
        
        blueBallCenterY = blueBalls.getEntry("centerY");
        
        return blueBallCenterY.getDouble(-1);

    }

    public double getRedBallCenterX() {
        
        redBallCenterX = redBalls.getEntry("centerX");
        
        return redBallCenterX.getDouble(-1);

    }

    public double getRedBallCenterY() {
        
        redBallCenterY = redBalls.getEntry("centerY");
        
        return redBallCenterY.getDouble(-1);

    }

    public double getBlueBallBoxWidth() {
        
        blueBallBoxWidth = blueBalls.getEntry("Box Width");

        return blueBallBoxWidth.getDouble(-1);

    }

    public double getBlueBallBoxHeight() {
       
        blueBallBoxHeight = blueBalls.getEntry("Box Height");

        return blueBallBoxHeight.getDouble(-1);

    }

    public double getRedBallBoxWidth() {
        
        redBallBoxWidth = redBalls.getEntry("Box Width");

        return redBallBoxWidth.getDouble(-1);

    }

    public double getRedBallBoxHeight() {
        
        redBallBoxHeight = redBalls.getEntry("Box Height");

        return redBallBoxHeight.getDouble(-1);

    }

    public boolean isBlueBallDetected() {
        isBlueBallDetected = blueBalls.getEntry("Blue Ball in Frame");

        timer.reset();
        timer.start();
        
        return isBlueBallDetected.getBoolean(false);
    }
    
    public boolean isRedBallDetected() {
        
        isRedBallDetected = redBalls.getEntry("Red Ball in Frame");
        
        return isRedBallDetected.getBoolean(false);

    }

    public boolean isAnyBallDetected() {
        
        boolean temp = false;

        if(isBlueBallDetected()) {
            temp = isBlueBallDetected();
        } else if(isRedBallDetected()) {
            temp = isRedBallDetected();
        } else {
            temp = false;
        }

        return temp;
    }

    public double getCrosshairX() {
        return getResX() / 2;
    }

    public double getCrosshairY() {
        return getResY() / 2;
    }

    public double getOffsetX(String ballType) {
        
        double offset = 0;
        
        if(ballType.equals("red")) {
            offset = getCrosshairX() - getRedBallCenterX();
        } else if(ballType.equals("blue")) {
            offset = getCrosshairX() - getBlueBallCenterX();
        }

        return -offset;
        
    }

    public double getOffsetY(String ballType) {
        
        double offset = 0;
        
        if(ballType.equals("red")) {
            offset = getCrosshairY() - getRedBallCenterY();
        } else if(ballType.equals("blue")) {
            offset = getCrosshairY() - getBlueBallCenterY();
        }

        return -offset;
        
    }

    public double piXPID(String ballType) {
		
        // double kP = 0.008;
        // double correctionMin = 0.003;
        // double deadzone = 0.05;
        // double correction = 0;
        
        // if((ballType.equals("red"))) {
    
        //     correction = getOffsetX("blue") * kP;

        //     if(correction < correctionMin)
        //         correction = Math.copySign(correctionMin, correction);

        //     if(Math.abs(getOffsetX("blue")) < deadzone)
        //         correction = 0;

        // } else if((ballType.equals("blue"))) {
        //     correction = getOffsetX("red") * kP;

        //     if(correction < correctionMin)
        //         correction = Math.copySign(correctionMin, correction);

        //     if(Math.abs(getOffsetX("red")) < deadzone)
        //         correction = 0;
        // } else {

        //     System.out.println("No ball detected");
        // }
		
		// return correction;

        double kP = 0.0005;
		double deadzone = 5;
		double correction = getOffsetX(ballType) * kP;

		if(Math.abs(getOffsetX(ballType)) < deadzone)
			correction = 0;

		return correction;
	}

	public double piYPID(String ballType) {
        
        double kP = 0.0005;
		double deadzone = 5;
		double correction = getOffsetY(ballType) * kP;

		if(Math.abs(getOffsetY(ballType)) < deadzone)
			correction = 0;

		return correction;
        
        // double kP = 0.008;
        // double correctionMin = 0.003;
        // double deadzone = 0.05;
        // double correction = 0;
        
        // if(isBlueBallDetected()) {
    
        //     correction = getOffsetY("blue") * kP;

        //     if(correction < correctionMin)
        //         correction = Math.copySign(correctionMin, correction);

        //     if(Math.abs(getOffsetY("blue")) < deadzone)
        //         correction = 0;

        // } else if(isRedBallDetected()) {
        //     correction = getOffsetY("red") * kP;

        //     if(correction < correctionMin)
        //         correction = Math.copySign(correctionMin, correction);

        //     if(Math.abs(getOffsetY("red")) < deadzone)
        //         correction = 0;
        // } else {

        //     System.out.println("No ball detected");
        // }
		
		// return correction;

	}
}

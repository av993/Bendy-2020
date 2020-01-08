package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous {

	ArrayList<Action> arr = new ArrayList<Action>();
	int taskNum = 0;
	
	public Autonomous() {
	}

	enum AutoStates {
		Default, RightPeg, LeftPeg, MidPeg, rBoiler, bBoiler, rHopper, rShootAlign, shoot, bHopper, bBoilerAutoline, rBoilerAutoline, rCenterShoot, bCenterShoot,scoreOpRight, scoreOpLeft, scoreOpRightCent, scoreOpLeftCent, bShootAlign;
	}

	public void run() {
		if (arr.size() >= taskNum) {
			arr.get(taskNum).run();
			if (arr.get(taskNum).isFinished()) {
				taskNum++;
			}
		}
	}

	public void assembleTest() {
		arr.add(new ActionTrajectory("Straight", 0, 0.01, false));
		arr.add(new ActionWait(9999999));
	}

}

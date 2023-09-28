package frc.lib.team5557.factory;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Timer;

public class BurnManager {
	private static boolean shouldBurn = true;

	public static void restoreFactoryDefaults(CANSparkMax sparkMax) {
		if (shouldBurn) {
			sparkMax.restoreFactoryDefaults();
			Timer.delay(0.2);
		}
	}

	public static void burnFlash(CANSparkMax sparkMax) {
		if (shouldBurn) {
			sparkMax.burnFlash();
			Timer.delay(0.2);
		}
	}

	public static boolean shouldBurn() {
		return shouldBurn;
	}
}

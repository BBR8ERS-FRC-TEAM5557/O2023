package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ObjectiveTracker {
    private static NodeLevel nodeLevel = NodeLevel.HYBRID;
    private static GamePiece gamePiece = GamePiece.CONE;


    public static GamePiece getGamePiece() {
        return gamePiece;
    }

    public static void setGamePiece(GamePiece piece) {
        gamePiece = piece;
    }

    public static NodeLevel getNodeLevel() {
        return nodeLevel;
    }

    /** Shifts the selected node in the selector by one position. */
    public static void shiftNode(Direction direction) {
        switch (direction) {
            case UP:
                switch (nodeLevel) {
                    case HYBRID:
                        break;
                    case MID:
                        nodeLevel = NodeLevel.HYBRID;
                        break;
                    case HIGH:
                        nodeLevel = NodeLevel.MID;
                        break;
                }
                break;

            case DOWN:
                switch (nodeLevel) {
                    case HYBRID:
                        nodeLevel = NodeLevel.MID;
                        break;
                    case MID:
                        nodeLevel = NodeLevel.HIGH;
                        break;
                    case HIGH:
                        break;
                }
                break;
        }
    }

    /** Command factory to shift the selected node in the selector by one position. */
    public static Command shiftNodeCommand(Direction direction) {
        return new InstantCommand(() -> shiftNode(direction)).ignoringDisable(true);
    }

    public static Command setNodeCommand(NodeLevel level) {
        return new InstantCommand(() -> nodeLevel = level);
    }

    public static enum NodeLevel {
        HYBRID, MID, HIGH
    }

    public static enum Direction {
        UP, DOWN
    }

    public static enum GamePiece {
        CUBE, CONE
    }
}

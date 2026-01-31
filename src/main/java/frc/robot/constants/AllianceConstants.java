// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Constants for alliance-specific targeting (Blue vs Red team fuel hubs).
 *
 * <h2>What is Alliance Targeting?</h2>
 * <p>In FRC competitions, you're either on the BLUE alliance or the RED alliance.
 * Each alliance has their own scoring targets (fuel hubs) on opposite sides of
 * the field. Your robot needs to know which team you're on so it can:
 * <ul>
 *   <li>Find the CORRECT fuel hub to aim at</li>
 *   <li>Ignore the opponent's fuel hub</li>
 *   <li>Use vision lock to automatically face YOUR target</li>
 * </ul>
 *
 * <h2>AprilTags on the Field</h2>
 * <p>Each fuel hub has AprilTags attached to it. These are special markers that
 * the Limelight camera can recognize:
 *
 * <pre>
 *   ┌──────────────────────────────────────────────────────────┐
 *   │                        FIELD                             │
 *   │                                                          │
 *   │   ┌──────────┐                        ┌──────────┐       │
 *   │   │ BLUE HUB │                        │ RED HUB  │       │
 *   │   │          │                        │          │       │
 *   │   │ Tag 1    │                        │ Tag 3    │       │
 *   │   │ Tag 2    │                        │ Tag 4    │       │
 *   │   └──────────┘                        └──────────┘       │
 *   │                                                          │
 *   │        ↑                                    ↑            │
 *   │   If you're on                        If you're on       │
 *   │   BLUE team,                          RED team,          │
 *   │   aim at these                        aim at these       │
 *   │                                                          │
 *   └──────────────────────────────────────────────────────────┘
 * </pre>
 *
 * <h2>Why Two Tags Per Hub?</h2>
 * <p>Having two AprilTags on each hub helps because:
 * <ul>
 *   <li>If one tag is blocked, the camera can see the other</li>
 *   <li>More tags = more accurate position estimation</li>
 *   <li>Tags can be placed at different angles for better visibility</li>
 * </ul>
 *
 * <h2>How to Configure Tag IDs</h2>
 * <p>The tag IDs below are EXAMPLES. You need to change them to match
 * the actual tags on YOUR field:
 *
 * <ol>
 *   <li>Look at the AprilTags on the blue fuel hub</li>
 *   <li>Note their ID numbers (printed on the tags)</li>
 *   <li>Update BLUE_HUB_TAG_IDS with those numbers</li>
 *   <li>Repeat for the red fuel hub</li>
 * </ol>
 *
 * @see frc.robot.subsystems.vision.AprilTagTracker for the targeting system
 */
public final class AllianceConstants {

  // ==================== Blue Alliance Tags ====================

  /**
   * AprilTag IDs on the BLUE alliance fuel hub.
   *
   * <p><b>CONFIGURE THIS!</b> Change these numbers to match the actual
   * AprilTag IDs on your field's blue fuel hub.
   *
   * <p>Example: If the blue hub has tags numbered 1 and 2, use {1, 2}
   */
  public static final int[] BLUE_HUB_TAG_IDS = {1, 2};

  /**
   * Primary tag to look for on the blue hub.
   *
   * <p>When multiple tags are available, the tracker will prefer this one.
   * This is the first tag in the BLUE_HUB_TAG_IDS array.
   */
  public static final int BLUE_HUB_PRIMARY_TAG = BLUE_HUB_TAG_IDS[0];

  // ==================== Red Alliance Tags ====================

  /**
   * AprilTag IDs on the RED alliance fuel hub.
   *
   * <p><b>CONFIGURE THIS!</b> Change these numbers to match the actual
   * AprilTag IDs on your field's red fuel hub.
   *
   * <p>Example: If the red hub has tags numbered 3 and 4, use {3, 4}
   */
  public static final int[] RED_HUB_TAG_IDS = {3, 4};

  /**
   * Primary tag to look for on the red hub.
   *
   * <p>When multiple tags are available, the tracker will prefer this one.
   * This is the first tag in the RED_HUB_TAG_IDS array.
   */
  public static final int RED_HUB_PRIMARY_TAG = RED_HUB_TAG_IDS[0];

  // ==================== Alliance Enum ====================

  /**
   * Enum representing which alliance the robot is on.
   *
   * <p>This is used throughout the code to determine which fuel hub to target.
   *
   * <h3>Usage Example:</h3>
   * <pre>
   * // In your code:
   * Alliance currentAlliance = Alliance.BLUE;
   *
   * // Get the tags for current alliance:
   * int[] tagsToTrack = currentAlliance.getHubTagIds();
   *
   * // Check which alliance:
   * if (currentAlliance == Alliance.RED) {
   *     System.out.println("Targeting red hub!");
   * }
   * </pre>
   */
  public enum Alliance {
    /**
     * Blue alliance - targets the blue fuel hub.
     */
    BLUE(BLUE_HUB_TAG_IDS, "Blue"),

    /**
     * Red alliance - targets the red fuel hub.
     */
    RED(RED_HUB_TAG_IDS, "Red");

    private final int[] hubTagIds;
    private final String displayName;

    Alliance(int[] hubTagIds, String displayName) {
      this.hubTagIds = hubTagIds;
      this.displayName = displayName;
    }

    /**
     * Gets the AprilTag IDs for this alliance's fuel hub.
     *
     * @return Array of tag IDs on this alliance's hub
     */
    public int[] getHubTagIds() {
      return hubTagIds;
    }

    /**
     * Gets the primary (preferred) tag ID for this alliance's hub.
     *
     * @return The primary tag ID
     */
    public int getPrimaryTagId() {
      return hubTagIds[0];
    }

    /**
     * Checks if a given tag ID belongs to this alliance's hub.
     *
     * @param tagId The tag ID to check
     * @return true if this tag is on this alliance's hub
     */
    public boolean isHubTag(int tagId) {
      for (int id : hubTagIds) {
        if (id == tagId) {
          return true;
        }
      }
      return false;
    }

    /**
     * Gets a human-readable name for this alliance.
     *
     * @return "Blue" or "Red"
     */
    public String getDisplayName() {
      return displayName;
    }

    /**
     * Gets the opposite alliance.
     *
     * <p>Useful for determining which tags to IGNORE (opponent's hub).
     *
     * @return The other alliance
     */
    public Alliance getOpposite() {
      return this == BLUE ? RED : BLUE;
    }
  }

  // ==================== Default Alliance ====================

  /**
   * Default alliance when the robot starts up.
   *
   * <p>The driver should select the correct alliance using the controller
   * buttons before the match starts. This default is just a fallback.
   *
   * <p>Change this to RED if your team usually practices as red alliance.
   */
  public static final Alliance DEFAULT_ALLIANCE = Alliance.BLUE;

  // ==================== Private Constructor ====================

  /**
   * Private constructor to prevent instantiation.
   */
  private AllianceConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

package com.conveyal.r5.streets;

import com.conveyal.r5.common.GeometryUtils;
import com.conveyal.r5.profile.ProfileRequest;
import com.conveyal.r5.profile.StreetMode;
import org.apache.commons.math3.util.FastMath;
import org.locationtech.jts.algorithm.Angle;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.LineString;

/**
 * This encapsulates the original/default turning and edge traversal cost calculations that were in the street router.
 * It uses hard-wired values for the amount of time it takes to complete a turn.
 */
public class BasicTraversalTimeCalculator implements TraversalTimeCalculator {

    public final StreetLayer layer;

    // Turn costs for various types of turns, in seconds.
    // These are all specified for drive-on-right countries, and reversed in drive-on-left countries.
    // TODO re-express as "turn against traffic" and "turn with traffic" to make this less confusing.

    public static final int LEFT_TURN = 30;
    public static final int STRAIGHT_ON = 0;
    public static final int RIGHT_TURN = 10;
    public static final int U_TURN = 90; // penalize U turns extremely heavily

    public boolean driveOnRight; // TODO instead of a field, this should be a different implementation class

    // ====== KOTI signal-delay extension ======================================
    // Extra wait (seconds) added when a CAR turn ENTERS an intersection that has
    // a traffic signal (VertexFlag.TRAFFIC_SIGNAL). Added ON TOP of the turn-angle
    // cost above. Differentiated by movement (through / left / right) and by time
    // band (peak / offpeak / night) selected from the route's departure time.
    // Values are starting points (KOTI), tunable. Right turns ~ free in Korea.
    // Index: [band][movement]  band 0=peak 1=offpeak 2=night; movement 0=through 1=left 2=right
    public static int[][] SIGNAL_DELAY = {
        // through, left, right
        { 14, 12, 2 },  // peak    (07-09, 17-19)
        {  8,  7, 0 },  // offpeak (other daytime)
        {  3,  3, 0 }   // night   (22-06)
    };
    // Master switch: signal delay only applied when true (set by R5RCore/r5r).
    public static boolean SIGNAL_DELAY_ENABLED = false;

    // Departure seconds-from-midnight for the current route; set by StreetRouter.route().
    // -1 means "unknown" → treated as offpeak.
    public int fromTimeSeconds = -1;

    private int timeBand() {
        if (fromTimeSeconds < 0) return 1; // unknown → offpeak
        int h = (fromTimeSeconds / 3600) % 24;
        boolean peak  = (h >= 7 && h < 9) || (h >= 17 && h < 19);
        boolean night = (h >= 22) || (h < 6);
        return peak ? 0 : (night ? 2 : 1);
    }
    // =========================================================================

    public BasicTraversalTimeCalculator (StreetLayer layer, boolean driveOnRight) {
        this.layer = layer;
        this.driveOnRight = driveOnRight;
    }

    @Override
    public int traversalTimeSeconds (EdgeStore.Edge currentEdge, StreetMode streetMode, ProfileRequest req) {
        float speedMetersPerSecond = currentEdge.calculateSpeed(req, streetMode);
        double traversalTimeSeconds = currentEdge.getLengthM() / speedMetersPerSecond;
        return (int) FastMath.ceil(traversalTimeSeconds);
    }

    /**
     * Quantify the difficulty of turning from the fromEdge onto the toEdge with the given streetMode of transport.
     * Quantities should be time-like with units of seconds or perceived seconds.
     * TODO pull this out into an interface to allow generalization to data from generalized cost tags
     */
    @Override
    public int turnTimeSeconds (int fromEdge, int toEdge, StreetMode streetMode) {
        if (streetMode == StreetMode.CAR) {
            double angle = calculateNewTurnAngle(fromEdge, toEdge);
            int turnCost;
            int movement; // 0=through, 1=left, 2=right (for signal-delay lookup)
            if (angle < 27) {
                turnCost = STRAIGHT_ON; movement = 0;
            } else if (angle < 153) {
                turnCost = driveOnRight ? LEFT_TURN : RIGHT_TURN;
                movement = driveOnRight ? 1 : 2;
            } else if (angle < 207) {
                turnCost = U_TURN; movement = 1; // treat U-turn like left for signal band
            } else if (angle < 333) {
                turnCost = driveOnRight ? RIGHT_TURN : LEFT_TURN;
                movement = driveOnRight ? 2 : 1;
            } else {
                turnCost = STRAIGHT_ON; movement = 0;
            }

            // KOTI signal delay: if the turn enters a signalized intersection, add
            // a movement- and time-band-specific wait on top of the turn cost.
            if (SIGNAL_DELAY_ENABLED) {
                EdgeStore.Edge e = layer.edgeStore.getCursor(fromEdge);
                int enteredVertex = e.getToVertex();
                if (layer.vertexStore.getCursor(enteredVertex)
                        .getFlag(VertexStore.VertexFlag.TRAFFIC_SIGNAL)) {
                    turnCost += SIGNAL_DELAY[timeBand()][movement];
                }
            }
            return turnCost;
        }
        return 0;
    }

    /**
     * Gets in/out angles from edges and calculates angle between them
     * @return angle in degrees from 0-360
     */
    private double calculateNewTurnAngle(int fromEdge, int toEdge) {
        EdgeStore.Edge e = layer.edgeStore.getCursor(fromEdge);
        int outAngle = e.getOutAngle();
        e = layer.edgeStore.getCursor(toEdge);
        int inAngle = e.getInAngle();
        return calculateTurnAngle(outAngle, inAngle);
    }

    /**
     *
     * @param angleIntoIntersectionDeg from edge Out angle in degrees
     * @param angleOutOfIntersectionDeg to edge In angle in degrees
     * @return angle in degrees between 0 and 360
     */
    private int calculateTurnAngle(int angleIntoIntersectionDeg, int angleOutOfIntersectionDeg) {

        if (angleIntoIntersectionDeg < angleOutOfIntersectionDeg) {
            angleIntoIntersectionDeg += 360;
        }

        int turnAngle = angleIntoIntersectionDeg - angleOutOfIntersectionDeg;
        if (!driveOnRight) {
            turnAngle = 360 - turnAngle;
        }

        return turnAngle;

    }

    /** Compute the angle between two edges, positive, from 0 to 2 * Pi. The angle is _counterclockwise_ because that's how it's done in JTS. */
    public double computeAngle (int fromEdge, int toEdge) {
        // figure out turn angle
        EdgeStore.Edge e = layer.edgeStore.getCursor(fromEdge);
        VertexStore.Vertex v = layer.vertexStore.getCursor(e.getToVertex());

        Coordinate p1 = pointOnLine(e.getGeometry(), true);
        Coordinate p2 = new Coordinate(v.getLon(), v.getLat());
        e.seek(toEdge);
        Coordinate p3 = pointOnLine(e.getGeometry(), false);

        // project to local coordinate system, but don't reverse axes
        double cosLat = Math.abs(Math.cos(p2.y * Math.PI / 180));
        p1.x /= cosLat;
        p2.x /= cosLat;
        p3.x /= cosLat;

        // figure out the angle. All below computations in radians. Add 2 * pi so everything is positive.
        double angleIn = Angle.angle(p1, p2);
        double angleOut = Angle.angle(p2, p3);

        double angle = angleOut - angleIn;

        // make sure it's positive
        if (angle < 0) angle += Math.PI * 2;

        return angle;
    }

    public static Coordinate pointOnLine(LineString line, boolean end) {
        Coordinate[] coords = end ? line.reverse().getCoordinates() : line.getCoordinates();

        double distanceSoFar = 0;
        int coordIndex = 1;

        while (distanceSoFar < 10 && coordIndex < coords.length) {
            distanceSoFar += GeometryUtils.distance(coords[coordIndex - 1].x, coords[coordIndex - 1].y, coords[coordIndex].x, coords[coordIndex].y);
            coordIndex++;
        }

        return coords[coordIndex - 1];
    }
}

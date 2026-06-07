/**
 * @file OptimizationHints.h
 * @brief Tuning guidelines and optimization strategies for GyroBot firmware
 *
 * This file documents recommended parameter optimization techniques
 * based on the tuning_guide.md and phase-by-phase tuning process.
 *
 * Reference: Documents/tuning_guide.md
 */

#ifndef OPTIMIZATION_HINTS_H
#define OPTIMIZATION_HINTS_H

/**
 * RECOMMENDED TUNING PARAMETERS
 * ═════════════════════════════════════════════════════════════════
 *
 * After complete tuning (Phase 1-4), expect these values:
 *
 * IK Algorithm (Documents/simulateur.py):
 * ┌──────────────────────┬──────────┬─────────────────────────────┐
 * │ Parameter            │ Range    │ Recommended (Start)          │
 * ├──────────────────────┼──────────┼─────────────────────────────┤
 * │ DRAW_SPEED           │ 0.5-3.0  │ 2.0 cm/s (adjust ±0.5)     │
 * │ APPROACH_GAIN        │ 0.5-3.0  │ 1.5 (adjust ±0.3)          │
 * │ WAYPOINT_TOL         │ 0.1-0.3  │ 0.2 cm (0.1-0.25 typical)  │
 * └──────────────────────┴──────────┴─────────────────────────────┘
 *
 * Motor Control (include/Robot.h, src/MotorUtility.h):
 * ┌──────────────────────┬──────────┬─────────────────────────────┐
 * │ Parameter            │ Range    │ Recommended                  │
 * ├──────────────────────┼──────────┼─────────────────────────────┤
 * │ PWM_BASE             │ 30-150   │ 80 (nominal), adjust ±20   │
 * │ K (straight > 5cm)   │ 0.5-3.0  │ 1.0-1.5                    │
 * │ K (curve 2-5cm)      │ 1.0-5.0  │ 2.0-3.0                    │
 * │ K (sharp < 2cm)      │ 2.0-8.0  │ 4.0-5.0                    │
 * └──────────────────────┴──────────┴─────────────────────────────┘
 *
 * ═════════════════════════════════════════════════════════════════
 */

/**
 * DYNAMIC K GAIN SELECTION (Implementation Guide)
 * ═════════════════════════════════════════════════════════════════
 *
 * Current implementation in bouger_ticks_en_1s() uses a simple binary K:
 *
 *   float K = (distance > 5.0) ? 1.5 : 4.0;
 *
 * OPTIMIZATION: Use piecewise K selection based on segment distance
 *
 * Step 1: Modify IKMove struct to include segment metadata
 * ─────────────────────────────────────────────────────────────────
 *
 *   struct IKMove {
 *       int   tg, td;           // ticks targets
 *       int   vg, vd;           // velocities
 *       float dg, dd;           // delays
 *       float distance;         // ADDED: segment distance (cm)
 *       int   segment_type;     // ADDED: 0=straight, 1=curve, 2=sharp
 *   };
 *
 * Step 2: Compute distance in GCodeParser::computeSegment()
 * ─────────────────────────────────────────────────────────────────
 *
 *   void GCodeParser::computeSegment(const State& from, float wx, float wy,
 *                                     std::vector<IKMove>& out) {
 *       // ... existing code ...
 *
 *       // Compute initial distance to waypoint
 *       float px = from.rx + ROBOT_STYLO_OFFSET * cosf(from.theta);
 *       float py = from.ry + ROBOT_STYLO_OFFSET * sinf(from.theta);
 *       float segment_dist = sqrtf((wx - px)*(wx - px) + (wy - py)*(wy - py));
 *
 *       // After creating IKMove m in flush:
 *       m.distance = segment_dist;
 *       m.segment_type = (segment_dist > 10.0) ? 0 :      // straight
 *                        (segment_dist > 2.0) ? 1 :        // curve
 *                        2;                                 // sharp
 *
 *       out.push_back(m);
 *   }
 *
 * Step 3: Use segment metadata in bouger_ticks_en_1s()
 * ─────────────────────────────────────────────────────────────────
 *
 *   void bouger_ticks_en_1s(int tg, int td, int vg, int vd, float dg, float dd,
 *                           int segment_type = 1) {
 *       // Select K based on segment type
 *       float K;
 *       switch(segment_type) {
 *           case 0:  // straight (distance >= 10cm)
 *               K = 1.0f;
 *               break;
 *           case 1:  // curve (2cm <= distance < 10cm)
 *               K = 2.5f;
 *               break;
 *           case 2:  // sharp (distance < 2cm)
 *               K = 4.0f;
 *               break;
 *           default:
 *               K = 1.5f;  // fallback
 *       }
 *       // ... rest of function, use K for servo gain ...
 *   }
 *
 * Step 4: Update CommandDispatcher to handle segment_type
 * ─────────────────────────────────────────────────────────────────
 *
 *   if (upperCmd.startsWith("IK_MOVE:")) {
 *       IKMove m = IKMove::fromParams(upperCmd.substring(8));
 *       int seg_type = 1;  // default: curve
 *       // Could extend protocol to include segment_type in future
 *       bouger_ticks_en_1s(m.tg, m.td, m.vg, m.vd, m.dg, m.dd, seg_type);
 *       return;
 *   }
 *
 * ═════════════════════════════════════════════════════════════════
 */

/**
 * TUNING DECISION MATRIX
 * ═════════════════════════════════════════════════════════════════
 *
 * Use this to diagnose tuning issues and select corrective actions:
 *
 * SYMPTOM: Oscillation on curves
 * ────────────────────────────────
 *   Cause: K too high, aggressive servo
 *   Fixes: 1) Reduce all K values by 0.5-1.0
 *          2) Lower APPROACH_GAIN by 0.2-0.3
 *          3) Increase WAYPOINT_TOL by 0.05 cm
 *
 * SYMPTOM: Lag (pen behind target)
 * ────────────────────────────────
 *   Cause: Speed too slow, K too low, poor response
 *   Fixes: 1) Increase DRAW_SPEED by 0.5 cm/s
 *          2) Increase APPROACH_GAIN by 0.3-0.5
 *          3) Increase K values by 1.0 (but watch for oscillation)
 *
 * SYMPTOM: Jerky motion / stutter
 * ────────────────────────────────
 *   Cause: Acceleration changes abruptly, motor saturation
 *   Fixes: 1) Decrease DRAW_SPEED by 0.5 cm/s
 *          2) Increase PWM_BASE by 10-20 (more power reduces jitter)
 *          3) Lower APPROACH_GAIN slightly (0.2 decrement)
 *
 * SYMPTOM: Motor stalling / not moving
 * ────────────────────────────────────
 *   Cause: Insufficient motor power, high friction
 *   Fixes: 1) Increase PWM_BASE by 20-30
 *          2) Decrease WAYPOINT_TOL by 0.05 cm (less demanding)
 *          3) Lower APPROACH_GAIN (less aggressive target chasing)
 *
 * SYMPTOM: Overshooting waypoints (visible loops)
 * ────────────────────────────────────────────────
 *   Cause: K too high, APPROACH_GAIN too high
 *   Fixes: 1) Reduce all K values by 0.5-1.0
 *          2) Reduce APPROACH_GAIN by 0.3-0.5
 *          3) Increase WAYPOINT_TOL by 0.1 cm (more lenient)
 *
 * ═════════════════════════════════════════════════════════════════
 */

/**
 * PERFORMANCE TARGETS (After tuning)
 * ═════════════════════════════════════════════════════════════════
 *
 * ✓ ACCURACY:
 *   - RMS position error: 3-5 mm (excellent)
 *   - Maximum overshoot: < 2 mm
 *   - Waypoint convergence: 0.8-1.2 seconds
 *
 * ✓ SMOOTHNESS:
 *   - No visible oscillation on curves
 *   - No jerky acceleration changes
 *   - Motor power smooth and continuous
 *
 * ✓ SPEED:
 *   - Total drawing time: < 2 minutes (full page)
 *   - Per waypoint: 0.8-1.5 seconds convergence
 *   - No stalling on any segment
 *
 * ✓ POWER:
 *   - Average motor current: 100-200 mA
 *   - Peak current: < 300 mA (avoid thermal stress)
 *   - Battery voltage stable (> 10.5V nominal)
 *
 * ═════════════════════════════════════════════════════════════════
 */

/**
 * TESTING PROCEDURE (Quick Verification)
 * ═════════════════════════════════════════════════════════════════
 *
 * After tuning, run this quick test to verify parameters:
 *
 * 1. Load staircase_5steps.gcode
 *    Expected: ~15-20 seconds total, smooth 90° corners
 *
 * 2. Load spiral_square.gcode
 *    Expected: Nested squares align well, < 2mm drift per layer
 *
 * 3. Load circle.gcode
 *    Expected: Smooth curve, no faceting or jitter
 *
 * 4. Observe motor current (multimeter or estimation)
 *    Expected: 100-200 mA average, < 300 mA peak
 *
 * 5. Check telemetry with analyze_telemetry.py
 *    Expected: Score >= 8.0 / 10
 *
 * ═════════════════════════════════════════════════════════════════
 */

#endif // OPTIMIZATION_HINTS_H

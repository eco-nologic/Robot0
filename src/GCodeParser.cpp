#include "GCodeParser.h"
#include <math.h>

const float GCodeParser::DT            = 0.020f;
const float GCodeParser::DRAW_SPEED    = 2.0f;
const float GCodeParser::APPROACH_GAIN = 1.5f;
const float GCodeParser::WAYPOINT_TOL  = 0.2f;
const int   GCodeParser::DEFAULT_PWM   = 80;

float GCodeParser::State::penX() const {
    return rx + ROBOT_STYLO_OFFSET * cosf(theta);
}

float GCodeParser::State::penY() const {
    return ry + ROBOT_STYLO_OFFSET * sinf(theta);
}

void GCodeParser::computeSegment(const State& from, float wx, float wy,
                                  std::vector<IKMove>& out) {
    float rx = from.rx, ry = from.ry, theta = from.theta;

    float ticksAccumL = 0.0f, ticksAccumR = 0.0f;
    float speedAccumL = 0.0f, speedAccumR = 0.0f;
    int   speedSteps = 0;
    float timeSince = 0.0f;

    // Lambda : flush accumulator → IKMove
    auto flush = [&]() {
        if (speedSteps == 0) return;
        IKMove m;
        m.tg = (int)roundf(ticksAccumL);
        m.td = (int)roundf(ticksAccumR);
        m.vg = (int)roundf(speedAccumL / speedSteps);
        m.vd = (int)roundf(speedAccumR / speedSteps);
        m.dg = 0.0f;
        m.dd = 0.0f;
        out.push_back(m);
        ticksAccumL = ticksAccumR = speedAccumL = speedAccumR = 0.0f;
        speedSteps = 0;
        timeSince = 0.0f;
    };

    // IK resolved-rate loop — Shih & Lin eq. 8 (Tc=20ms)
    while (true) {
        float px = rx + ROBOT_STYLO_OFFSET * cosf(theta);
        float py = ry + ROBOT_STYLO_OFFSET * sinf(theta);
        float ex = wx - px, ey = wy - py;
        float dist = sqrtf(ex*ex + ey*ey);

        if (dist < WAYPOINT_TOL) break;

        // Proportional approach velocity (eq. 8 term 1)
        float speed = fmaxf(0.1f, fminf(DRAW_SPEED, dist * APPROACH_GAIN));
        float vpx = (ex / dist) * speed;
        float vpy = (ey / dist) * speed;

        // Inverse Jacobian (eq. 7, Shih & Lin 2017)
        // [ẋ, ẏ] = J⁻¹(θ) × [v, ω]
        // But we solve backwards: given [vpx, vpy], compute [v, ω]
        float v     =  vpx * cosf(theta) + vpy * sinf(theta);
        float omega = (-vpx * sinf(theta) + vpy * cosf(theta)) / ROBOT_STYLO_OFFSET;

        // Wheel velocities from body velocities
        float vL = v - omega * ROBOT_ENTRAXE / 2.0f;
        float vR = v + omega * ROBOT_ENTRAXE / 2.0f;

        // Odometry integration over one DT step
        float dl = vL * DT;
        float dr = vR * DT;
        theta += (dr - dl) / ROBOT_ENTRAXE;
        float ds = (dl + dr) / 2.0f;
        rx += ds * cosf(theta);
        ry += ds * sinf(theta);

        // Accumulate ticks and speeds for this step
        ticksAccumL += (vL * DT / ROBOT_ROUE_PERIMETRE) * ROBOT_TICKS_PAR_ROTATION;
        ticksAccumR += (vR * DT / ROBOT_ROUE_PERIMETRE) * ROBOT_TICKS_PAR_ROTATION;
        speedAccumL += fabsf(vL);
        speedAccumR += fabsf(vR);
        speedSteps++;
        timeSince += DT;

        // Flush to IKMove when accumulated 1 second
        if (timeSince >= 1.0f - 1e-6f) {
            flush();
        }
    }

    // Flush any remaining partial segment
    flush();
}

std::vector<IKMove> GCodeParser::parse(const String& gcode,
                                        float startX, float startY, float startTheta) {
    std::vector<IKMove> result;

    // Initialize robot state: center of wheels at (startX - D·cosθ, startY - D·sinθ)
    State state;
    state.rx = startX - ROBOT_STYLO_OFFSET * cosf(startTheta);
    state.ry = startY - ROBOT_STYLO_OFFSET * sinf(startTheta);
    state.theta = startTheta;

    // Parse GCode line by line
    int lineStart = 0;
    while (lineStart < (int)gcode.length()) {
        int lineEnd = gcode.indexOf('\n', lineStart);
        if (lineEnd < 0) lineEnd = gcode.length();

        String line = gcode.substring(lineStart, lineEnd);
        line.trim();
        lineStart = lineEnd + 1;

        // Skip comments and empty lines
        if (line.startsWith(";") || line.length() == 0) continue;

        // Parse only G0/G1 commands
        if (!line.startsWith("G0") && !line.startsWith("G1")) continue;

        // Extract X and Y parameters
        float wx = state.penX(), wy = state.penY();  // default: stay in place
        int xi = line.indexOf('X');
        int yi = line.indexOf('Y');
        if (xi >= 0) wx = line.substring(xi + 1).toFloat();
        if (yi >= 0) wy = line.substring(yi + 1).toFloat();

        // Compute segment from current to waypoint
        size_t before = result.size();
        computeSegment(state, wx, wy, result);

        // Update state: integrate odometry through newly generated segments
        for (size_t i = before; i < result.size(); i++) {
            const IKMove& m = result[i];
            float dL = ((float)m.tg / ROBOT_TICKS_PAR_ROTATION) * ROBOT_ROUE_PERIMETRE;
            float dR = ((float)m.td / ROBOT_TICKS_PAR_ROTATION) * ROBOT_ROUE_PERIMETRE;
            float ds = (dL + dR) / 2.0f;
            state.theta += (dR - dL) / ROBOT_ENTRAXE;
            state.rx += ds * cosf(state.theta);
            state.ry += ds * sinf(state.theta);
        }
    }

    return result;
}

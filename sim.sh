#!/usr/bin/env bash
# Team 9520 autonomy simulator — starts the robot code in simulation AND the web UI.
#
#   ./sim.sh              robot sim with the WPILib sim GUI + web UI
#   ./sim.sh --headless   robot sim without the WPILib GUI window + web UI
#
# Then open http://localhost:5173. Ctrl+C stops both.
# Needs: WPILib 2026 (provides Java 17) and Node.js 20+.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
ROBOT_DIR="$ROOT/Swerve9520 ctre tuner x project (og home base ver.)"
UI_DIR="$ROOT/sim-ui"

GRADLE_ARGS=()
[[ "${1:-}" == "--headless" ]] && GRADLE_ARGS+=(-Pheadless)

# Java 17: prefer the JDK that ships with a WPILib installer (newest year first).
# Any WPILib year's JDK 17 works; the 2026 libraries are downloaded by Gradle.
if [[ -z "${JAVA_HOME:-}" ]]; then
  for d in $(ls -d "$HOME"/wpilib/*/jdk /Users/Shared/wpilib/*/jdk /c/Users/Public/wpilib/*/jdk 2>/dev/null | sort -r); do
    [[ -x "$d/bin/java" ]] && export JAVA_HOME="$d" && break
  done
fi
# (macOS has a /usr/bin/java stub even without Java, so actually run it)
if [[ -n "${JAVA_HOME:-}" ]]; then JAVA_BIN="$JAVA_HOME/bin/java"; else JAVA_BIN=java; fi
if ! "$JAVA_BIN" -version >/dev/null 2>&1; then
  echo "Java 17 not found. Install WPILib 2026 (https://docs.wpilib.org) or set JAVA_HOME." >&2
  exit 1
fi
echo "Using Java: ${JAVA_HOME:-$(command -v java)}"
command -v npm >/dev/null || { echo "Node.js not found. Install Node.js 20 LTS or newer." >&2; exit 1; }

cleanup() {
  echo; echo "Stopping simulator…"
  kill 0 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ ! -d "$UI_DIR/node_modules" ]]; then
  echo "Installing web UI dependencies (first run only)…"
  (cd "$UI_DIR" && npm install --no-audit --no-fund)
fi

echo "Starting robot code in simulation…"
# If the robot sim stops (build error, crash), stop everything so it's noticed.
(cd "$ROBOT_DIR" && ./gradlew simulateJava ${GRADLE_ARGS[@]+"${GRADLE_ARGS[@]}"};
 echo; echo "Robot sim exited — see the messages above. Stopping."; kill $$) &

echo "Starting web UI on http://localhost:5173 …"
(cd "$UI_DIR" && npm run dev) &

wait

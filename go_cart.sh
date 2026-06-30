#!/usr/bin/env bash
# ===========================================================================
#  ONE COMMAND — bring up the whole K1 cart-seek on the REAL robot.
#  head camera -> YOLO-World cart detector -> REACTIVE visual-servo -> SDK-Move bridge.
#  (No Nav2/SLAM: cart_approach turns the body to the cart + walks to the standoff.)
#
#  *** SAFE BY DESIGN ***  This brings everything up but the robot comes up
#  IDLE and DOES NOT MOVE. Nothing walks until you explicitly say so.
#
#    bash go_cart.sh     # <- this: bring it all up (robot stays put)
#    bash walk.sh        # start walking to the cart (hand on the strap!)
#    bash estop.sh       # EMERGENCY STOP -- halts, robot STAYS STANDING
#
#  Speeds are hard-capped in the bridge (vx<=0.20 m/s default, <=0.80 absolute).
# ===========================================================================
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
CAM=/home/boosterk1/robots/k1/workspace/k1-vlm-navigation/bringup_camera.sh

echo "[go] stopping any previous run (robust cleanup -- kills orphaned nav/TF zombies) ..."
bash "$HERE/nav_cleanup.sh"

echo
echo "================ STEP 1/2 — head camera (the cart detector's eyes) ================"
bash "$CAM" || echo "[go] WARNING: camera not confirmed live -- the detector will idle until it is."

echo
echo "================ STEP 2/2 — cart-seek stack (REAL robot, comes up IDLE) ============"
echo "  The robot will NOT move on launch. When you're ready, in ANOTHER terminal:"
echo "     bash $HERE/walk.sh      # start walking to the cart"
echo "     bash $HERE/estop.sh     # EMERGENCY stop (stays standing, never limp)"
echo "==================================================================================="
echo
exec bash "$HERE/run_seek_cart_reactive.sh" net:=eno1     # gated: IDLE until /start_walking

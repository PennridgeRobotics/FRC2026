import launchpad_py as launchpad
import time
from networktables import NetworkTables # Using static NT3 methods
import sys
import traceback
import math # For ceiling function

# --- Configuration ---
LAUNCHPAD_MIDI_NAME = 'Launchpad MK2'
ROBORIO_IP = '10.105.84.2'

# NetworkTables Configuration
NT_LAUNCHPAD_TABLE = 'Launchpad'
NT_PRESET_COMMAND_KEY = 'PresetCommand'
NT_ROBOT_DATA_TABLE = 'RobotData' # Table where robot sends data
NT_ELEVATOR_POS_KEY = 'ElevatorPosition' # Key for elevator position

# Preset Pad Mapping (X, Y coordinates -> Preset Number)
XY_TO_PRESET_MAP = {
    (0, 1): 1,  # Pad -> Preset 1
    (1, 1): 2,  # Pad -> Preset 2
    (2, 1): 3,  # Pad -> Preset 3
    (3, 1): 4,  # Pad -> Preset 4
    # ADD MORE
}

# --- Lighting Configuration ---
PAD_COLOR_OFF = 0   # Black/Off
PAD_COLOR_IDLE = 5  # Dim Red (For preset pads)
PAD_COLOR_PRESSED = 3 # Full Green (For preset pads)
ELEVATOR_BAR_COLOR = 21 # Bright Green (Example)
ELEVATOR_BAR_LENGTH = 4 # Number of LEDs in the bar (X=0 to 3)
# --- End Configuration ---

# --- Elevator Visualization Config ---
ELEVATOR_MAX_POS = 0.0     # Position corresponding to the bottom row
ELEVATOR_MIN_POS = 0.95   # Position corresponding to the top row (negative is up)
ELEVATOR_VIS_BOTTOM_ROW = 8 # Y-coordinate for the bottom of the visualization
ELEVATOR_VIS_TOP_ROW = 1    # Y-coordinate for the top of the visualization
# --- End Elevator Visualization Config ---


# --- Global Variables ---
lp = None
# NetworkTables
nt_table_launchpad = None
preset_entry = None
nt_table_robotdata = None
elevator_pos_entry = None
networktables_connected = False
networktables_initialized_attempted = False
# Visualization State
current_elevator_pos = 0.0
current_bar_row = -1 # Start invalid to force initial draw
last_nt_read_time = 0.0
nt_read_interval = 0.1 # Read NT every 100ms
# ---

def cleanup():
    global networktables_connected, lp, networktables_initialized_attempted
    print("\n--- Cleaning Up ---")
    if lp:
        try:
            print("Turning off Launchpad LEDs...")
            lp.Reset()
            print("Closing Launchpad connection...")
            lp.Close()
            lp = None
        except Exception as e:
            print(f"Error closing Launchpad: {e}")
    if networktables_initialized_attempted:
        try:
            # No need to set preset_entry here, just shut down
            print("Shutting down NetworkTables...")
            NetworkTables.shutdown()
        except Exception as e:
             print(f"Error during NetworkTables cleanup: {e}")
    else:
        print("NetworkTables initialization was not attempted or failed early.")
    networktables_connected = False
    networktables_initialized_attempted = False
    print("Cleanup complete. Bridge stopped.")

def initialize_networktables():
    """Initializes and tries to connect to NetworkTables. Gets table entries."""
    global nt_table_launchpad, preset_entry, nt_table_robotdata, elevator_pos_entry
    global networktables_connected, networktables_initialized_attempted

    print(f"\n--- Initializing NetworkTables (Attempting NT3 connection to {ROBORIO_IP}) ---")
    networktables_initialized_attempted = True
    try:
        NetworkTables.initialize(server=ROBORIO_IP)

        # Get tables
        nt_table_launchpad = NetworkTables.getTable(NT_LAUNCHPAD_TABLE)
        nt_table_robotdata = NetworkTables.getTable(NT_ROBOT_DATA_TABLE)

        # Get entries
        preset_entry = nt_table_launchpad.getEntry(NT_PRESET_COMMAND_KEY)
        elevator_pos_entry = nt_table_robotdata.getEntry(NT_ELEVATOR_POS_KEY)

        connect_timeout_seconds = 7.0
        start_time = time.time()
        while not NetworkTables.isConnected():
            if time.time() - start_time > connect_timeout_seconds:
                print(f"WARN: Connection to {ROBORIO_IP} timed out.")
                print("-----------------------------------------------------")
                print("WARN: Running in Input Verification Mode ONLY.")
                print("      Elevator visualization will not update.")
                print("-----------------------------------------------------")
                networktables_connected = False
                return False
            time.sleep(0.2)

        print(f"SUCCESS: NetworkTables Connected!")
        # Initialize command entry on robot side, Python just sets it when needed
        # preset_entry.setDouble(0.0) # Robot initializes this now
        networktables_connected = True
        return True

    except Exception as e:
        print(f"ERROR during NetworkTables initialization/connection: {e}")
        traceback.print_exc()
        networktables_connected = False
        return False

def set_initial_pad_colors():
    """Sets the initial colors for the mapped preset pads."""
    if not lp: return
    print("Setting initial pad colors...")
    try:
        # Don't Reset here, allow initial elevator bar to draw first
        # lp.Reset()
        # time.sleep(0.05)
        for (x,y) in XY_TO_PRESET_MAP.keys():
             # Avoid overwriting elevator bar area if they overlap initially
             is_elevator_area = (x < ELEVATOR_BAR_LENGTH and y >= ELEVATOR_VIS_TOP_ROW and y <= ELEVATOR_VIS_BOTTOM_ROW)
             if not is_elevator_area:
                 lp.LedCtrlXYByCode(x, y, PAD_COLOR_IDLE)
        print("Initial preset pad colors set (excluding initial bar area).")
    except Exception as e:
        print(f"Error setting initial pad colors: {e}")

def map_position_to_row(position):
    """Maps elevator position to Launchpad row (8 to 1)."""
    pos_range = ELEVATOR_MAX_POS - ELEVATOR_MIN_POS
    vis_range = ELEVATOR_VIS_BOTTOM_ROW - ELEVATOR_VIS_TOP_ROW # Should be 7

    if pos_range <= 1e-6: return ELEVATOR_VIS_BOTTOM_ROW # Avoid division by zero

    # Normalize position (0 at bottom, 1 at top)
    normalized_pos = (position - ELEVATOR_MAX_POS) / (ELEVATOR_MIN_POS - ELEVATOR_MAX_POS)
    normalized_pos = max(0.0, min(1.0, normalized_pos)) # Clamp to 0-1

    # Map to row index (0 to vis_range) and invert for Y-coordinate
    # Using ceiling ensures that even slightly below 0 maps to row 8.
    # Map 0 -> 0, slightly > 0 -> 1, ..., slightly > 6 -> 7
    row_step = math.ceil(normalized_pos * vis_range)

    # Convert step (0=bottom, 7=top) to Y coordinate (8=bottom, 1=top)
    target_row = ELEVATOR_VIS_BOTTOM_ROW - row_step

    # Clamp to valid row range
    target_row = max(ELEVATOR_VIS_TOP_ROW, min(ELEVATOR_VIS_BOTTOM_ROW, target_row))

    return int(target_row)

def update_elevator_leds(target_row):
    """Updates the elevator visualization bar on the Launchpad."""
    global current_bar_row
    if not lp or target_row == current_bar_row:
        return # No change needed or no launchpad

    try:
        # Turn off LEDs on the old row (if valid)
        if current_bar_row >= ELEVATOR_VIS_TOP_ROW and current_bar_row <= ELEVATOR_VIS_BOTTOM_ROW:
            for x in range(ELEVATOR_BAR_LENGTH):
                # Decide what color to turn off to: OFF or IDLE if it's a preset pad?
                # Check if this pad is also a preset pad before turning it fully off
                 coord = (x, current_bar_row)
                 idle_color = PAD_COLOR_IDLE if coord in XY_TO_PRESET_MAP else PAD_COLOR_OFF
                 lp.LedCtrlXYByCode(x, current_bar_row, idle_color)


        # Turn on LEDs on the new row
        if target_row >= ELEVATOR_VIS_TOP_ROW and target_row <= ELEVATOR_VIS_BOTTOM_ROW:
            for x in range(ELEVATOR_BAR_LENGTH):
                lp.LedCtrlXYByCode(x, target_row, ELEVATOR_BAR_COLOR)

            # Update the current row state
            current_bar_row = target_row
            # print(f"Debug: Moved bar to row {current_bar_row}") # Optional debug

    except Exception as e:
        print(f"Error updating elevator LEDs: {e}")
        # Consider resetting current_bar_row?
        current_bar_row = -1 # Force redraw next time


def launchpad_loop():
    """Main loop: process button presses, update LEDs, send commands."""
    global lp, networktables_connected, current_elevator_pos, last_nt_read_time
    print("\n--- Starting Launchpad Listener Loop ---")
    if not networktables_connected:
        print("!!! RUNNING IN INPUT VERIFICATION MODE - NOT CONNECTED TO ROBOT !!!")
    # print(f"Mapped X/Y for commands: {list(XY_TO_PRESET_MAP.keys())}")
    print("Press mapped pads to send presets. Watch elevator viz. Ctrl+C to exit.")
    print("-" * 30)

    last_activity_time = time.time()
    check_interval = 10.0

    # Set initial colors (preset pads first, then elevator bar)
    set_initial_pad_colors()
    # Draw initial elevator bar based on assumed starting position (0)
    initial_row = map_position_to_row(0.0)
    update_elevator_leds(initial_row)


    while True:
        try:
            current_time = time.time()

            # --- NetworkTables Update ---
            # Check connection status periodically
            if not networktables_connected and networktables_initialized_attempted and current_time % 15 < 0.1:
                if NetworkTables.isConnected():
                    print("\n*** NetworkTables Connection RE-ESTABLISHED! ***")
                    networktables_connected = True
                # else: # Still disconnected - handled by networktables_connected flag

            # Read elevator position periodically
            if networktables_connected and (current_time - last_nt_read_time > nt_read_interval):
                try:
                    # Use getDouble with default 0.0 if entry doesn't exist or isn't double
                    new_pos = elevator_pos_entry.getDouble(0.0)
                    if abs(new_pos - current_elevator_pos) > 0.01: # Update only on change
                        current_elevator_pos = new_pos
                        # Map position and update visualization LEDs
                        target_row = map_position_to_row(current_elevator_pos)
                        update_elevator_leds(target_row)
                    last_nt_read_time = current_time
                except Exception as nt_read_err:
                     print(f"\nError reading elevator position from NT: {nt_read_err}")
                     # Optionally stop updating viz if read fails repeatedly

            # --- Launchpad Button Event Processing ---
            button_events = lp.ButtonStateXY()

            if button_events:
                last_activity_time = current_time # Use current_time from this loop iteration
                if isinstance(button_events, list):
                    if len(button_events) % 3 == 0:
                        for i in range(0, len(button_events), 3):
                            try:
                                x, y, value = button_events[i], button_events[i+1], button_events[i+2]
                                is_pressed = (value > 0)
                                coord = (x, y)

                                print(f"  Pad Event: X={x}, Y={y}, Pressed={is_pressed}", end="")

                                # Handle Preset Pad LED feedback and Command Sending
                                if coord in XY_TO_PRESET_MAP:
                                    preset_num = XY_TO_PRESET_MAP[coord]
                                    target_color = PAD_COLOR_PRESSED if is_pressed else PAD_COLOR_IDLE
                                    try:
                                        # Avoid overwriting elevator bar with idle color on release
                                        is_elevator_area = (x < ELEVATOR_BAR_LENGTH and y == current_bar_row)
                                        if not (not is_pressed and is_elevator_area): # Don't set idle if it's where the bar is
                                             lp.LedCtrlXYByCode(x, y, target_color)
                                    except Exception as led_e:
                                        print(f"\n   Error setting preset LED ({x},{y}): {led_e}")

                                    if is_pressed:
                                        print(f" -> Mapped to PRESET {preset_num}")
                                        if networktables_connected:
                                            try:
                                                preset_entry.setDouble(float(preset_num))
                                                # print(f"     NT Sent: {NT_PRESET_COMMAND_KEY} = {float(preset_num)}") # Verbose
                                                # NetworkTables.flush() # Optional: Might help ensure it gets sent soone

                                            except Exception as nt_err:
                                                print(f"\n     ERROR sending NT: {nt_err}")
                                                # setting networktables_connected = False here
                                        else:
                                            print("     (NT Disconnected)")
                                    else:
                                         print(" -> Mapped Pad Released")
                                         # NOTE: We are NOT sending 0.0 on release. Robot handles reset.
                                else:
                                     print(" (Not mapped)") # Unmapped press/release

                            except IndexError:
                                print(f"\n  ERROR processing event chunk in {button_events}")
                                break
                            except Exception as proc_err:
                                print(f"\n  ERROR processing event data: {proc_err}")
                    else:
                         print(f"\n  WARN: ButtonStateXY() list length not multiple of 3: {button_events}")
                else:
                     print(f"\n  WARN: ButtonStateXY() returned non-list: {button_events}")

            else: # No button events
                if current_time - last_activity_time > check_interval:
                    print(f"  (Still listening... No Launchpad events in last {check_interval}s)")
                    last_activity_time = current_time

            time.sleep(0.02) # Polling interval

        except Exception as e:
            print(f"\nERROR in Launchpad loop: {e}")
            traceback.print_exc()
            return # Exit loop on error

def main():
    global lp, networktables_initialized_attempted
    networktables_initialized_attempted = False
    print("--- Launchpad.py -> NetworkTables Bridge START ---")
    print(f"Configured for RoboRIO IP: {ROBORIO_IP}")
    print("\n--- Initializing Launchpad ---")
    try:
        lp = launchpad.LaunchpadMk2()
        # Try finding by name first
        lp_found = False
        for i in range(10): # Check first few MIDI ports
            if lp.Open(i, LAUNCHPAD_MIDI_NAME):
                 print(f"SUCCESS: Opened Launchpad '{LAUNCHPAD_MIDI_NAME}' on port {i}.")
                 lp_found = True
                 break
        if not lp_found:
             print(f"WARN: Could not find Launchpad by name '{LAUNCHPAD_MIDI_NAME}'. Trying first available MK2...")
             if lp.Open(0, "mk2"):
                 print(f"SUCCESS: Opened first available Launchpad MK2 (may not be the intended one).")
             else:
                 print("ERROR: Could not open any Launchpad MK2.")
                 print("Check connections and if another program is using it.")
                 sys.exit(1)
    except Exception as e:
        print(f"ERROR: An exception occurred during Launchpad initialization: {e}")
        traceback.print_exc()
        if lp:
            try: lp.Close()
            except: pass
        sys.exit(1)

    initialize_networktables()

    try:
        launchpad_loop()
    except KeyboardInterrupt: print("\nCtrl+C detected...")
    except Exception as e: print(f"\nUnhandled exception in main execution: {e}"); traceback.print_exc()
    finally: cleanup(); sys.exit(0)

if __name__ == "__main__":
    main()

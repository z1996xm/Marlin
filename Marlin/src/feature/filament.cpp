#include "../inc/MarlinConfigPre.h"

#if HAS_FILAMENT_SFS

#include "filament.h"


FilamentSFSMonitor filament;

bool FilamentSFSMonitorBase::enabled = true,
     FilamentSFSMonitorBase::filamentsfs_status;  // = false


#if ENABLED(HOST_ACTION_COMMANDS)
  bool FilamentSFSMonitorBase::host_handling; // = false
#endif


#if HAS_FILAMENTSFS_RUNOUT_DISTANCE
  float SFSResponseDelayed::runout_distance_mm = FILAMENTSFS_RUNOUT_DISTANCE_MM;
  volatile float SFSResponseDelayed::runout_mm_countdown[NUM_FILAMENT_SFS_SENSORS];
  uint8_t FilamentSFSSensorEncoder::motion_detected;
#endif
int8_t SFSResponseDelayed::runout_count[NUM_FILAMENT_SFS_SENSORS]; // = 0


#include "../MarlinCore.h"
#include "../feature/pause.h"
#include "../gcode/queue.h"


#if ENABLED(HOST_ACTION_COMMANDS)
  #include "host_actions.h"
#endif

void event_filament_func(const uint8_t extruder){

    if (did_pause_print) return;  // Action already in progress. Purge triggered repeated runout. 

    #if ENABLED(TOOLCHANGE_MIGRATION_FEATURE)
        if (migration.in_progress) {
        DEBUG_ECHOLNPGM("Migration Already In Progress");
        return;  // Action already in progress. Purge triggered repeated runout.
        }
        if (migration.automode) {
        DEBUG_ECHOLNPGM("Migration Starting");
        if (extruder_migration()) return;
        }
    #endif

    TERN_(EXTENSIBLE_UI, ExtUI::onFilamentRunout(ExtUI::getTool(extruder)));
    TERN_(DWIN_LCD_PROUI, DWIN_FilamentRunout(extruder));  

    #if ANY(HOST_PROMPT_SUPPORT, HOST_ACTION_COMMANDS, MULTI_FILAMENT_SENSOR)
        const char tool = '0' + TERN0(MULTI_FILAMENTSFS_SENSOR, extruder);
    #endif 

    //action:out_of_filament
    #if ENABLED(HOST_PROMPT_SUPPORT)
        hostui.prompt_do(PROMPT_FILAMENT_RUNOUT, F("FilamentRunout T"), tool); //action:out_of_filament
    #endif

    const bool run_runout_script = !filament.host_handling;

    #if ENABLED(HOST_ACTION_COMMANDS)
        if (run_runout_script
        && ( strstr(FILAMENT_SFS_SCRIPT, "M600")
            || strstr(FILAMENT_SFS_SCRIPT, "M125")
            || TERN0(ADVANCED_PAUSE_FEATURE, strstr(FILAMENT_SFS_SCRIPT, "M25"))
        )
        ) {
        hostui.paused(false);
        }
        else {
        // Legacy Repetier command for use until newer version supports standard dialog
        // To be removed later when pause command also triggers dialog
        #ifdef ACTION_ON_FILAMENT_RUNOUT
            hostui.action(F(ACTION_ON_FILAMENT_RUNOUT " T"), false);
            SERIAL_CHAR(tool);
            SERIAL_EOL();
        #endif

        hostui.pause(false);
        }
        SERIAL_ECHOPGM(" " ACTION_REASON_ON_FILAMENT_RUNOUT " ");
        SERIAL_CHAR(tool);
        SERIAL_EOL();
    #endif // HOST_ACTION_COMMANDS

    if (run_runout_script) {
        #if MULTI_FILAMENT_SENSOR
            char script[strlen(FILAMENT_RUNOUT_SCRIPT) + 1];
            sprintf_P(script, PSTR(FILAMENT_RUNOUT_SCRIPT), tool);
            #if ENABLED(FILAMENT_RUNOUT_SENSOR_DEBUG)
                SERIAL_ECHOLNPGM("Runout Command: ", script);
            #endif
            queue.inject(script);
        #else
            #if ENABLED(FILAMENT_RUNOUT_SENSOR_DEBUG)
                SERIAL_ECHOPGM("Runout Command: ");
                SERIAL_ECHOLNPGM(FILAMENT_RUNOUT_SCRIPT);
            #endif
        queue.inject(F(FILAMENT_SFS_SCRIPT));
    #endif
  }
}

#endif // HAS_FILAMENT_SFS
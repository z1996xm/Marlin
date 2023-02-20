#include "../inc/MarlinConfigPre.h"

#if HAS_FILAMENT_SFS

#include "filament.h"


FilamentSFSMonitor filament;

bool FilamentSFSMonitorBase::enabled = true,
     FilamentSFSMonitorBase::filamentsfs_status;  // = false


#if HAS_FILAMENTSFS_RUNOUT_DISTANCE
  float SFSResponseDelayed::runout_distance_mm = FILAMENTSFS_RUNOUT_DISTANCE_MM;
  volatile float SFSResponseDelayed::runout_mm_countdown[NUM_FILAMENT_SFS_SENSORS];
  uint8_t FilamentSFSSensorEncoder::motion_detected;
  int8_t SFSResponseDelayed::runout_count[NUM_FILAMENT_SFS_SENSORS]; // = 0
#endif


#include "../MarlinCore.h"
#include "../feature/pause.h"
#include "../gcode/queue.h"


void event_filament_func(const uint8_t extruder){

    if (did_pause_print) return;  // Action already in progress. Purge triggered repeated runout. 

    TERN_(EXTENSIBLE_UI, ExtUI::onFilamentRunout(ExtUI::getTool(extruder)));
    TERN_(DWIN_LCD_PROUI, DWIN_FilamentRunout(extruder));  

    #if ANY(HOST_PROMPT_SUPPORT, HOST_ACTION_COMMANDS, MULTI_FILAMENT_SENSOR)
        const char tool = '0' + TERN0(MULTI_FILAMENT_SENSOR, extruder);
    #endif 

    //action:out_of_filament
    #if ENABLED(HOST_PROMPT_SUPPORT)
        hostui.prompt_do(PROMPT_FILAMENT_RUNOUT, F("FilamentRunout T"), tool); //action:out_of_filament
    #endif

    const bool run_runout_script = !filament.host_handling;

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
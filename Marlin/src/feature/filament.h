#pragma once

/**
 * feature/runout.h - Runout sensor support
 */

#include "../sd/cardreader.h"
#include "../module/printcounter.h"
#include "../module/planner.h"
#include "../module/stepper.h" // for block_t
#include "../gcode/queue.h"
#include "../feature/pause.h"

#include "../inc/MarlinConfig.h"

#if ENABLED(EXTENSIBLE_UI)
  #include "../lcd/extui/ui_api.h"
#endif

#ifndef FILAMENT_RUNOUT_THRESHOLD
  #define FILAMENT_RUNOUT_THRESHOLD 5
#endif

void event_filament_func(const uint8_t extruder);

template<class RESPONSE_T, class SENSOR_T>
class TFilamentSFSMonitor;
class FilamentSFSSensorEncoder;
class SFSResponseDelayed;

typedef TFilamentSFSMonitor<
         SFSResponseDelayed,  FilamentSFSSensorEncoder
        > FilamentSFSMonitor;

extern FilamentSFSMonitor filament;


class FilamentSFSMonitorBase {
  public:
    static bool enabled, filamentsfs_status;

    #if ENABLED(HOST_ACTION_COMMANDS)
      static bool host_handling;
    #else
      static constexpr bool host_handling = false;
    #endif
};


template<class RESPONSE_T, class SENSOR_T>
class TFilamentSFSMonitor : public FilamentSFSMonitorBase {
  private:
    typedef RESPONSE_T response_t;
    typedef SENSOR_T   sensor_t;
    static  response_t response;
    static  sensor_t   sensor;

  public:
    static void setup() {
      sensor.setup();
      reset();
    }

    static void reset() {
      filamentsfs_status = false;
      response.reset();
    }

    // Call this method when filament is present,
    // so the response can reset its counter.
    static void filamentrun_present(const uint8_t extruder) {
      response.filamentrun_present(extruder);
    }


    // Handle a block completion. RunoutResponseDelayed uses this to
    // add up the length of filament moved while the filament is out.
    static void block_completed(const block_t * const b) {
      if (enabled) {
        response.block_completed(b);
        sensor.block_completed(b);
      }
    }

    // Give the response a chance to update its counter.
    static void run() {
      if (enabled && !filamentsfs_status && (printingIsActive() || did_pause_print)) {
        response.run();
        sensor.run();
        const uint8_t sfs_flags = response.has_run_out();
        #if MULTI_FILAMENT_SENSOR
          #if ENABLED(WATCH_ALL_RUNOUT_SENSORS)
            const bool ran_out = !!sfs_flags;  // any sensor triggers
            uint8_t extruder = 0;
            if (ran_out) {
              uint8_t bitmask = sfs_flags;
              while (!(bitmask & 1)) {
                bitmask >>= 1;
                extruder++;
              }
            }
          #else
            const bool ran_out = TEST(sfs_flags, active_extruder);  // suppress non active extruders
            uint8_t extruder = active_extruder;
          #endif
        #else
          const bool ran_out = !!sfs_flags;
          uint8_t extruder = active_extruder;
        #endif

        #if ENABLED(FILAMENT_RUNOUT_SENSOR_DEBUG)
          if (sfs_flags) {
            SERIAL_ECHOPGM("Runout Sensors: ");
            LOOP_L_N(i, 8) SERIAL_ECHO('0' + TEST(sfs_flags, i));
            SERIAL_ECHOPGM(" -> ", extruder);
            if (ran_out) SERIAL_ECHOPGM(" RUN OUT");
            SERIAL_EOL();
          }
        #endif
        
        if (ran_out) 
        {
          filamentsfs_status = true;
          event_filament_func(extruder);
          planner.synchronize();
          return ;
        }
      }
    }

};

/*************************** FILAMENT PRESENCE SENSORS ***************************/

class FilamentSFSSensorBase {
  protected:
    /**
     * Called by FilamentSFSSensorSwitch::run when filament is detected.
     * Called by FilamentSFSSensorEncoder::block_completed when motion is detected.
     */
    static void filamentrun_present(const uint8_t extruder) {
      filament.filamentrun_present(extruder); // ...which calls response.filament_present(extruder)
    }

  public:
    static void setup() {
      #define _INIT_RUNOUT_PIN(P,S,U,D) do{ if (ENABLED(U)) SET_INPUT_PULLUP(P); else if (ENABLED(D)) SET_INPUT_PULLDOWN(P); else SET_INPUT(P); }while(0)
      #define  INIT_RUNOUT_PIN(N) _INIT_RUNOUT_PIN(SFS_RUNOUT##N##_PIN, SFS_RUNOUT##N##_STATE, SFS_RUNOUT##N##_PULLUP, SFS_RUNOUT##N##_PULLDOWN)

      #if NUM_RUNOUT_SENSORS >= 1
        INIT_RUNOUT_PIN(1);
      #endif
      #if NUM_RUNOUT_SENSORS >= 2
        INIT_RUNOUT_PIN(2);
      #endif
      #if NUM_RUNOUT_SENSORS >= 3
        INIT_RUNOUT_PIN(3);
      #endif
      #if NUM_RUNOUT_SENSORS >= 4
        INIT_RUNOUT_PIN(4);
      #endif
      #if NUM_RUNOUT_SENSORS >= 5
        INIT_RUNOUT_PIN(5);
      #endif
      #if NUM_RUNOUT_SENSORS >= 6
        INIT_RUNOUT_PIN(6);
      #endif
      #if NUM_RUNOUT_SENSORS >= 7
        INIT_RUNOUT_PIN(7);
      #endif
      #if NUM_RUNOUT_SENSORS >= 8
        INIT_RUNOUT_PIN(8);
      #endif
      #undef _INIT_RUNOUT_PIN
      #undef  INIT_RUNOUT_PIN
    }

    // Return a bitmask of runout pin states
    static uint8_t poll_runout_pins() {
      #define _OR_RUNOUT(N) | (READ(SFS_RUNOUT##N##_PIN) ? _BV((N) - 1) : 0)
      return (0 REPEAT_1(NUM_RUNOUT_SENSORS, _OR_RUNOUT));
      #undef _OR_RUNOUT
    }

    // Return a bitmask of runout flag states (1 bits always indicates runout)
    static uint8_t poll_runout_states() {
      return poll_runout_pins() ^ uint8_t(0
        #if NUM_RUNOUT_SENSORS >= 1
          | (SFS_RUNOUT1_STATE ? 0 : _BV(1 - 1))
        #endif
        #if NUM_RUNOUT_SENSORS >= 2
          | (SFS_RUNOUT2_STATE ? 0 : _BV(2 - 1))
        #endif
        #if NUM_RUNOUT_SENSORS >= 3
          | (SFS_RUNOUT3_STATE ? 0 : _BV(3 - 1))
        #endif
        #if NUM_RUNOUT_SENSORS >= 4
          | (SFS_RUNOUT4_STATE ? 0 : _BV(4 - 1))
        #endif
        #if NUM_RUNOUT_SENSORS >= 5
          | (SFS_RUNOUT5_STATE ? 0 : _BV(5 - 1))
        #endif
        #if NUM_RUNOUT_SENSORS >= 6
          | (SFS_RUNOUT6_STATE ? 0 : _BV(6 - 1))
        #endif
        #if NUM_RUNOUT_SENSORS >= 7
          | (SFS_RUNOUT7_STATE ? 0 : _BV(7 - 1))
        #endif
        #if NUM_RUNOUT_SENSORS >= 8
          | (SFS_RUNOUT8_STATE ? 0 : _BV(8 - 1))
        #endif
      );
    }

};

  /**
   * This sensor uses a magnetic encoder disc and a Hall effect
   * sensor (or a slotted disc and optical sensor). The state
   * will toggle between 0 and 1 on filament movement. It can detect
   * filament runout and stripouts or jams.
   */
  class FilamentSFSSensorEncoder : public FilamentSFSSensorBase {
    private:
      static bool poll_runout_state(const uint8_t extruder) {
        const uint8_t runout_states = poll_runout_states();
        #if MULTI_FILAMENT_SENSOR
          if ( !TERN0(DUAL_X_CARRIAGE, idex_is_duplicating())
            && !TERN0(MULTI_NOZZLE_DUPLICATION, extruder_duplication_enabled)
          ) return TEST(runout_states, extruder); // A specific extruder ran out
        #else
          UNUSED(extruder);
        #endif
        return !!runout_states;                   // Any extruder ran out
      }

    public:
      static void block_completed(const block_t * const b) {
      }

      static void run() { 
        LOOP_L_N(s, NUM_RUNOUT_SENSORS) {
          const bool out = poll_runout_state(s);
          if (!out) filamentrun_present(s);
          #if ENABLED(FILAMENT_RUNOUT_SENSOR_DEBUG)
            static uint8_t was_out; // = 0
            if (out != TEST(was_out, s)) {
              TBI(was_out, s);
              SERIAL_ECHOLNF(F("Filament Sensor "), AS_DIGIT(s), out ? F(" OUT") : F(" IN"));
            }
          #endif
        }
      }
  };

/********************************* RESPONSE TYPE *********************************/
  // RunoutResponseDelayed triggers a runout event only if the length
  // of filament specified by FILAMENT_RUNOUT_DISTANCE_MM has been fed
  // during a runout condition.
  class SFSResponseDelayed {
    private:
      static constexpr int8_t runout_threshold = FILAMENT_RUNOUT_THRESHOLD;
      static int8_t runout_count[NUM_RUNOUT_SENSORS];

    public:

      static void reset() {
        LOOP_L_N(i, NUM_RUNOUT_SENSORS) filamentrun_present(i);
      }

      static void run() {
        LOOP_L_N(i, NUM_RUNOUT_SENSORS) if (runout_count[i] >= 0) runout_count[i]--;
      }

      static uint8_t has_run_out() {
        uint8_t runout_flags = 0;
        LOOP_L_N(i, NUM_RUNOUT_SENSORS) if (runout_count[i] < 0) SBI(runout_flags, i);
        return runout_flags;
      }

      static void block_completed(const block_t * const b) {
      }

      static void filamentrun_present(const uint8_t extruder) {
        runout_count[extruder] = runout_threshold;
      }
  };

# Strain Gauge Z Offset Calibration Module
#
# Uses an HX711 load cell sensor at a fixed bed corner pad to calibrate
# the Z offset independently from the main probe (bltouch/crtouch).
# This module does NOT register as 'probe', allowing coexistence with bltouch.
#
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from .load_cell import hx71x, ads1220, load_cell

CALIBRATION_MOVE_SPEED = 1.0      # mm/s - slow descent speed
CALIBRATION_STEP = 0.05           # mm - step size per iteration
FORCE_THRESHOLD_GRAMS = 20.       # grams of force to detect contact
TARE_SAMPLES = 40                 # number of samples for tare averaging
CONTACT_CONFIRM_SAMPLES = 5       # consecutive samples above threshold
RETRACT_DIST = 2.0                # mm to retract after contact
CLEARANCE_Z = 10.0                # mm Z hop for safe travel moves

class StrainGaugeZCal:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        # Pad position on the bed (machine coordinates)
        self.pad_x = config.getfloat('pad_x', 0.)
        self.pad_y = config.getfloat('pad_y', 5.)
        # Z calibration parameters
        self.z_start_height = config.getfloat('z_start_height', 5.)
        self.z_min_position = config.getfloat('z_min_position', -2.)
        self.force_threshold = config.getfloat('force_threshold',
                                               FORCE_THRESHOLD_GRAMS)
        self.step_size = config.getfloat('step_size', CALIBRATION_STEP)
        self.speed = config.getfloat('speed', CALIBRATION_MOVE_SPEED)
        self.retract_dist = config.getfloat('retract_dist', RETRACT_DIST)
        self.clearance_z = config.getfloat('clearance_z', CLEARANCE_Z)
        self.travel_speed = config.getfloat('travel_speed', 80.)
        # Target probe section to save z_offset to (usually bltouch)
        self.target_probe_section = config.get('target_probe_section',
                                               'bltouch')
        # Initialize the HX711 sensor and LoadCell (standalone)
        sensors = {}
        sensors.update(hx71x.HX71X_SENSOR_TYPES)
        sensors.update(ads1220.ADS1220_SENSOR_TYPE)
        sensor_class = config.getchoice('sensor_type', sensors)
        self.sensor = sensor_class(config)
        self.load_cell = load_cell.LoadCell(config, self.sensor)
        # Register gcode commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command(
            'STRAIN_GAUGE_CALIBRATE_Z', self.cmd_CALIBRAR_Z_OFFSET,
            desc="Calibrate Z offset using strain gauge at bed corner pad")
        self.gcode.register_command(
            'STRAIN_GAUGE_TARE', self.cmd_STRAIN_GAUGE_TARE,
            desc="Tare the strain gauge sensor")
        self.gcode.register_command(
            'STRAIN_GAUGE_READ', self.cmd_STRAIN_GAUGE_READ,
            desc="Read current strain gauge force")
        logging.info("strain_gauge_z_cal: initialized (pad at X=%.1f Y=%.1f)"
                     % (self.pad_x, self.pad_y))

    def _get_toolhead(self):
        return self.printer.lookup_object('toolhead')

    def _check_homed(self):
        toolhead = self._get_toolhead()
        curtime = self.printer.get_reactor().monotonic()
        kin_status = toolhead.get_kinematics().get_status(curtime)
        if 'x' not in kin_status['homed_axes'] or \
           'y' not in kin_status['homed_axes'] or \
           'z' not in kin_status['homed_axes']:
            return False
        return True

    def _tare_sensor(self, gcmd):
        """Tare the load cell by averaging readings with no applied force."""
        gcmd.respond_info("Taring strain gauge sensor...")
        try:
            tare_counts = self.load_cell.avg_counts(TARE_SAMPLES)
        except Exception as e:
            raise gcmd.error("Tare failed: %s" % str(e))
        self.load_cell.tare(tare_counts)
        pct = self.load_cell.counts_to_percent(tare_counts)
        gcmd.respond_info("Tare complete: counts=%d (%.2f%%)" %
                          (int(tare_counts), pct))
        return tare_counts

    def _read_force_grams(self):
        """Read current force in grams from the load cell."""
        if not self.load_cell.is_calibrated():
            return None
        if not self.load_cell.is_tared():
            return None
        try:
            counts = self.load_cell.avg_counts(5)
            return self.load_cell.counts_to_grams(counts)
        except Exception:
            return None

    def _probe_z_with_force(self, gcmd):
        """Two-stage strain-gauge Z probing.

        1. Descend quickly using configured step_size to locate contact.
        2. Back off and retare near the bed.
        3. Descend at 0.01mm resolution for the final contact measurement.
        4. Confirm contact with repeated force readings at the SAME Z.
        """
        toolhead = self._get_toolhead()
        reactor = self.printer.get_reactor()

        coarse_step = self.step_size
        fine_step = min(0.01, coarse_step)
        backoff = max(0.50, coarse_step * 10.0)

        gcmd.respond_info(
            "Starting two-stage Z probe "
            "(threshold=%.1fg, coarse=%.3fmm, fine=%.3fmm)..."
            % (self.force_threshold, coarse_step, fine_step))

        # --------------------------------------------------------------
        # Stage 1: fast coarse search
        # --------------------------------------------------------------
        while True:
            curpos = toolhead.get_position()

            if curpos[2] <= self.z_min_position:
                raise gcmd.error(
                    "Z min position (%.2f) reached without contact!"
                    % self.z_min_position)

            next_z = curpos[2] - coarse_step
            toolhead.manual_move([None, None, next_z], self.speed)
            toolhead.wait_moves()

            reactor.pause(reactor.monotonic() + 0.05)

            force = self._read_force_grams()
            if force is None:
                continue

            if abs(force) >= self.force_threshold:
                coarse_contact_z = toolhead.get_position()[2]
                gcmd.respond_info(
                    "Coarse contact found: Force=%.1fg at Z=%.4f"
                    % (force, coarse_contact_z))
                break

        # --------------------------------------------------------------
        # Back away far enough for the bed/strain gauge to fully unload.
        # Keep the ORIGINAL tare taken before probing. Retaring near
        # contact can zero-out residual mechanical load and shift the
        # detected contact point.
        # --------------------------------------------------------------
        fine_start_z = coarse_contact_z + backoff

        gcmd.respond_info(
            "Backing off %.3fmm to Z=%.4f for fine probe..."
            % (backoff, fine_start_z))

        toolhead.manual_move([None, None, fine_start_z], self.speed)
        toolhead.wait_moves()

        gcmd.respond_info(
            "Allowing strain gauge to relax; preserving original tare...")
        reactor.pause(reactor.monotonic() + 0.50)

        # --------------------------------------------------------------
        # Stage 2: fine search
        # --------------------------------------------------------------
        gcmd.respond_info(
            "Starting fine Z probe (step=%.3fmm)..."
            % fine_step)

        while True:
            curpos = toolhead.get_position()

            if curpos[2] <= self.z_min_position:
                raise gcmd.error(
                    "Z min position (%.2f) reached without contact!"
                    % self.z_min_position)

            next_z = curpos[2] - fine_step
            toolhead.manual_move([None, None, next_z], self.speed)
            toolhead.wait_moves()

            reactor.pause(reactor.monotonic() + 0.05)

            force = self._read_force_grams()
            if force is None:
                continue

            if abs(force) < self.force_threshold:
                continue

            # Confirm contact without moving Z any farther.
            # The previous implementation moved another full step for
            # every confirmation sample, which added systematic overshoot.
            confirmed = 1
            confirm_force = force

            for _ in range(CONTACT_CONFIRM_SAMPLES - 1):
                reactor.pause(reactor.monotonic() + 0.05)
                confirm_force = self._read_force_grams()

                if (confirm_force is None or
                        abs(confirm_force) < self.force_threshold):
                    confirmed = 0
                    break

                confirmed += 1

            if confirmed >= CONTACT_CONFIRM_SAMPLES:
                contact_z = toolhead.get_position()[2]
                gcmd.respond_info(
                    "Fine contact detected! Force=%.1fg at Z=%.4f"
                    % (confirm_force, contact_z))
                return contact_z

    cmd_CALIBRAR_Z_OFFSET_help = ("Calibrate Z offset using strain gauge "
                                  "at bed corner pad")
    def cmd_CALIBRAR_Z_OFFSET(self, gcmd):
        # Calibrate BLTouch/CR Touch Z offset using the V3 SE strain gauge.
        #
        # The CR Touch and nozzle must measure the SAME physical bed point.
        # The active bed mesh is temporarily cleared so it cannot alter
        # the calibration coordinate system.

        if not self._check_homed():
            raise gcmd.error("Printer must be homed first! Run G28")

        toolhead = self._get_toolhead()
        gcode = self.printer.lookup_object('gcode')
        reactor = self.printer.get_reactor()

        probe = self.printer.lookup_object('probe')
        current_offsets = probe.get_offsets()
        x_offset = current_offsets[0]
        y_offset = current_offsets[1]
        current_z_offset = current_offsets[2]

        # The configured pad_x/pad_y are NOZZLE coordinates for physical
        # contact with the strain gauge pad.
        #
        # To put the CR Touch over that SAME physical point:
        #   probe_position = nozzle_position + probe_offset
        # therefore:
        #   nozzle_position = pad_position - probe_offset
        probe_x = self.pad_x - x_offset
        probe_y = self.pad_y - y_offset

        # Remember active mesh so we can restore it after calibration.
        bed_mesh = self.printer.lookup_object('bed_mesh', None)
        mesh_was_active = False
        active_profile = ""

        if bed_mesh is not None:
            mesh_status = bed_mesh.get_status(reactor.monotonic())
            active_profile = mesh_status.get('profile_name', '')
            mesh_matrix = mesh_status.get('mesh_matrix', [[]])
            mesh_was_active = bool(mesh_matrix and mesh_matrix[0])

            if mesh_was_active and not active_profile:
                raise gcmd.error(
                    "Active bed mesh has no saved profile name. "
                    "Load a saved mesh profile before running calibration.")

        gcmd.respond_info(
            "=== Z OFFSET CALIBRATION (CR Touch + Strain Gauge) ===\n"
            "Strain pad physical point: X=%.2f Y=%.2f\n"
            "CR Touch offset: X=%.2f Y=%.2f\n"
            "CR Touch carriage position: X=%.2f Y=%.2f"
            % (self.pad_x, self.pad_y,
               x_offset, y_offset,
               probe_x, probe_y))

        try:
            # Remove mesh compensation during ALL calibration measurements.
            if mesh_was_active:
                gcmd.respond_info(
                    "Temporarily clearing bed mesh profile '%s'..."
                    % active_profile)
                gcode.run_script_from_command("BED_MESH_CLEAR")

            # ----------------------------------------------------------
            # STEP 1: CR Touch measurement of the physical strain pad
            # ----------------------------------------------------------
            curpos = toolhead.get_position()
            safe_z = max(curpos[2], self.clearance_z)
            toolhead.manual_move([None, None, safe_z], 10.)
            toolhead.wait_moves()

            gcmd.respond_info(
                "Moving CR Touch over strain pad at carriage "
                "X=%.2f Y=%.2f..."
                % (probe_x, probe_y))

            toolhead.manual_move(
                [probe_x, probe_y, None], self.travel_speed)
            toolhead.wait_moves()

            # Use this Klipper build's native probe API.
            # run_probe() handles configured samples, tolerance,
            # retries and averaging, and returns [x, y, z].
            probe_pos = probe.run_probe(gcmd)

            if probe_pos is None or len(probe_pos) < 3:
                raise gcmd.error("CR Touch did not return a valid probe result")

            probe_contact_z = probe_pos[2]

            gcmd.respond_info(
                "CR Touch estimates bed contact at Z=%.4f"
                % probe_contact_z)

            # Retract before moving the nozzle onto the pad.
            curpos = toolhead.get_position()
            retract_z = max(
                curpos[2] + self.retract_dist,
                self.clearance_z)
            toolhead.manual_move([None, None, retract_z], 10.)
            toolhead.wait_moves()

            # ----------------------------------------------------------
            # STEP 2: Nozzle strain-gauge measurement at SAME bed point
            # ----------------------------------------------------------
            gcmd.respond_info(
                "Moving nozzle to strain gauge pad at X=%.1f Y=%.1f..."
                % (self.pad_x, self.pad_y))

            toolhead.manual_move(
                [self.pad_x, self.pad_y, None],
                self.travel_speed)
            toolhead.wait_moves()

            toolhead.manual_move(
                [None, None, self.z_start_height], 10.)
            toolhead.wait_moves()

            self._tare_sensor(gcmd)

            nozzle_contact_z = self._probe_z_with_force(gcmd)

            gcmd.respond_info(
                "Nozzle strain-gauge contact at Z=%.4f"
                % nozzle_contact_z)

            # Retract in raw toolhead coordinates while mesh is still clear.
            retract_z = nozzle_contact_z + self.retract_dist
            toolhead.manual_move([None, None, retract_z], 10.)
            toolhead.wait_moves()

            # ----------------------------------------------------------
            # STEP 3: Calculate offset using Klipper PROBE_CALIBRATE math
            #
            # new_offset =
            #     old_offset - nozzle_contact + probe_contact
            # ----------------------------------------------------------
            # Match Klipper's native PROBE_CALIBRATE calculation:
            # z_offset = probe trigger Z - physical nozzle contact Z
            new_z_offset = probe_contact_z - nozzle_contact_z

            if new_z_offset < 0.:
                raise gcmd.error(
                    "Calculated z_offset is negative (%.4f). "
                    "Calibration rejected."
                    % new_z_offset)

            gcmd.respond_info(
                "=== CALIBRATION RESULT ===\n"
                "CR Touch bed position: %.4f\n"
                "Nozzle contact position: %.4f\n"
                "Difference: %.4f\n"
                "Previous Z offset: %.4f\n"
                "New Z offset: %.4f\n"
                "Saving to [%s] z_offset..."
                % (probe_contact_z,
                   nozzle_contact_z,
                   probe_contact_z - nozzle_contact_z,
                   current_z_offset,
                   new_z_offset,
                   self.target_probe_section))

            configfile = self.printer.lookup_object('configfile')
            configfile.set(
                self.target_probe_section,
                'z_offset',
                "%.3f" % new_z_offset)

            gcmd.respond_info(
                "Z offset staged in config. "
                "DO NOT run SAVE_CONFIG until the result is verified.\n"
                "=== CALIBRATION COMPLETE ===")

        finally:
            if mesh_was_active and active_profile:
                try:
                    gcmd.respond_info(
                        "Restoring bed mesh profile '%s'..."
                        % active_profile)
                    gcode.run_script_from_command(
                        'BED_MESH_PROFILE LOAD="%s"'
                        % active_profile)
                except Exception as e:
                    gcmd.respond_info(
                        "WARNING: Could not restore bed mesh '%s': %s"
                        % (active_profile, str(e)))

    cmd_STRAIN_GAUGE_TARE_help = "Tare the strain gauge sensor"
    def cmd_STRAIN_GAUGE_TARE(self, gcmd):
        self._tare_sensor(gcmd)

    cmd_STRAIN_GAUGE_READ_help = "Read current strain gauge force"
    def cmd_STRAIN_GAUGE_READ(self, gcmd):
        if not self.load_cell.is_calibrated():
            raise gcmd.error("Load cell not calibrated! Set counts_per_gram "
                             "and reference_tare_counts in config.")
        if not self.load_cell.is_tared():
            raise gcmd.error("Load cell not tared! Run STRAIN_GAUGE_TARE")
        force = self._read_force_grams()
        if force is None:
            raise gcmd.error("Could not read force from sensor")
        counts = self.load_cell.avg_counts(5)
        gcmd.respond_info("Force: %.2f g  (raw counts: %d, tare: %d)"
                          % (force, int(counts),
                             self.load_cell.get_tare_counts()))

    def get_status(self, eventtime):
        return self.load_cell.get_status(eventtime)


def load_config(config):
    return StrainGaugeZCal(config)

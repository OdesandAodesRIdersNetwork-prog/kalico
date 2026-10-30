# Strain Gauge Z Offset Calibration Module
#
# Uses an HX711 load cell sensor at a fixed bed corner pad to calibrate
# the Z offset independently from the main probe (bltouch/crtouch).
# This module does NOT register as 'probe', allowing coexistence with bltouch.
#
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from klippy import mcu
from .load_cell import hx71x, ads1220, load_cell, load_cell_probe

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

        # Private native Kalico MCU load-cell probing stack.
        # CR Touch remains the global Klipper probe.
        self._native_config_helper = (
            load_cell_probe.LoadCellProbeConfigHelper(
                config, self.load_cell))
        self._native_trigger_dispatch = mcu.TriggerDispatch(
            self.sensor.get_mcu())
        self._native_tare_filter = (
            load_cell_probe.ContinuousTareFilterHelper(
                config,
                self.sensor,
                self._native_trigger_dispatch.get_command_queue()))
        self._native_mcu_probe = load_cell_probe.McuLoadCellProbe(
            config,
            self.load_cell,
            self._native_tare_filter.get_sos_filter(),
            self._native_config_helper,
            self._native_trigger_dispatch)
        self._native_primitives = load_cell_probe.LoadCellPrimitives(
            config,
            self._native_mcu_probe,
            self._native_tare_filter,
            self._native_config_helper)

        self._native_tapping_move = load_cell_probe.TappingMove(
            config, self._native_primitives)

        # Register Kalico's native no-motion load-cell tap test command.
        load_cell_probe.LoadCellProbeCommands(
            config, self._native_primitives)

        self.printer.register_event_handler(
            "klippy:mcu_identify",
            self._handle_native_mcu_identify)

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
        self.gcode.register_command(
            'STRAIN_GAUGE_NATIVE_AIR_TEST',
            self.cmd_STRAIN_GAUGE_NATIVE_AIR_TEST,
            desc="Test native MCU strain trigger safely above the bed")
        self.gcode.register_command(
            'STRAIN_GAUGE_NATIVE_STATUS',
            self.cmd_STRAIN_GAUGE_NATIVE_STATUS,
            desc="Report native MCU strain probe stepper attachment")
        self.gcode.register_command(
            'STRAIN_GAUGE_NATIVE_CONTACT_TEST',
            self.cmd_STRAIN_GAUGE_NATIVE_CONTACT_TEST,
            desc="Test native MCU nozzle contact on strain pad")
        self.gcode.register_command(
            'STRAIN_GAUGE_NATIVE_BENCHMARK',
            self.cmd_STRAIN_GAUGE_NATIVE_BENCHMARK,
            desc="Benchmark native MCU strain contact across probe speeds")
        self.gcode.register_command(
            'STRAIN_GAUGE_NOZZLE_PREP',
            self.cmd_STRAIN_GAUGE_NOZZLE_PREP,
            desc="Prepare and clean nozzle for strain Z calibration")
        logging.info("strain_gauge_z_cal: initialized (pad at X=%.1f Y=%.1f)"
                     % (self.pad_x, self.pad_y))

    def _handle_native_mcu_identify(self):
        kin = self._get_toolhead().get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis("z"):
                self._native_primitives.add_stepper(stepper)

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

    def _fresh_avg_counts(self, num_samples):
        """Average only load-cell samples captured after this call begins."""
        collector = self.load_cell.get_collector()
        reactor = self.printer.get_reactor()
        mcu = self.sensor.get_mcu()

        min_time = mcu.estimated_print_time(reactor.monotonic())
        collector.start_collecting(min_time=min_time)
        samples, errors = collector.collect_min(num_samples)

        if errors:
            raise self.printer.command_error(
                "Sensor reported %i errors while sampling"
                % (errors[0] + errors[1]))

        if not samples:
            raise self.printer.command_error(
                "No fresh strain-gauge samples received")

        return sum(sample[2] for sample in samples) / float(len(samples))

    def _tare_sensor(self, gcmd):
        """Tare the load cell using only newly captured sensor samples."""
        gcmd.respond_info("Taring strain gauge sensor...")
        try:
            tare_counts = self._fresh_avg_counts(TARE_SAMPLES)
        except Exception as e:
            raise gcmd.error("Tare failed: %s" % str(e))
        self.load_cell.tare(tare_counts)
        pct = self.load_cell.counts_to_percent(tare_counts)
        gcmd.respond_info("Tare complete: counts=%d (%.2f%%)" %
                          (int(tare_counts), pct))
        return tare_counts

    def _read_force_grams(self):
        """Read force using only samples captured after this read begins."""
        if not self.load_cell.is_calibrated():
            return None
        if not self.load_cell.is_tared():
            return None
        try:
            counts = self._fresh_avg_counts(5)
            return self.load_cell.counts_to_grams(counts)
        except Exception:
            return None

    def _probe_z_with_force(self, gcmd):
        """Two-stage strain-gauge Z probing.

        1. Descend quickly using configured step_size to locate contact.
        2. Back off, unload, and establish a local fine-probe baseline.
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

        # Measure the local unloaded baseline for this fine pass.
        # Do not retare the sensor. Subtract this small drift only from
        # the fine-probe force measurements.
        fine_baseline = self._read_force_grams()
        if fine_baseline is None:
            raise gcmd.error(
                "Could not establish fine-probe strain baseline")

        gcmd.respond_info(
            "Fine-probe local baseline: %.2fg"
            % fine_baseline)

        # --------------------------------------------------------------
        # Stage 2: fine search
        # --------------------------------------------------------------
        gcmd.respond_info(
            "Starting fine Z probe (step=%.3fmm)..."
            % fine_step)

        # Diagnostic force-vs-Z history. This does not change detection.
        fine_trace = []

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

            raw_force = self._read_force_grams()
            if raw_force is None:
                continue

            force = raw_force - fine_baseline

            fine_trace.append((toolhead.get_position()[2], force))
            if len(fine_trace) > 40:
                fine_trace.pop(0)

            if abs(force) < self.force_threshold:
                continue

            # Confirm contact without moving Z any farther.
            # The previous implementation moved another full step for
            # every confirmation sample, which added systematic overshoot.
            confirmed = 1
            confirm_force = force

            for _ in range(CONTACT_CONFIRM_SAMPLES - 1):
                reactor.pause(reactor.monotonic() + 0.05)

                raw_confirm_force = self._read_force_grams()
                if raw_confirm_force is None:
                    confirmed = 0
                    break

                confirm_force = (
                    raw_confirm_force - fine_baseline
                )

                if abs(confirm_force) < self.force_threshold:
                    confirmed = 0
                    break

                confirmed += 1

            if confirmed >= CONTACT_CONFIRM_SAMPLES:
                measured_z = toolhead.get_position()[2]
                contact_z = measured_z

                # Interpolate the configured force-threshold crossing
                # between the last below-threshold sample and the first
                # above-threshold sample. This removes the fine-step
                # quantization from the final contact estimate.
                if len(fine_trace) >= 2:
                    prev_z, prev_force = fine_trace[-2]
                    prev_mag = abs(prev_force)
                    cur_mag = abs(force)

                    if (prev_mag < self.force_threshold and
                            cur_mag > prev_mag):
                        fraction = (
                            (self.force_threshold - prev_mag) /
                            (cur_mag - prev_mag)
                        )
                        fraction = max(0.0, min(1.0, fraction))
                        contact_z = (
                            prev_z +
                            fraction * (measured_z - prev_z)
                        )

                gcmd.respond_info(
                    "Fine contact detected! Force=%.1fg\n"
                    "  measured step Z=%.4f\n"
                    "  interpolated threshold Z=%.5f"
                    % (confirm_force, measured_z, contact_z))

                trace_lines = [
                    "  Z=%8.4f  force=%8.2fg" % (z, f)
                    for z, f in fine_trace
                ]
                gcmd.respond_info(
                    "Fine force trace (oldest -> newest):\n%s"
                    % "\n".join(trace_lines))
                return contact_z

    cmd_CALIBRAR_Z_OFFSET_help = ("Calibrate Z offset using strain gauge "
                                  "at bed corner pad")
    def cmd_CALIBRAR_Z_OFFSET(self, gcmd):
        analyze_only = gcmd.get_int('ANALYZE_ONLY', 0, minval=0, maxval=1)
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
            # STEP 2: Native MCU strain-gauge measurement at SAME bed point
            # ----------------------------------------------------------
            gcmd.respond_info(
                "Moving nozzle to strain gauge pad at X=%.1f Y=%.1f..."
                % (self.pad_x, self.pad_y))

            toolhead.manual_move(
                [self.pad_x, self.pad_y, None],
                self.travel_speed)
            toolhead.wait_moves()

            native_speed = gcmd.get_float(
                "NATIVE_SPEED", 0.50, above=0.0)
            search_end_z = gcmd.get_float(
                "SEARCH_END_Z", 0.0, minval=-0.5, maxval=0.3)
            trigger_force = (
                self._native_config_helper.get_trigger_force_grams(gcmd))

            strain_contacts = []

            for pass_index in range(3):
                gcmd.respond_info(
                    "=== NATIVE STRAIN CONTACT PASS %d/3 ==="
                    % (pass_index + 1))

                # Start every pass safely above the bed.  The lower Z is
                # only a search boundary; the load-cell response determines
                # the measured physical contact height.
                toolhead.manual_move([None, None, 3.0], 5.0)
                toolhead.wait_moves()

                pos = toolhead.get_position()
                pos[2] = search_end_z

                epos, collector = self._native_primitives.probing_move(
                    self._native_tapping_move,
                    pos,
                    native_speed,
                    gcmd)

                toolhead.flush_step_generation()
                move_end = toolhead.get_last_move_time()
                results = collector.collect_until(move_end)
                samples = load_cell_probe.check_sensor_errors(
                    results, self.printer)

                trigger_z = epos[2]
                eventtime = reactor.monotonic()
                trigger_time = self._native_primitives.get_status(
                    eventtime)["last_trigger_time"]

                threshold_z, onset_z, fit_count = (
                    self._analyze_native_contact_samples(
                        samples,
                        trigger_time,
                        trigger_z,
                        native_speed,
                        trigger_force))

                if threshold_z is None:
                    raise gcmd.error(
                        "Could not determine raw force threshold "
                        "during native strain calibration")

                strain_contacts.append(threshold_z)

                gcmd.respond_info(
                    "Native strain pass %d/3:\n"
                    "  MCU halt Z=%.5f\n"
                    "  raw %.1fg crossing Z=%.5f\n"
                    "  onset estimate Z=%s"
                    % (
                        pass_index + 1,
                        trigger_z,
                        trigger_force,
                        threshold_z,
                        ("%.5f" % onset_z)
                        if onset_z is not None else "N/A"))

                toolhead.manual_move([None, None, 3.0], 5.0)
                toolhead.wait_moves()

            sorted_contacts = sorted(strain_contacts)
            nozzle_contact_z = sorted_contacts[1]

            gcmd.respond_info(
                "Native nozzle strain-gauge median:\n"
                "  pass 1 = %.5f\n"
                "  pass 2 = %.5f\n"
                "  pass 3 = %.5f\n"
                "  MEDIAN raw %.1fg = %.5f"
                % (
                    strain_contacts[0],
                    strain_contacts[1],
                    strain_contacts[2],
                    trigger_force,
                    nozzle_contact_z))

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

            if analyze_only:
                gcmd.respond_info(
                    "ANALYZE_ONLY=1: Z offset was NOT staged or changed.\n"
                    "=== CALIBRATION ANALYSIS COMPLETE ===")
            else:
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

    def _analyze_native_contact_samples(
            self, samples, trigger_time, trigger_z, speed, trigger_force):
        mapped = []

        for sample in samples:
            sample_time = sample[0]
            force = sample[1]
            delta_t = trigger_time - sample_time
            sample_z = trigger_z + speed * delta_t

            if abs(delta_t) <= 0.150:
                mapped.append((sample_time, sample_z, force))

        threshold_z = None
        for i in range(1, len(mapped)):
            _, prev_z, prev_force = mapped[i - 1]
            _, cur_z, cur_force = mapped[i]

            prev_mag = abs(prev_force)
            cur_mag = abs(cur_force)

            if (prev_mag < trigger_force <= cur_mag
                    and cur_mag > prev_mag):
                fraction = (
                    (trigger_force - prev_mag) /
                    (cur_mag - prev_mag)
                )
                threshold_z = (
                    prev_z + fraction * (cur_z - prev_z)
                )
                break

        fit_min_force = max(3.0, trigger_force * 0.15)
        fit_max_force = trigger_force * 1.50
        fit_points = []

        for sample_time, sample_z, force in mapped:
            if sample_time > trigger_time:
                continue

            mag = abs(force)
            if fit_min_force <= mag <= fit_max_force:
                fit_points.append((mag, sample_z))

        contact_onset_z = None

        if len(fit_points) >= 2:
            n = float(len(fit_points))
            sx = sum(point[0] for point in fit_points)
            sy = sum(point[1] for point in fit_points)
            sxx = sum(point[0] * point[0] for point in fit_points)
            sxy = sum(point[0] * point[1] for point in fit_points)
            denom = n * sxx - sx * sx

            if abs(denom) > 1.0e-12:
                slope = (n * sxy - sx * sy) / denom
                contact_onset_z = (sy - slope * sx) / n

        return threshold_z, contact_onset_z, len(fit_points)

    def cmd_STRAIN_GAUGE_NOZZLE_PREP(self, gcmd):
        if not self._check_homed():
            raise gcmd.error("Printer must be homed first! Run G28")

        toolhead = self._get_toolhead()
        reactor = self.printer.get_reactor()

        start_x = gcmd.get_float("X", 2.0)
        start_y = gcmd.get_float("Y", 10.0)
        wipe_length = gcmd.get_float("LENGTH", 30.0, above=5.0)
        end_y = start_y + wipe_length

        probe_speed = gcmd.get_float("SPEED", 0.50, above=0.0)
        search_end_z = gcmd.get_float(
            "SEARCH_END_Z", 0.0, minval=-0.5, maxval=0.3)
        wipe_speed = gcmd.get_float("WIPE_SPEED", 5.0, above=0.0)
        wipe_relief = gcmd.get_float(
            "WIPE_RELIEF", 0.015, minval=0.0, maxval=0.10)
        passes = gcmd.get_int("PASSES", 6, minval=1, maxval=20)

        trigger_force = (
            self._native_config_helper.get_trigger_force_grams(gcmd))

        def probe_contact(x, y):
            toolhead.manual_move([None, None, 10.0], 10.0)
            toolhead.wait_moves()

            toolhead.manual_move([x, y, None], self.travel_speed)
            toolhead.wait_moves()

            toolhead.manual_move([None, None, 3.0], 5.0)
            toolhead.wait_moves()

            pos = toolhead.get_position()
            pos[2] = search_end_z

            epos, collector = self._native_primitives.probing_move(
                self._native_tapping_move,
                pos,
                probe_speed,
                gcmd)

            toolhead.flush_step_generation()
            move_end = toolhead.get_last_move_time()
            collected = collector.collect_until(move_end)
            samples = load_cell_probe.check_sensor_errors(
                collected, self.printer)

            trigger_z = epos[2]
            eventtime = reactor.monotonic()
            trigger_time = self._native_primitives.get_status(
                eventtime)["last_trigger_time"]

            threshold_z, onset_z, fit_count = (
                self._analyze_native_contact_samples(
                    samples,
                    trigger_time,
                    trigger_z,
                    probe_speed,
                    trigger_force))

            toolhead.manual_move([None, None, 5.0], 5.0)
            toolhead.wait_moves()

            if threshold_z is None:
                raise gcmd.error(
                    "Could not determine raw force threshold at wipe point")

            gcmd.respond_info(
                "Wipe-point contact X%.1f Y%.1f: "
                "halt=%.5f raw %.1fg=%.5f"
                % (
                    x, y,
                    trigger_z,
                    trigger_force,
                    threshold_z))

            return threshold_z

        gcmd.respond_info(
            "=== STRAIN GAUGE NOZZLE PREP ===\n"
            "Bed 60C, nozzle 180C probe, 200C wipe, 140C finish\n"
            "Wipe line X%.1f Y%.1f -> X%.1f Y%.1f"
            % (start_x, start_y, start_x, end_y))

        # Creality-style controlled thermal starting state.
        self.gcode.run_script_from_command("M106 S0")
        self.gcode.run_script_from_command("M140 S60")
        self.gcode.run_script_from_command("M104 S180")
        self.gcode.run_script_from_command("M190 S60")
        self.gcode.run_script_from_command("M109 S180")

        # Find the bed at both wipe endpoints using the native
        # strain probe instead of assuming bed height.
        start_contact = probe_contact(start_x, start_y)
        end_contact = probe_contact(start_x, end_y)

        # Heat to Creality's nozzle rubbing temperature.
        self.gcode.run_script_from_command("M109 S200")

        # Positive relief means we stay ABOVE the measured 20g
        # crossing.  We deliberately do not copy Creality's
        # aggressive -0.15mm penetration.
        start_wipe_z = start_contact + wipe_relief
        end_wipe_z = end_contact + wipe_relief

        gcmd.respond_info(
            "Wiping at Z %.5f -> %.5f, relief=+%.3fmm, passes=%d"
            % (
                start_wipe_z,
                end_wipe_z,
                wipe_relief,
                passes))

        toolhead.manual_move([None, None, 5.0], 5.0)
        toolhead.wait_moves()

        toolhead.manual_move(
            [start_x, start_y, None],
            self.travel_speed)
        toolhead.wait_moves()

        toolhead.manual_move(
            [None, None, start_wipe_z],
            2.0)
        toolhead.wait_moves()

        for i in range(passes):
            if i % 2 == 0:
                toolhead.manual_move(
                    [start_x, end_y, end_wipe_z],
                    wipe_speed)
            else:
                toolhead.manual_move(
                    [start_x, start_y, start_wipe_z],
                    wipe_speed)
            toolhead.wait_moves()

        toolhead.manual_move([None, None, 10.0], 5.0)
        toolhead.wait_moves()

        # Creality-style filament pumpback.
        # Retract while still at 200C so freshly cleaned nozzle does not
        # immediately grow another ooze/nub before Z calibration.
        pumpback_mm = gcmd.get_float(
            "PUMPBACK", 10.0, minval=0.0, maxval=20.0)

        if pumpback_mm > 0.0:
            gcmd.respond_info(
                "Filament pumpback: retracting %.1fmm before cooldown..."
                % pumpback_mm)

            gcode_move = self.printer.lookup_object('gcode_move')
            was_absolute_extrude = gcode_move.get_status(
                self.printer.get_reactor().monotonic()
            ).get('absolute_extrude', True)

            self.gcode.run_script_from_command("M83")
            self.gcode.run_script_from_command(
                "G1 E-%.3f F300" % pumpback_mm)

            if was_absolute_extrude:
                self.gcode.run_script_from_command("M82")
            else:
                self.gcode.run_script_from_command("M83")

        # Cool to Creality's calibration temperature.
        self.gcode.run_script_from_command("M104 S140")
        self.gcode.run_script_from_command("M106 S255")
        self.gcode.run_script_from_command("M109 S140")
        self.gcode.run_script_from_command("M106 S0")

        gcmd.respond_info(
            "=== NOZZLE PREP COMPLETE ===\n"
            "Bed target: 60C\n"
            "Nozzle target: 140C\n"
            "No Z offset or configuration was changed.")

    def cmd_STRAIN_GAUGE_NATIVE_BENCHMARK(self, gcmd):
        if not self._check_homed():
            raise gcmd.error("Printer must be homed first! Run G28")

        toolhead = self._get_toolhead()
        reactor = self.printer.get_reactor()
        trigger_force = (
            self._native_config_helper.get_trigger_force_grams(gcmd))

        start_z = 3.0
        end_z = 0.5
        retract_z = 5.0

        speed_cycles = [
            [0.25, 0.50, 1.00],
            [1.00, 0.50, 0.25],
            [0.25, 0.50, 1.00],
        ]

        results = []
        references = []

        probe = self.printer.lookup_object('probe')
        probe_offsets = probe.get_offsets()
        probe_x = self.pad_x - probe_offsets[0]
        probe_y = self.pad_y - probe_offsets[1]

        def read_temperatures():
            eventtime = reactor.monotonic()
            extruder_status = self.printer.lookup_object(
                'extruder').get_status(eventtime)
            bed_status = self.printer.lookup_object(
                'heater_bed').get_status(eventtime)

            return {
                "nozzle": extruder_status.get("temperature", 0.0),
                "nozzle_target": extruder_status.get("target", 0.0),
                "bed": bed_status.get("temperature", 0.0),
                "bed_target": bed_status.get("target", 0.0),
            }

        def take_cr_reference(label):
            # Measure the exact same physical bed point with CR Touch.
            toolhead.manual_move([None, None, 10.0], 10.0)
            toolhead.wait_moves()

            toolhead.manual_move(
                [probe_x, probe_y, None],
                self.travel_speed)
            toolhead.wait_moves()

            probe_pos = probe.run_probe(gcmd)

            if probe_pos is None or len(probe_pos) < 3:
                raise gcmd.error(
                    "CR Touch did not return a valid benchmark reference")

            temps = read_temperatures()

            reference = {
                "label": label,
                "z": probe_pos[2],
                "nozzle": temps["nozzle"],
                "nozzle_target": temps["nozzle_target"],
                "bed": temps["bed"],
                "bed_target": temps["bed_target"],
            }

            gcmd.respond_info(
                "CR %s: Z=%.5f  nozzle=%.1f/%.1fC  bed=%.1f/%.1fC"
                % (
                    label,
                    reference["z"],
                    reference["nozzle"],
                    reference["nozzle_target"],
                    reference["bed"],
                    reference["bed_target"]))

            # Return safely to nozzle-over-strain-pad position.
            toolhead.manual_move([None, None, 10.0], 10.0)
            toolhead.wait_moves()
            toolhead.manual_move(
                [self.pad_x, self.pad_y, None],
                self.travel_speed)
            toolhead.wait_moves()

            return reference

        gcmd.respond_info(
            "=== NATIVE STRAIN BENCHMARK ===\n"
            "9 contacts, fresh tare each pass\n"
            "Speeds: 0.25 / 0.50 / 1.00 mm/s\n"
            "Trigger force: %.1fg"
            % trigger_force)

        # Establish CR Touch reference before any strain contacts.
        references.append(take_cr_reference("START"))

        run_number = 0

        try:
            for cycle_index, speeds in enumerate(speed_cycles):
                for speed in speeds:
                    run_number += 1

                    toolhead.manual_move(
                        [None, None, start_z], 5.0)
                    toolhead.wait_moves()

                    pos = toolhead.get_position()
                    pos[2] = end_z

                    epos, collector = (
                        self._native_primitives.probing_move(
                            self._native_tapping_move,
                            pos,
                            speed,
                            gcmd))

                    toolhead.flush_step_generation()
                    move_end = toolhead.get_last_move_time()
                    collected = collector.collect_until(move_end)
                    samples = load_cell_probe.check_sensor_errors(
                        collected, self.printer)

                    trigger_z = epos[2]
                    eventtime = reactor.monotonic()
                    trigger_time = (
                        self._native_primitives.get_status(
                            eventtime)["last_trigger_time"])

                    threshold_z, onset_z, fit_count = (
                        self._analyze_native_contact_samples(
                            samples,
                            trigger_time,
                            trigger_z,
                            speed,
                            trigger_force))

                    temps = read_temperatures()

                    results.append({
                        "run": run_number,
                        "cycle": cycle_index + 1,
                        "speed": speed,
                        "halt": trigger_z,
                        "threshold": threshold_z,
                        "onset": onset_z,
                        "fit_count": fit_count,
                        "nozzle": temps["nozzle"],
                        "nozzle_target": temps["nozzle_target"],
                        "bed": temps["bed"],
                        "bed_target": temps["bed_target"],
                    })

                    gcmd.respond_info(
                        "Run %d/9 C%d %.2fmm/s: "
                        "halt=%.5f threshold=%s onset=%s fit=%d "
                        "Tn=%.1f/%.1fC Tb=%.1f/%.1fC"
                        % (
                            run_number,
                            cycle_index + 1,
                            speed,
                            trigger_z,
                            "%.5f" % threshold_z
                            if threshold_z is not None else "N/A",
                            "%.5f" % onset_z
                            if onset_z is not None else "N/A",
                            fit_count,
                            temps["nozzle"],
                            temps["nozzle_target"],
                            temps["bed"],
                            temps["bed_target"]))

                    toolhead.manual_move(
                        [None, None, retract_z], 5.0)
                    toolhead.wait_moves()

                    reactor.pause(
                        reactor.monotonic() + 0.50)

                # CR Touch reference after every complete speed cycle.
                references.append(
                    take_cr_reference(
                        "CYCLE%d" % (cycle_index + 1)))

        except self.printer.command_error:
            if not self.printer.is_shutdown():
                toolhead.manual_move(
                    [None, None, retract_z], 5.0)
                toolhead.wait_moves()
            raise

        lines = ["=== BENCHMARK SUMMARY ==="]

        for speed in [0.25, 0.50, 1.00]:
            rows = [
                row for row in results
                if abs(row["speed"] - speed) < 0.0001
            ]

            lines.append("%.2f mm/s:" % speed)

            for field, label in [
                    ("halt", "MCU halt"),
                    ("threshold", "20g cross"),
                    ("onset", "Contact onset")]:

                values = [
                    row[field] for row in rows
                    if row[field] is not None
                ]

                if not values:
                    lines.append(
                        "  %-13s N/A" % label)
                    continue

                avg = sum(values) / float(len(values))
                minimum = min(values)
                maximum = max(values)
                spread = maximum - minimum

                lines.append(
                    "  %-13s avg=%.5f min=%.5f "
                    "max=%.5f spread=%.5f"
                    % (
                        label,
                        avg,
                        minimum,
                        maximum,
                        spread))

        all_onsets = [
            row["onset"] for row in results
            if row["onset"] is not None
        ]

        if all_onsets:
            lines.append(
                "All onset values: avg=%.5f spread=%.5f"
                % (
                    sum(all_onsets) / float(len(all_onsets)),
                    max(all_onsets) - min(all_onsets)))

        lines.append("CR Touch references:")

        for ref in references:
            lines.append(
                "  %-6s Z=%.5f  nozzle=%.1fC  bed=%.1fC"
                % (
                    ref["label"],
                    ref["z"],
                    ref["nozzle"],
                    ref["bed"]))

        cr_values = [ref["z"] for ref in references]
        if cr_values:
            lines.append(
                "CR Touch drift: min=%.5f max=%.5f spread=%.5f"
                % (
                    min(cr_values),
                    max(cr_values),
                    max(cr_values) - min(cr_values)))

        nozzle_values = [row["nozzle"] for row in results]
        bed_values = [row["bed"] for row in results]

        if nozzle_values:
            lines.append(
                "Nozzle temp range during strain runs: "
                "%.1fC to %.1fC"
                % (min(nozzle_values), max(nozzle_values)))

        if bed_values:
            lines.append(
                "Bed temp range during strain runs: "
                "%.1fC to %.1fC"
                % (min(bed_values), max(bed_values)))

        gcmd.respond_info("\n".join(lines))

    def cmd_STRAIN_GAUGE_NATIVE_CONTACT_TEST(self, gcmd):
        if not self._check_homed():
            raise gcmd.error("Printer must be homed first! Run G28")

        toolhead = self._get_toolhead()

        start_z = 3.0
        end_z = gcmd.get_float("END_Z", 0.5, minval=-0.5, maxval=0.5)
        speed = gcmd.get_float("SPEED", 0.25, above=0.0)
        retract_z = 5.0
        test_x = gcmd.get_float("X", self.pad_x)
        test_y = gcmd.get_float("Y", self.pad_y)

        gcmd.respond_info(
            "Native contact test: X=%.1f Y=%.1f, "
            "start Z=%.2f, limit Z=%.2f, speed=%.2f mm/s"
            % (test_x, test_y, start_z, end_z, speed))

        # Travel safely above the pad first.
        toolhead.manual_move([None, None, 10.0], 10.0)
        toolhead.wait_moves()
        toolhead.manual_move(
            [test_x, test_y, None], self.travel_speed)
        toolhead.wait_moves()
        toolhead.manual_move([None, None, start_z], 5.0)
        toolhead.wait_moves()

        pos = toolhead.get_position()
        pos[2] = end_z

        try:
            epos, collector = self._native_primitives.probing_move(
                self._native_tapping_move, pos, speed, gcmd)

            toolhead.flush_step_generation()
            move_end = toolhead.get_last_move_time()
            results = collector.collect_until(move_end)
            samples = load_cell_probe.check_sensor_errors(
                results, self.printer)

        except self.printer.command_error:
            if not self.printer.is_shutdown():
                toolhead.manual_move([None, None, retract_z], 5.0)
                toolhead.wait_moves()
            raise

        trigger_z = epos[2]
        eventtime = self.printer.get_reactor().monotonic()
        trigger_time = self._native_primitives.get_status(
            eventtime)["last_trigger_time"]

        # Map timestamped HX711 samples back onto the constant-speed
        # Z trajectory around the MCU trigger.
        trigger_force = (
            self._native_config_helper.get_trigger_force_grams(gcmd))
        trace = []
        mapped = []

        for sample in samples:
            sample_time = sample[0]
            force = sample[1]

            delta_t = trigger_time - sample_time
            sample_z = trigger_z + speed * delta_t

            if abs(delta_t) <= 0.150:
                mapped.append((sample_time, sample_z, force))
                trace.append(
                    "  dt=%+8.4f s  Z=%8.5f  force=%8.2f g"
                    % (-delta_t, sample_z, force))

        # Interpolate the raw force-threshold crossing.
        threshold_z = None
        for i in range(1, len(mapped)):
            _, prev_z, prev_force = mapped[i - 1]
            _, cur_z, cur_force = mapped[i]
            prev_mag = abs(prev_force)
            cur_mag = abs(cur_force)

            if (prev_mag < trigger_force <= cur_mag
                    and cur_mag > prev_mag):
                fraction = (
                    (trigger_force - prev_mag) /
                    (cur_mag - prev_mag)
                )
                threshold_z = (
                    prev_z + fraction * (cur_z - prev_z)
                )
                break

        # Estimate physical contact onset by fitting Z versus force
        # magnitude through the rising contact ramp, then extrapolating
        # that line back to zero force.  This is diagnostic only.
        fit_min_force = max(3.0, trigger_force * 0.15)
        fit_max_force = trigger_force * 1.50
        fit_points = []

        for sample_time, sample_z, force in mapped:
            # Fit only the rising force ramp BEFORE the MCU trigger.
            # Post-trigger samples include the compressed/settled plateau
            # and will bias the extrapolated contact onset.
            if sample_time > trigger_time:
                continue

            mag = abs(force)
            if fit_min_force <= mag <= fit_max_force:
                fit_points.append((mag, sample_z))

        contact_onset_z = None
        if len(fit_points) >= 2:
            n = float(len(fit_points))
            sx = sum(point[0] for point in fit_points)
            sy = sum(point[1] for point in fit_points)
            sxx = sum(point[0] * point[0] for point in fit_points)
            sxy = sum(point[0] * point[1] for point in fit_points)
            denom = n * sxx - sx * sx

            if abs(denom) > 1.0e-12:
                slope = (n * sxy - sx * sy) / denom
                contact_onset_z = (sy - slope * sx) / n

        toolhead.manual_move([None, None, retract_z], 5.0)
        toolhead.wait_moves()

        gcmd.respond_info(
            "Native nozzle contact SUCCESS at Z=%.5f; "
            "trigger_time=%.6f; retracted to Z=%.1f"
            % (trigger_z, trigger_time, retract_z))

        if threshold_z is not None:
            gcmd.respond_info(
                "Raw %.1fg threshold crossing Z=%.5f"
                % (trigger_force, threshold_z))
        else:
            gcmd.respond_info(
                "Raw %.1fg threshold crossing could not be interpolated"
                % trigger_force)

        if contact_onset_z is not None:
            gcmd.respond_info(
                "Estimated contact onset Z=%.5f "
                "(linear ramp fit, %d samples)"
                % (contact_onset_z, len(fit_points)))
        else:
            gcmd.respond_info(
                "Estimated contact onset unavailable "
                "(only %d ramp samples)"
                % len(fit_points))

        if trace:
            gcmd.respond_info(
                "HX711 waveform around trigger:\n%s"
                % "\n".join(trace))
        else:
            gcmd.respond_info(
                "HX711 waveform: no samples found within +/-150 ms")

    def cmd_STRAIN_GAUGE_NATIVE_STATUS(self, gcmd):
        steppers = self._native_primitives.get_steppers()
        names = [s.get_name() for s in steppers]
        gcmd.respond_info(
            "Native strain trigger steppers: %s"
            % (", ".join(names) if names else "NONE"))

    cmd_STRAIN_GAUGE_NATIVE_AIR_TEST_help = (
        "Test native MCU strain trigger safely above the bed")
    def cmd_STRAIN_GAUGE_NATIVE_AIR_TEST(self, gcmd):
        if not self._check_homed():
            raise gcmd.error("Printer must be homed first! Run G28")

        toolhead = self._get_toolhead()
        reactor = self.printer.get_reactor()
        start_z = 15.0
        end_z = 8.0
        speed = 0.5

        gcmd.respond_info(
            "Native MCU air test: moving to Z=%.1f. "
            "Descent begins in 3 seconds to Z=%.1f at %.1f mm/s. "
            "Press the bed ONLY after Z starts moving."
            % (start_z, end_z, speed))

        toolhead.manual_move([None, None, start_z], 10.0)
        toolhead.wait_moves()

        reactor.pause(reactor.monotonic() + 3.0)

        pos = toolhead.get_position()
        pos[2] = end_z

        try:
            epos, is_good = self._native_tapping_move.probing_move(
                pos, speed, gcmd)
        except self.printer.command_error:
            if not self.printer.is_shutdown():
                toolhead.manual_move([None, None, start_z], 10.0)
                toolhead.wait_moves()
            raise

        trigger_z = epos[2]

        toolhead.manual_move([None, None, start_z], 10.0)
        toolhead.wait_moves()

        gcmd.respond_info(
            "Native MCU trigger SUCCESS at Z=%.5f; retracted to Z=%.1f"
            % (trigger_z, start_z))

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

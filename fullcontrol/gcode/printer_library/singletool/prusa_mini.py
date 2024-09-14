from fullcontrol.gcode import Point, Printer, Extruder, ManualGcode, PrinterCommand, GcodeComment, Buildplate, Hotend, Fan, StationaryExtrusion
import fullcontrol.gcode.printer_library.singletool.base_settings as base_settings


def set_up(user_overrides: dict):
    ''' DO THIS
    '''

    # overrides for this specific printer relative those defined in base_settings.py
    printer_overrides = {"printer_command_list": {
        "home": "G28 ; home axes",
        "retract": "G1 E-2.8 F2700 ; retract",
        "unretract": "G1 E2.8 F2700 ; unretract",
        "absolute_coords": "G90 ; absolute coordinates",
        "relative_coords": "G91 ; absolute coordinates",
        "units_mm": "G21 ; set units to millimeters"
    }}
    # update default initialization settings with printer-specific overrides and user-defined overrides
    initialization_data = {**base_settings.default_initial_settings, **printer_overrides}
    initialization_data = {**initialization_data, **user_overrides}

    sps = []

    sps.append(ManualGcode(text="M201 X4000 Y4000 Z400 E5000")) # sets maximum accelerations, mm/sec^2
    sps.append(ManualGcode(text="M203 X400 Y400 Z12 E80")) # sets maximum feedrates, mm / sec
    sps.append(ManualGcode(text="M204 P4000 R1250 T4000")) # sets acceleration (P, T) and retract acceleration (R), mm/sec^2
    sps.append(ManualGcode(text="M205 X8.00 Y8.00 Z2.00 E10.00")) # sets the jerk limits, mm/sec
    sps.append(ManualGcode(text="M205 S0 T0")) # sets the minimum extruding and travel feed rate, mm/sec
    sps.append(ManualGcode(text="M862.3 P \"MINI\"")) # printer model check
    sps.append(ManualGcode(text="M862.1 P0.6")) # nozzle diameter check
    sps.append(ManualGcode(text="M862.5 P2")) # g-code level check
    sps.append(ManualGcode(text="M862.6 P\"Input shaper\"")) # FW feature check

    sps.append(PrinterCommand(id='absolute_coords'))
    sps.append(ManualGcode(text="M83")) # extruder relative mode
    sps.append(Buildplate(temp=initialization_data["bed_temp"], wait=False)) # set bed temp
    sps.append(Hotend(temp=170, wait=False)) # set nozzle temp
    sps.append(Buildplate(temp=initialization_data["bed_temp"], wait=True)) # wait for bed temp
    sps.append(Hotend(temp=170, wait=True)) # wait for nozzle temp

    sps.append(ManualGcode(text="M569 S1 X Y")) # set stealthchop for X Y
    sps.append(ManualGcode(text="M204 T1250"))  # set travel acceleration
    sps.append(PrinterCommand(id='home'))
    sps.append(ManualGcode(text="G29"))
    sps.append(Hotend(temp=initialization_data["nozzle_temp"], wait=False))
    sps.append(ManualGcode(text="G92 E0"))

    sps.append(ManualGcode(text="G1 X0 Y-2 Z3 F2400"))

    sps.append(Hotend(temp=initialization_data["nozzle_temp"], wait=True))

    sps.append(ManualGcode(text="G1 X10 Z0.2 F1000\nG1 X70 E8 F900\nG1 X140 E10 F700\nG92 E0")) # primer

    sps.append(ManualGcode(text="M569 S0 X Y")) # set spreadcycle for X Y
    sps.append(ManualGcode(text="M204 T4000")) # restore travel acceleration
    sps.append(ManualGcode(text="M572 W0.06")) # set smooth time
    sps.append(ManualGcode(text="M221 S95")) # set flow

    sps.append(PrinterCommand(id='units_mm'))
    sps.append(PrinterCommand(id='absolute_coords'))
    sps.append(Extruder(relative_gcode=True))
    sps.append(ManualGcode(text="M900 K0.12")) # Filament gcode
    sps.append(ManualGcode(text="M572 S0.22"))
    sps.append(ManualGcode(text="M204 T4000"))
    sps.append(ManualGcode(text="M204 P3000"))
    sps.append(PrinterCommand(id='retract'))

    sps.append(ManualGcode(text=';-----\n; END OF STARTING PROCEDURE\n;-----\n'))

    eps = []

    eps.append(ManualGcode(text='\n;-----\n; START OF ENDING PROCEDURE\n;-----\n'))
    eps.append(PrinterCommand(id='relative_coords'))
    eps.append(ManualGcode(text="G1 Z12.1 F720"))  # Move print head up
    sps.append(PrinterCommand(id='absolute_coords'))
    eps.append(ManualGcode(text="G1 X170 Y170 F4200"))  # park print head
    eps.append(ManualGcode(text="G1 Z60.1 F720"))  # Move print head further up
    eps.append(ManualGcode(text="M73 P99 R0"))
    eps.append(ManualGcode(text="M73 Q99 S0"))
    eps.append(ManualGcode(text="G4"))  # wait
    eps.append(ManualGcode(text="M104 S0"))  # turn off temperature
    eps.append(ManualGcode(text="M140 S0"))  # turn off heatbed
    eps.append(ManualGcode(text="M107"))  # turn off fan
    eps.append(ManualGcode(text="M221 S100"))  # reset flow
    eps.append(ManualGcode(text="M572 S0"))  # reset PA
    eps.append(ManualGcode(text="M569 S1 X Y"))  # reset to stealthchop for X Y
    eps.append(ManualGcode(text="M84"))  # disable motors

    initialization_data['starting_procedure_steps'] = sps
    initialization_data['ending_procedure_steps'] = eps

    return initialization_data

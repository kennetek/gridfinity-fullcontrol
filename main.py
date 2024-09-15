import fullcontrol as fc
from gridfinity import Gridfinity

options = {
    'EW': 0.8,
    'EH': 0.30,
    'LENGTH': 42,
    'HEIGHT': 42,
    'SPAN': 2,
    'nozzle_temp': 225,
    'bed_temp': 85,
    'print_speed': 1800,
    'print_speed_bridge': 300,
    'fan_percent': 20,
    'printer_name': 'prusa_mini',
}


gridfinity = Gridfinity(options)
steps = gridfinity.generate()

# hover the cursor over the lines in the plot to check xyz positions of the points in the design
# fc.transform(steps, 'plot', fc.PlotControls(style='line', zoom=0.7))

# uncomment the next line to create a plot with real heights/widths for extruded lines to preview the real 3D printed geometry
plot_controls = fc.PlotControls(
    style='tube',
    zoom=0.7,
    initialization_data={
        'extrusion_width': options['EW'],
        'extrusion_height': options['EH']})
fc.transform(steps, 'plot', plot_controls)

# uncomment the next line to create a neat preview (click the top-left button in the plot for a .png file) - post and tag @FullControlXYZ :)
plot_controls = fc.PlotControls(
    neat_for_publishing=True,
    zoom=0.5,
    initialization_data={
        'extrusion_width': options['EW'],
        'extrusion_height': options['EH']})
fc.transform(steps, 'plot', plot_controls)

# generate and save gcode

gcode_controls = fc.GcodeControls(
    printer_name=options['printer_name'],
    save_as= f'1x{options['SPAN']}x{options['HEIGHT']}-gridfinity-fullcontrol',
    initialization_data={
        'primer': 'no_primer',
        'print_speed': options['print_speed'],
        'nozzle_temp': options['nozzle_temp'],
        'bed_temp': options['bed_temp'],
        'fan_percent': options['fan_percent'],
        'extrusion_width': options['EW'],
        'extrusion_height': options['EH'],
        'relative_e': True})
gcode = fc.transform(steps, 'gcode', gcode_controls)

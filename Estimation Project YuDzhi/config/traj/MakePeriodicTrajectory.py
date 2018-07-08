import math;

def fmt(value):
    return "%.3f" % value

period = [4, 2, 4]
radius = 1.5
timestep = 0.01
maxtime = max(period)*3
timemult = [1, 1, 1]
phase=[0,0,0]
amp = [1,0.4,.5]
center = [0, 0, -2]

with open('FigureEightFF.txt', 'w') as the_file:
    t=0;
    px = 0;
    py = 0;
    pz = 0;
    
    while t <= maxtime:
        x = math.sin(t * 2 * math.pi / period[0] + phase[0]) * radius * amp[0] + center[0];
        y = math.sin(t * 2 * math.pi / period[1] + phase[1]) * radius * amp[1] + center[1];
        z = math.sin(t * 2 * math.pi / period[2] + phase[2]) * radius * amp[2] + center[2];
        the_file.write(fmt(t) + "," + fmt(x) + "," + fmt(y) + "," + fmt(z));
        vx = math.cos(t * 2 * math.pi / period[0] + phase[0]) * 2 * math.pi * radius * amp[0] / period[0];
        vy = math.cos(t * 2 * math.pi / period[1] + phase[1]) * 2 * math.pi * radius * amp[1] / period[1];
        vz = math.cos(t * 2 * math.pi / period[2] + phase[2]) * 2 * math.pi * radius * amp[2] / period[2];
    		######## BEGIN STUDENT CODE
        the_file.write("," + fmt(vx) + "," + fmt(vy) + "," + fmt(vz));
        ax = -math.sin(t * 2 * math.pi / period[0] + phase[0]) * radius * amp[0] * (2 * math.pi / period[0]) ** 2
        ay = -math.sin(t * 2 * math.pi / period[1] + phase[1]) * radius * amp[1] * (2 * math.pi / period[1]) ** 2
        az = -math.sin(t * 2 * math.pi / period[2] + phase[2]) * radius * amp[2] * (2 * math.pi / period[2]) ** 2
        the_file.write("," + fmt(ax) + "," + fmt(ay) + "," + fmt(az));
    		######## END STUDENT CODE

    		######## EXAMPLE SOLUTION
#        the_file.write("," + fmt((x-px)/timestep) + "," + fmt((y-py)/timestep) + "," + fmt((z-pz)/timestep));
#        px = x;
#        py = y;
#        pz = z;
    		######## END EXAMPLE SOLUTION
		
        the_file.write("\n");
        
        t += timestep;
            

#Terascan & Standa must be connected

#Import libraries:
import sys
import numpy as np
import time as t

# Paths, these are required to work
sys.path.insert(0,'C:/Users/ruurdk/Downloads/Documents/Standa/multiscanner/src') 
sys.path.insert(0,'C:/Users/ruurdk/Downloads/Standa/multiscanner/src/multiscanner/scanner')
sys.path.insert(0,'C:/Users/ruurdk/Downloads/export_ruurd/Standa/multiscanner/src/multiscanner/scanner')
sys.path.insert(0,'C:/Users/ruurdk/Downloads/Toptica/control_software/') # Path for Tera scan

# Import stage and Terascan control software
import standa_stage as stage_control                            # Load Standa stage control software
from DLCsmartTHz_V3_0_0 import DLCsmartTHz, NetworkConnection   # Load THz VNA (DLCsmart) control software


####### Functions 
### Standa stages
def move_xyz_stage(Xstage, Ystage, Zstage, Xstep_pos, Ystep_pos, Zstep_pos):
    Xstep_pos = int(Xstep_pos)
    Ystep_pos = int(Ystep_pos)
    Zstep_pos = int(Zstep_pos)

    print("Move to x,y,z (steps):", Xstep_pos, Ystep_pos, Zstep_pos)

    # Move to number of steps position
    errorX = Xstage.move_1D_simple(Xstep_pos)
    errorY = Ystage.move_1D_simple(Ystep_pos)
    errorZ = Zstage.move_1D_simple(Zstep_pos)

    # Wait to stabalise
    t.sleep(t_rest)

    while errorX or errorY or errorZ :
        print( "errorX : ", errorX, ", errorY : ",errorY, ", errorZ : ", errorZ )
        
        print ("Re-initialise stages")
        close_stages(Xstage, Ystage, Zstage)
        [Xstage, Ystage, Zstage] = initialise_stages_XYZ_1()
        t.sleep(5)

        errorX = Xstage.move_1D_simple(Xstep_pos) 
        errorY = Ystage.move_1D_simple(Ystep_pos)
        errorZ = Zstage.move_1D_simple(Zstep_pos)

        print( "New = errorX : ", errorX, ", errorY : ",errorY, ", errorZ : ", errorZ )

def close_stages(Xstage, Ystage, Zstage):
    # Close
    Xstage.close()
    Ystage.close()
    Zstage.close()

def initialise_stages_XYZ_1():
    x1 = stage_control.Stage(axis_count=1, axis_name="X1")
    y1 = stage_control.Stage(axis_count=1, axis_name="Z1")
    z1 = stage_control.Stage(axis_count=1, axis_name="Y1")

    return x1, y1, z1

### Terascan
def set_frequency(dlc, freq):
    dlc.frequency.frequency_set.set(freq)
    t.sleep(3)
    print( 'Actual frequency (GHz): ',  np.round(dlc.frequency.frequency_act.get()) )

def set_integration_time(LockIn, t_int):
    LockIn.integration_time.set(t_int)
    print( 'Integration time (ms): ',  np.round(LockIn.integration_time.get()) )

def initialise_THz_VNA(dlc, freq, t_int):
    # Lasers 
    LaserCon    = dlc.laser_operation
    # Turn both lasers on
    LaserCon.emission_global_enable.set(True)

    # LockIn: set all measurement parameters + does the measurement of the THz wave
    LockIn      = dlc.lockin
    ampgain     = LockIn.amplifier_gain.get()       # transimpedance amplifier (V / A )
    LockIn.mod_out_set_to_zero()                    # Set Bias voltage to zero

    # Data aquisition and storage
    Measure     = LockIn.fast_phase_modulation
    set_frequency(dlc, freq)                    # set frequency of THz VNA to measurement frequency 
    set_integration_time(LockIn, t_int)         # set integration time of THz VNA

    return LockIn, Measure, ampgain, LaserCon

def shutdown_TeraScan(dlc, LockIn):
    # Set Bias voltage to zero
    #LockIn.mod_out_set_to_zero()
    print( " Modulation ouput amplitude (V): ", LockIn.mod_out_amplitude.get(), "\n odulation output offset (V): ", LockIn.mod_out_offset.get() )
    # Turn lasers off
    #LaserCon.emission_global_enable.set(False)

    # Close connection 
    dlc.close()
 
#Optica
def measure(i, Measure):
    print( "Measurement id: ", i)
    # Measurement
    t.sleep(0.1)
    Measure.clear() # clear buffer & start aquisition
    t.sleep(0.3) # Wait 
    Measure.stop_acquisition() # Stop aquisition
    Measure.data_valid # wait until data is valid
        
    print("Amplitude (nA):", Measure.fit_data_amplitude_nanoamp.get(), "Phase:", Measure.fit_data_phase.get() )
    data_int[i,3] = t.perf_counter()
    data_int[i,4] = Measure.fit_data_amplitude_nanoamp.get()
    data_int[i,5] = Measure.fit_data_phase.get()



####### Specifics of measurement ####### 
savedir = 'C:/Users/ruurdk/Downloads/export_ruurd/Data'
date    = '20240405'
device_name     = 'P2P#2'
fname           = device_name + '_test'

### min, max, centre ppositions (in steps) for x, y, z.
# THESE ARE HARD CODED INTO THE STANDA STAGES!
xmin    = 0
xmax    = 0
xcen    = 0

ymin    = 0
ymax    = 0
ycen    = 0

zmin    = -40000
zmax    = 40000
zcen    = 0

### Define scanning grid
# Step resolution stage: needed for conversion from mm to #steps
Xstep   = 1.25e-4 #x step size in mm
Ystep   = 1.25e-4 #y step size in mm
Zstep   = 1.25e-4 #z step size in mm

# Range
xrange  = 80 #total range of scan in mm
yrange  = 0 #mm
zrange  = 0 #mm

# Number of points
Nx = 81
Ny = 0 
Nz = 0

# Create scan grid
xstart  = 0 
xstop   = np.round( xrange / Xstep )
ystart  = 0 
ystop   = np.round( yrange / Ystep )
zstart  = 0
zstop   = np.round( zrange / Zstep )
xpos    = -1 * ( np.linspace(xstart,xstop,Nx,dtype=int) - xcen - xstop//2 )  # in steps
ypos    = -1 * ( np.linspace(ystart,ystop,Ny,dtype=int) - ycen - ystop//2 ) # in steps
zpos    = -1 * ( np.linspace(zstart,zstop,Nz,dtype=int) - zcen - zstop//2 )
#zpos    = np.linspace(zstart,zstop,Nz,dtype=int) # in steps

# Frequency
freq    = 414 # GHz
c       = 300 # mm Ghz-1
wl      = c / freq # mm
wl_4    = wl / 4 #mm
wl_4c   = wl_4 / Xstep #Counts

# Integration time
t_int   = 300 # ms
w       = 10 # windowing for moving average
dist    = 50 
t_rest  = .1 # time to wait for stabalisation of linear stages in s
t_restm = .05 # time to wait for stabalisation before clearing measurment buffer in s: 50 ms
Ntry_set= 5 

# Time between reference scans in seconds 
tref    = 120 #s
Nbit    = 1020
t_int = 0.5

#Create empty array
data_int = np.zeros([2*((Nx)+(Ny)+(Nz))+2, 6])

# Main of software
def main():
    with DLCsmartTHz(NetworkConnection("192.168.54.82")) as dlc:
        print(np.shape(data_int))
        ### Initialise THz VNA 
        LaserCon = dlc.laser_operation
        
        # Turn both lasers on
        #LaserCon.emission_global_enable.set(True)
        set_frequency(dlc, freq)
        t.sleep(1)

        FreqScanFast    = dlc.frequency
        LockIn          = dlc.lockin
        t_int           = LockIn.integration_time.get() / 1000
        ampgain         = LockIn.amplifier_gain.get() # transimpedance amplifier (V / A )

        # Set Bias voltage to default
        LockIn.mod_out_set_to_default()
        Biasamp = LockIn.mod_out_amplitude.get()
        Biasphase = LockIn.mod_out_offset.get()
        t.sleep(1) 
        print( "Set bias to default \n modulation ouput amplitude (V): ", Biasamp, "\n modulation output offset (V): ", Biasphase)
        print( "integration time (ms): ", t_int * 1000, '\n amplifier gain (V/A): ', ampgain)
        
        #Data aquisition 
        Measure         = LockIn.fast_phase_modulation
        #print( "Fast phase modulation enabled? ", LockIn.fast_phase_modulation.enabled.get() )

        ### Initialise Standa stages
        x1 = stage_control.Stage(axis_count=1, axis_name="X1")
        y1 = stage_control.Stage(axis_count=1, axis_name="Y1")
        z1 = stage_control.Stage(axis_count=1, axis_name="Z1")


        # Move along stage
        start_time = t.perf_counter()
        i=0
        for idxpos in range(Nx):
            move_xyz_stage(x1, y1, z1, xpos[idxpos], ycen, zcen)
            data_int[i,0], data_int[i,1], data_int[i,2] = xpos[idxpos]*Xstep, ycen*Xstep, zcen*Xstep
            measure(i, Measure)
            i += 1
            move_xyz_stage(x1, y1, z1, xpos[idxpos], ycen, zcen+wl_4c)
            data_int[i,0], data_int[i,1], data_int[i,2] = xpos[idxpos]*Xstep, ycen*Xstep, (zcen+wl_4c)*Xstep
            measure(i, Measure)
            i += 1
        print("End")
        
        #Header
        header = ' # Format: X (mm), Y (mm), Z (mm), t (s), Amplitude(nA), Phase(rad) \n # Modulation ouput amplitude (V): %r \n # Modulation output offset (V): %r \n # Grid: %r x %r \n # Frequency: %r GHz \n' % (Biasamp, Biasphase, Nx, Ny, freq)
    
        # Close connection to Standa stages and Terascan
        close_stages(x1, y1, z1)
        
        #Update time
        data_int[:,3] = data_int[:,3] - start_time
        
        #Save file
        np.savetxt('C:/Users/ruurdk/Downloads/export_ruurd/Data/Linear_scan_x_414GHz_2_c.csv', data_int, delimiter=' ', header=header, comments='')

if __name__ == "__main__":
    main()


# Function
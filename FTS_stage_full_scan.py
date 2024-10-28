#Import libraries
import sys
import numpy as np
import time as t
from time import sleep
import logging

logger = logging.getLogger(__name__)

# Paths required
sys.path.insert(0, 'C:/Users/ruurdk/Documents/egress/egse-master/python')
sys.path.insert(0, 'C:/Users/ruurdk/Documents/egress/')
sys.path.insert(0,'C:/Users/ruurdk/Downloads/Toptica/control_software/') # Path for Tera scan

# Import stage and Terascan control software
from DLCsmartTHz_V3_0_0 import DLCsmartTHz, NetworkConnection   # Load THz VNA (DLCsmart) control software
from gssw.ucelib.ensemble import Ensemble
from fts import PoemmFts



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
    
def measure(i, Measure):
    print( "Measurement id: ", i)
    # Measurement
    t.sleep(0.1)
    Measure.clear() # clear buffer & start aquisition
    t.sleep(0.3) # Wait 
    Measure.stop_acquisition() # Stop aquisition
    Measure.data_valid # wait until data is valid
        
    print("Amplitude (nA):", Measure.fit_data_amplitude_nanoamp.get(), "Phase:", Measure.fit_data_phase.get() )
    #data_int[i,3] = t.perf_counter()
    #data_int[i,4] = Measure.fit_data_amplitude_nanoamp.get()
    #data_int[i,5] = Measure.fit_data_phase.get()

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
xrange  = 0 #total range of scan in mm
yrange  = 0 #mm
zrange  = 2.5 #mm

# Number of points
Nx = 1
Ny = 1 
Nz = 201

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
freq    = 1280 # GHz
c       = 300 # mm Ghz-1
wl      = c / freq # mm
wl_4    = wl / 4 #mm
wl_4c   = wl_4 / Xstep #Counts

# Integration time
t_int   = 100 # ms
w       = 10 # windowing for moving average
dist    = 50 
t_rest  = .1 # time to wait for stabalisation of linear stages in s
t_restm = .05 # time to wait for stabalisation before clearing measurment buffer in s: 50 ms
Ntry_set= 5 

# Time between reference scans in seconds 
tref    = 120 #s
Nbit    = 1020
t_int = 0.5

# Main of software
def main():
    with DLCsmartTHz(NetworkConnection("192.168.54.82")) as dlc:
        ### Initialise THz VNA 
        LaserCon = dlc.laser_operation
        
        # Turn both lasers on
        LaserCon.emission_global_enable.set(True)
        set_frequency(dlc, freq)
        t.sleep(1)

        FreqScanFast    = dlc.frequency
        LockIn          = dlc.lockin
        t_int           = 0.1
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

        ### Initialise FTS Stage
    
        #Connect to FTS:
        fts = PoemmFts('192.168.1.2', 8000)
        
        #Clear FTS of faults:
        fts.setStageFaultsClear()
        print(fts.getStageStatus())
    
        #Enable and home stage:
        fts.setStageEnabled()
        fts.setStageHome()
        if not fts.getStageHomed():
            print("Homing FTS stage")
            fts.getStageHomed()
        print("Homed")
        print(fts.getStagePosition())
         
    
        #Set resolution and range:
        delta = wl_4/40
        ranged = np.arange(-24, 25, delta)
        
        #Collect data:
        data = np.zeros((len(ranged), 5))
        j = 0
        for i in ranged:
            #FTS Stage command & readout
            
            fts.setStagePosition(i)
            print(fts.getStagePosition())           
                
            time = t.perf_counter()
                
            startTime = t.time()

            while not fts.getStageStatus()['InPosition2']:
                print(f"Current position: {fts.getStagePosition()}")
                t.sleep(0.1)
                    
                if t.time() - startTime > 1:
                    print("Position error not < 0.0001mm, breaking")
                    break
            
            Measure.clear() # clear buffer & start aquisition
            t.sleep(0.1) # Wait 
            Measure.stop_acquisition() # Stop aquisition
            Measure.data_valid # wait until data is valid
            print("Amplitude (nA):", Measure.fit_data_amplitude_nanoamp.get(), "Phase:", Measure.fit_data_phase.get() )
            amp = Measure.fit_data_amplitude_nanoamp.get()
            phase = Measure.fit_data_phase.get()
            encoder = fts.getStagePosition()
            data[j] = i, encoder, time, amp, phase
            j += 1

        fts.setStageDisabled()
        #fts.setStagePosition(-0.134+0.072*6-0.0036*i)
        np.savetxt('C:/Users/ruurdk/Downloads/Toptica/P2C_Scan2_1280GHz.csv', data, delimiter=' ', comments='')        


        ### TO SCAN SEVERAL FREQUENCIES ###
            
        # freqs = [600, 800, 900, 1030, 1280]
        
        # for k in freqs:
        #     print("Frequency:", k)
        #     set_frequency(dlc, k)
        #     t.sleep(1)
        
        #     fts.setStageFaultsClear()
        #     print(fts.getStageStatus())
        
        #     fts.setStageEnabled()
        #     fts.setStageHome()
        #     if not fts.getStageHomed():
        #         print("Homing FTS stage")
        #         fts.getStageHomed()
    
        #     print("Homed")
        #     print(fts.getStagePosition())
        #     print("Frequency:", k)
            
        #     delta = wl_4/40
        #     center = 0.44 #mm
        #     rang = 3.75 #mm
        #     left = center - rang
        #     right = center + rang
        #     print(left, right)
        #     print(delta)
            
        #     ranged = np.arange(-3.31, 4.19, delta)
        #     data = np.zeros((len(ranged), 5))
        #     j = 0
            
        #     for i in ranged:
        #     #FTS Stage command & readout
            
        #         fts.setStagePosition(i)
        #         print(fts.getStagePosition())           
                    
        #         time = t.perf_counter()
                    
        #         startTime = t.time()
    
        #         #while not fts.getStageStatus()['InPosition2']:
        #         #    print(f"Current position: {fts.getStagePosition()}")
        #         #    t.sleep(0.1)
                        
        #         #    if t.time() - startTime > 1:
        #         #        print("Position error not < 0.0001mm, breaking")
        #         #        break
                    
        #         t.sleep(0.3)
        #         Measure.clear() # clear buffer & start aquisition
        #         t.sleep(0.1) # Wait 
        #         Measure.stop_acquisition() # Stop aquisition
        #         Measure.data_valid # wait until data is valid
        #         print("Amplitude (nA):", Measure.fit_data_amplitude_nanoamp.get(), "Phase:", Measure.fit_data_phase.get() )
        #         amp = Measure.fit_data_amplitude_nanoamp.get()
        #         phase = Measure.fit_data_phase.get()
        #         encoder = fts.getStagePosition()
        #         data[j] = i, encoder, time, amp, phase
        #         j += 1

        #     fts.setStageDisabled()
        #         #fts.setStagePosition(-0.134+0.072*6-0.0036*i)
        #     np.savetxt(f'C:/Users/ruurdk/Downloads/export_ruurd/Data/P2C_ZOPD_{k}GHz.csv', data, delimiter=' ', comments='')
        # # ranged = np.arange(-24, 26, 1)
        # data = np.zeros((len(ranged), 6))
        # print(np.shape(data))
        # j=0
        # for i in ranged:
        #     #FTS Stage command & readout
        #     fts.setStagePosition(i)
        #     command = i
        #     sleep(0.3)
        #     encoder = fts.getStagePosition()
        #     print(encoder)
        #     diff = command - encoder
            
        #     #Measure TopTica
        #     Measure.clear() # clear buffer & start aquisition
        #     t.sleep(0.1) # Wait 
        #     Measure.stop_acquisition() # Stop aquisition
        #     Measure.data_valid # wait until data is valid
                
        #     print("Amplitude (nA):", Measure.fit_data_amplitude_nanoamp.get(), "Phase:", Measure.fit_data_phase.get() )
        #     time = t.perf_counter()
        #     amp = Measure.fit_data_amplitude_nanoamp.get()
        #     phase = Measure.fit_data_phase.get()
        #     data[j] = command, encoder, diff, time, amp, phase
        #     j += 1
        # #setStagePositionIncrement(self, increment, velocity=None):
        # #
        # #print(getStageMoving()) 
        # #while fts.getStageMoving():
        # #     print(f"Current position: {fts.getStagePosition()}")
        # #     sleep(0.1)
        
        # # np.savetxt('C:/Users/ruurdk/Downloads/Toptica/P2C_Scan_1280GHz.csv', data, delimiter=' ', comments='')

    
if __name__ == "__main__":
    main()


# Function
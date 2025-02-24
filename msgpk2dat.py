# Convert msgpk->dat
# GlobusM2 digitized
# to 16bit binary

import sys
import struct
import msgpack
from pathlib import Path, PurePath
#import json


gain    = 2.0
adc_lim = 0xFFF
# GlobusTS samlong data geometry
# 100 laser pulses per tokamak shot, 8boards, 16ch each, 1024 window size
num_imp   = 100
num_board = 8
num_ch    = 16
num_s     = 1024

print ("GlobusTS raw file converter")

if len(sys.argv) <= 1:
    print ("Usage: msgpk2dat any_file.msgpk [pulses=100] [gain=2.0]")
    sys.exit()

#ifname = Path('raw/7.msgpk') 
ifname = Path(sys.argv[1])

num_range = num_imp
if len(sys.argv) > 2: num_range = int (sys.argv[2])
if num_range > num_imp: num_range = num_imp

if len(sys.argv) > 3: gain = float (sys.argv[3])

with ifname.open(mode='rb') as fin:
    data = msgpack.unpackb(fin.read())
    print('open', ifname ,'ok')
    warnlim = False

    fname_stem = PurePath(ifname.name).stem
    with open(fname_stem+'.csv', 'w', 1) as foutd:
        with open(fname_stem+'.bin', 'wb') as foutb:

            vmin = adc_lim
            vmax = 0
            for imp in range(0, num_range):
                for s in range(0, num_s):
                    for ch in range(0, num_ch):
                        raw_val = data[imp]['ch'][ch][s]
                        if raw_val < 0: raw_val = 0
                        val = int (raw_val * gain)
                        if val>adc_lim : warnlim = True
                        if val>vmax: vmax = val
                        if val<vmin: vmin = val

                        #print( val , end=' ')
                        try:
                            foutd.write("{0:>4} ".format(val))
                            foutb.write(struct.pack('<H', val))
                        except:
                            print( 'err',imp,s,ch,'=',raw_val, val)

                        #TR channel not exist in Samlong
                        if ch == 7 or ch == 15: 
                            #print( 0, end=' ')
                            foutd.write("   0 ")
                            foutb.write(struct.pack('>H', 0))
                    #print()
                    foutd.write("\n")
                sys.stdout.write('.')
                sys.stdout.flush()
        if (warnlim): print(':lim!')
        print ('\nmin/max=', vmin, '/', vmax, ', write ', fname_stem, '.bin/svs ok', sep='')
#EOF
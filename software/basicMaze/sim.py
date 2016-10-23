from __future__ import print_function

def mapf( x,  in_min,  in_max,  out_min,  out_max):
    return (x - in_min)*1.0 * (out_max - out_min)*1.0 / ((in_max - in_min)*1.0) + out_min;


def leerIRs(ind, data):
    cal=[[236,226,184,120,83,64,49,39,32,27,23  ,20,17,15,14,13,12,11,10,8,7],
             [245,237,231,198,121,85,62,48,38,31,25, 21,18,15,13,12,11,10,9,8,7],
             [236,233,218,177,129,99,77,61,50,42,35,30,26,23,20,18, 15, 14, 13,12,11]]
    res=0
    for j in range(1,21):
        if data > cal[ind][j]:
            #print "debug",ind, data ,j, cal[ind][j]
            res=mapf(data,cal[ind][j],cal[ind][j-1],j*10,(j-1)*10)
            return res
        res=mapf(data,cal[ind][20],0,200,1000)
    return res


for val in range(250):
    print(val,"",end="")
    for i in range(3):
        print( leerIRs(i,val),"",end="")
    print("")


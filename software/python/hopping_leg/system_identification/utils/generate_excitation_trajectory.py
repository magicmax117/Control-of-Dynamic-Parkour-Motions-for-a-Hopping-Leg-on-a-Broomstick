from __future__ import division
import numpy as np
from optexgen import optexgen
import csv
import time
import os
import sys

coefsHint = None

if len(sys.argv) > 1:
    try:
        coefsHint = np.genfromtxt(sys.argv[1])
    except:
        print("Could not parse coefficients file '"+sys.argv[1]+"'. Exiting.")
        sys.exit()


id = time.strftime('%Y-%m-%d-%H-%M-%S')
workdir = "data/twolink-"+id

L1 = 0.3
L2 = 0.2
#{-1.009350e+00,  6.444832e-03,  1.104934e+00,  1.739857e+00, -4.270800e-01, -3.344773e-03,  4.863526e-01};
DHP=[{'theta':0,
                   'd':0,
                   'a':L1,
                   'alpha':0*np.pi
              }]
DHP+=[{'theta':np.pi,
                   'd':0,
                   'a':L2,
                   'alpha':0
              }]
g0 = np.array([-9.81,0,0])

dh_params = []
for k in range(len(DHP)):
    dh_params += [[DHP[k]['theta'],DHP[k]['d'],DHP[k]['a'],DHP[k]['alpha']]]
    print("{{ {:7.4e}, {:7.4e}, {:7.4e}, {:7.4e} }}".format(*dh_params[-1]))

dh_params = np.array(dh_params)

n = len(dh_params)

lims = np.array([[- np.inf, np.inf],[- np.inf, np.inf]])

q_lim_fully_rotatable = [-3.0, 3.0]

q_lims = np.array( 
    [
        q_lim_fully_rotatable,
        q_lim_fully_rotatable
    ]
)

dq_lims = np.array([[-8.0, 8.0],[-8.0, 8.0]])

# (12-sin(pi/4)*0.4-1)/0.15 = 71,44771525

ddq_lims = np.array([[-80.0,80.0],[-80.0,80.0]])

#ee_lim_links = np.array([6, 5])

#x_lims= np.array(
    #[[-1.0, 1.0], [-1.0, 1.0]]
#)
#y_lims= np.array(
    #[[-1.0, 1.0], [-1.0, 1.0]]
#)
#z_lims= np.array(
    #[[0.250, 2.0], [0.25, 2.0]]
#)

q_lims = q_lims * 1.00
dq_lims = dq_lims * 1.00

Tp = 5.0
Ts = 1.0/3.0/100.0
nharm = 5

coefs = np.random.rand((1+nharm*2)*dh_params.shape[0])

if coefsHint is not None:
    nharm = (len(coefsHint) / dh_params.shape[0] - 1) / 2
    if nharm < 1:
        print("Failed to use guess number of harmonics from supplied file.")
        sys.exit()
        
    coefs = np.array(coefsHint, copy=True)
    
    print("Using parameter hint from supplied file with "+ str(nharm) + " harmonics")

exd = optexgen.ExperimentOptimization("", dh_params, g0, Ts = Ts*25, periodLength = Tp, numHarmonics = nharm)
exd.setJointLimits(q_lims, dq_lims, ddq_lims)

#exd.setCartesianLimits(ee_lim_links, x_lims, y_lims, z_lims)

exd.setCoefficients(coefs)

exd.optimize()

coefs = exd.getCoefficients()
print(coefs)

if not os.path.exists(workdir):
    os.makedirs(workdir)
    
with open(workdir+"/fourier_coefs.txt", 'w') as f:
    for c in coefs:
        f.write("{0: 15.6e}\n".format(c))


exd.setTrajectorySettings(Ts*10, Tp, nharm)
#exd.setCartesianLimits(ee_lim_links, x_lims, y_lims, z_lims)
exd.optimize()

coefs = exd.getCoefficients()
print(coefs)

exd.setTrajectorySettings(Ts, Tp, nharm)
exd.setCoefficients(coefs)

#print(exd.getCoefficients())

with open(workdir+"/fourier_coefs.txt", 'w') as f:
    for c in coefs:
        f.write("{0: 15.6e}\n".format(c))
        

def write_csv_data(msrDat, fileName='trajectory-pos.csv'):        
    header = []
    
    for (key, val) in msrDat.items():
        if isinstance(val[0], list):
            header += map(lambda x: str(key) + str(x), range(1,len(val[0])+1))
        else:
            header += [key]
            
    outfile = open(fileName, 'w')
    writer = csv.writer(outfile)
    writer.writerow(header)
        
    for msrSample in range(len(list(msrDat.values())[0])):
        row = []
        for (key, val) in msrDat.items():
            if isinstance(val[0], list):
                row += val[msrSample]
            else:
                row += [val[msrSample]]
        writer.writerow(row)
    
    outfile.flush()
    outfile.close()

def q_smooth(x,T_p):
    a = 0 
    b = T_p; 
    c = -2/3.0 
    d = 0.0
    v0 = -2*b/(3*c)
    return (a*x+b*x**2+c*x**3+d*x**4)/(a*v0+b*v0**2+c*v0**3+d*v0**4)


def composeTrajectory(exd, num_periods):
    """ calculates the robot program with fade-In and fade-out
    q is a matrix where the rows are joints and the columns are the time
    """

    (q, dq, ddq) = exd.getTrajectory()
    
    #T_p = exd.exdata.tdata.Tp
    
    spp = q.shape[1];
    t = np.linspace(0.0, Tp, spp);

    fade = q_smooth(t, Tp);
    q_init = 0; 
    
    q = np.tile(q, (1, num_periods));  
    dq = np.tile(dq, (1, num_periods));  
    ddq = np.tile(ddq, (1, num_periods));  
    
    for i in range(q.shape[0]):
        q[i,0:spp] = q[i,0:spp]*fade + q_init*(1-fade)
        q[i,-spp:] = q[i,-spp:]*(1-fade) + q_init*fade
        
               
        dq[i,:] = np.gradient(q[i,:])/Ts     
        ddq[i,:] = np.gradient(dq[i,:])/Ts 

    t = np.arange(spp*num_periods) * Ts
    #np.linspace(0.0, Tp*num_periods, spp*num_periods)
    
    return (t,q,dq,ddq)

    

    
    
#def writeToFile(ts,values,filename):
  #csvfile = csv.writer(open(filename, "wb"),dialect='excel')
  #headrow=["time"]
  #for k in range(values.shape[0]):
      #headrow.append("q"+ str(k))
  #csvfile.writerow(headrow)
  #for i in range(len(ts)):
    #row=[ts[i]]
    #for k in range(values.shape[0]):
      #row.append(values[k,i])
    #csvfile.writerow(row)
    

exd.setTrajectorySettings(Ts*0.2, Tp, nharm)
(t, q_avg, dq_avg, ddq_avg) = composeTrajectory(exd, num_periods=3)
dat = {"time": t, "q":q_avg.T.tolist(), "dq":dq_avg.T.tolist(), "ddq":ddq_avg.T.tolist()}
write_csv_data(dat, fileName=workdir+'/trajectory-pos-20.csv')

exd.setTrajectorySettings(Ts*0.5, Tp, nharm)
(t, q_avg, dq_avg, ddq_avg) = composeTrajectory(exd, num_periods=3)
dat = {"time": t, "q":q_avg.T.tolist(), "dq":dq_avg.T.tolist(), "ddq":ddq_avg.T.tolist()}
write_csv_data(dat, fileName=workdir+'/trajectory-pos-50.csv')

exd.setTrajectorySettings(Ts, Tp, nharm)
(t, q_avg, dq_avg, ddq_avg) = composeTrajectory(exd, num_periods=3)
dat = {"time": t, "q":q_avg.T.tolist(), "dq":dq_avg.T.tolist(), "ddq":ddq_avg.T.tolist()}
write_csv_data(dat, fileName=workdir+'/trajectory-pos-100.csv')

(t, q_avg, dq_avg, ddq_avg) = composeTrajectory(exd, num_periods=12)
dat = {"time": t, "q":q_avg.T.tolist(), "dq":dq_avg.T.tolist(), "ddq":ddq_avg.T.tolist()}
write_csv_data(dat, fileName=workdir+'/trajectory-pos.csv')


import pylab as p
p.subplot(4,1,1)
p.plot(t, q_avg.T)
p.legend(('q1_avg', 'q2_avg'))
p.subplot(4,1,2)
p.plot(t, dq_avg.T)
p.legend(('dq1_avg', 'dq2_avg'))
p.subplot(4,1,3)
p.plot(t, ddq_avg.T)
p.legend(('ddq1_avg', 'ddq2_avg'))
#p.subplot(4,1,4)
#p.plot(tau_avg.T)
#p.legend(('tau1_avg', 'tau2_avg'))
p.show()

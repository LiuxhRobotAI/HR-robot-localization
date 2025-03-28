import numpy as np
import matplotlib.pyplot as plt
import random
import sys
from scipy import stats
from scipy.special import gamma
from filtered_distance import filtered_distance

def load_data():
    if len(sys.argv) < 2 :
        filename = '2023-09-11-17-58-14_circle_typical_0.0_0.0_0.8'
    else:
        filename =sys.argv[1]
    filepath = '/home/liu/workspace/ws/data/jackal_bag/rosbag_restore_xh/uwb_dis/'
    txtfile = filepath + filename +'.txt'

    with open(txtfile, "r", encoding='utf-8') as file :
        anchor_data = [i[:-1].split(' ') for i in file.readlines()]

    uwb_dis_all =[]
    num_ref = 20
    ref_dis = np.zeros(num_ref)
    for i in range(len(anchor_data)):
        uwb_dis_all.append(float(anchor_data[i][3]))

    uwb_dis = uwb_dis_all[0:]
    fit_alpha, fit_loc, fit_beta = stats.gamma.fit(uwb_dis)
    print('uwb: fit_alpha={:.4f} fit_loc={:.4f} fit_beta={:.4f}'.format(fit_alpha,fit_loc, fit_beta))

    return uwb_dis

class pfilter_dis():
    """ class that implements a not-yet-full buffer """
    def __init__(self):
        self.anchors_coordinate = {}

class uwb_particle_filter():
    def __init__(self, particleN, x0, P0, Q, R, dt,option=None):

        self.N = particleN
        self.x = x0
        self.Q = Q
        self.R = R
        self.option = option
        self.dt = dt

        # Initial the particles using the initial state, and assumpe Guanssian distribution
        self.particles = np.random.normal(x0, P0, particleN)
        self.weights = np.ones(particleN) / particleN

    def update_estimate(self, z):

        # Predition: Update particles according to System Transfer Function(STF)
        self.particles = self.particles + self.dt * self.option['v_max'] * np.random.normal(0, self.Q, self.N)
        # self.particles = x_P_update
        # calculate the particle's value for weight calculation of the particles
        z_update = self.particles + np.random.normal(0,self.R)

        self.weights = (1 / np.sqrt(2 * np.pi * self.R)) * np.exp(-(z - z_update) ** 2 / (2 * self.R))
        self.weights /= np.sum(self.weights) # weight normalization

        mean = np.average(self.particles,weights=self.weights)
        var = np.average((self.particles - mean) ** 2, weights=self.weights)

        return mean, var, self.particles

    def resample(self):
        N = len(self.particles)
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1.0
        indexes = np.zeros(N,'i')

        # # # resample methods
        # ## Multinomial, residual resampling; recommend considering stratified resampling and systrmatic resampling
        if self.option['methods']=='simple' or self.option['methods']=='multinomial':
            rn = np.random.rand(N)
            indexes = np.searchsorted(cumulative_sum, rn)

        if None==self.option['methods'] or self.option['methods']=='residual':
            num_copies = (N*np.asarray(self.weights)).astype(int)
            k = 0
            for i in range(N):
                for _ in range(num_copies[i]):
                    indexes[k] = i
                    k += 1
            residual = N*np.asarray(self.weights) - num_copies
            residual /= sum(residual)
            cumulative_sum = np.cumsum(residual)
            cumulative_sum[-1] = 1. # ensures sum is exactly one
            indexes[k:N] = np.searchsorted(cumulative_sum, np.random.rand(N-k))

        if self.option['methods']=='stratified':
            # Stratified resampling
            positions =(np.random.rand(N)+range(N))/N
            indexes = np.zeros(N,'i')
            i,j = 0,0
            while i <N:
                if positions[i]<cumulative_sum[j]:
                    indexes[i]=j
                    i+=1
                else:
                    j+=1

        if self.option['methods']=='systrmatic':
            # Systrmatic resampling
            positions =(np.random.rand(N)+np.random.rand())/N
            indexes = np.zeros(N,'i')
            i,j = 0,0
            while i <N:
                if positions[i]<cumulative_sum[j]:
                    indexes[i]=j
                    i+=1
                else:
                    j+=1

        self.particles[:] = self.particles[indexes]
        self.weights.fill(1.0 / N)

def pfilter():
    uwb_dis = load_data() # uwb_dis

    x0=[uwb_dis[0]] # Initial state

    dt = 0.1 # data sampling time
    P0= 0.1 # Initial Variance matrix for state (vector,[[0.1]])
    x_Q = 1 # Standard deviation of system noisenp. np.std(random.normal(0, x_Q, N))
    z_R = 0.035 # Standard deviation of measurement noise

    N = 100 # number of particles

    x_est_out=[]
    x_P_out=[]
    pf = uwb_particle_filter(particleN=N, x0=x0, P0=P0, Q=x_Q, R=z_R, dt=dt,
                             option={'methods':None, 'v_max':0.2,'alpha':2,'lamda':50,'cte':0.03,})
    for i in range(len(uwb_dis)):
        x_est, _, x_particles = pf.update_estimate(uwb_dis[i])
        pf.resample()

        x_est_out.append(x_est)
        x_P_out.append(x_particles)

    # Save data to show particle trojectory, true values and estimation values
    t = np.arange(0, len(uwb_dis))
    x_P_out = np.array(x_P_out)
    z_out = uwb_dis

    # plot
    ishow =0
    for i in range(0, N):
        if 0 == i:
            plt.plot(t[ishow:], x_P_out[:, i][ishow:], color='gray', label='particle value')
        plt.plot(t[ishow:], x_P_out[:, i][ishow:], color='gray')

    plt.plot(t[ishow:], z_out[ishow:], color='lime', linewidth=2, label='true measurement value')
    plt.plot(t[ishow:], x_est_out[ishow:], color='red', linewidth=2, label='pf estimate value')
    plt.legend()
    plt.show()

# particle filter
def estimate(particles, weights):
    mean = np.average(particles, weights=weights) # x_est = E(x_k|y_k) = weight mean(particles)
    var = np.average((particles - mean) ** 2, weights=weights)
    return mean, var

def resample(particles, weights, methods:str=None):
    N = len(particles)
    cumulative_sum = np.cumsum(weights) # probability cumulative sum of weight
    cumulative_sum[-1] = 1.0 # to avoid round-off error, let the final element be 1.
    indexes = np.zeros(N,'i')

    # # # resample methods
    # ## Multinomial, residual resampling; recommend considering stratified resampling and systrmatic resampling
    if methods=='simple' or methods=='multinomial':
        # Multinomial resampling (easy understand but with bad performance, should not be used)
        rn = np.random.rand(N) # random samples
        indexes = np.searchsorted(cumulative_sum, rn)

    if None==methods or methods=='residual':
        # Residual resampling
        # take int(N*w) copies of each weight,
        num_copies = (N*np.asarray(weights)).astype(int)
        k = 0
        for i in range(N):
            for _ in range(num_copies[i]): # make n copies
                indexes[k] = i
                k += 1
        # use multinormial resample on the residual to fill up the rest.
        residual = N*np.asarray(weights) - num_copies  # get fractional part; residual = N*w-ronud_off(N*w)
        residual /= sum(residual)     # normalize
        cumulative_sum = np.cumsum(residual)
        cumulative_sum[-1] = 1. # ensures sum is exactly one
        indexes[k:N] = np.searchsorted(cumulative_sum, np.random.rand(N-k))

    if methods=='stratified':
        # Stratified resampling
        # make N subdivisions, chose a random position within each one
        positions =(np.random.rand(N)+range(N))/N
        indexes = np.zeros(N,'i')
        i,j = 0,0
        while i <N:
            if positions[i]<cumulative_sum[j]:
                indexes[i]=j
                i+=1
            else:
                j+=1

    if methods=='systrmatic':
        # Systrmatic resampling
        # make N subdivisions, choose positions with a consistent random offset
        positions =(np.random.rand(N)+np.random.rand())/N
        indexes = np.zeros(N,'i')
        i,j = 0,0
        while i <N:
            if positions[i]<cumulative_sum[j]:
                indexes[i]=j
                i+=1
            else:
                j+=1

    # Sampling based on index；Or using histc() directly
    particles[:] = particles[indexes]
    weights.fill(1.0 / N)
    return particles, weights

def pf_example():
    uwb_dis = load_data() # uwb_dis

    x = np.random.normal(0,1)  # initial true value
    x = uwb_dis[0]
    t = len(uwb_dis)/10
    T = len(uwb_dis) # 共进行50次, number of measurements
    dt = t/T

    x_Q = 1
    z_R = 0.03**2
    N = 300

    sigma0 = 0.1
    x_P = x + np.random.normal(0,sigma0,N)
    x_P_out = [x_P]

    v_max = 0.1

    # gamma distribution:
    alpha=2.87
    loc=1.7737
    beta=0.02
    lamda = 1/beta
    # create variables
    z_out = [x + np.random.normal(0,z_R) ]

    x_out = [x]
    x_est = x
    x_est_out = [x_est]
    P_w = np.ones(N) / N

    for t in range(1, T):

        z = uwb_dis[t]

        # Update particles
        x_P_update = x_P + dt * v_max * np.random.normal(0,x_Q,N)

        z_update = x_P_update + np.random.normal(0,z_R)

        P_w = (1 / np.sqrt(2 * np.pi * z_R)) * np.exp(-(z - z_update) ** 2 / (2 * z_R)) + \
              lamda*np.exp(-lamda*x)*((lamda*x)**(alpha-1))/(gamma(alpha))*dt

        P_w /= np.sum(P_w)

        x_est, var = estimate(x_P_update, P_w)

        # resample
        x_P, P_w = resample(x_P_update, P_w,methods='simple') # stratified,simple,systrmatic

        # save data for plot
        x_out.append(x)
        z_out.append(z)
        x_est_out.append(x_est)
        x_P_out.append(x_P)

    # Show particle trojectory, true values and estimation values
    t = np.arange(0, T)
    x_P_out = np.array(x_P_out)

    # plot
    ishow =0
    for i in range(0, N):
        if 0 == i:
            plt.plot(t[ishow:], x_P_out[:, i][ishow:], color='gray', label='particle value')
        plt.plot(t[ishow:], x_P_out[:, i][ishow:], color='gray')

    # plt.plot(t, x_out, color='lime', linewidth=2, label='true value')
    plt.plot(t[ishow:], z_out[ishow:], color='lime', linewidth=2, label='true measurement value')
    plt.plot(t[ishow:], x_est_out[ishow:], color='red', linewidth=2, label='pf estimate value')
    plt.legend()
    plt.show()

if __name__ == '__main__':

    a = 2
    mean, var, skew, kurt = stats.gamma.stats(a, moments='mvsk')
    x = np.linspace(stats.gamma.ppf(0.01,a), stats.gamma.ppf(0.99,a),100)
    rv = stats.gamma(a)

    pfilter()
    # pf_example()
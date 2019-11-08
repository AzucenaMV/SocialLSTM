import numpy as np
import matplotlib.pyplot as plt
'''
print(1)
class System():
    def __init__(self, obs = 100, t_0 = 0, t_f = 100):
​
        self.obs = obs
        self.t_0 = t_0
        self.t_f = t_f
        
        self.posx = np.sin(np.linspace(self.t_0,self.t_f,self.obs+1))
        self.velx = np.cumsum(self.posx)
        self.posy = np.cos(np.linspace(self.t_0,self.t_f,obs+1))
        self.vely = np.cumsum(self.posy)
        
        self.x = np.array([[self.posx],
                           [self.velx],
                           [self.posy],
                           [self.vely]])
​
        self.xdim = np.shape(self.x)
        pass
​
    def params(self, T = (t_f-t_0)/obs, sigx = 0.1**2, sigy = 0.1**2, lambdar = 0.3**2):
        F = np.array([[1, T],
                           [0, 1]])
        fdim = np.shape(F)
        
        sigma = np.array([[T**3/3, T**2/2],
                          [T**2/2, T]])
        
        self.sigmax = sigx * sigma
        self.sigmay = sigy * sigma
​
        zeros = np.zeros(fshape)
​
        #model params
        self.A = np.block([[F, zeros],
                          [zeros, F]])
        self.P = np.block([[sigmax, zeros],
                           [zeros, sigmay]])
​
        #measure params
        self.H = np.array([[1,0,0,0],
                           [0,0,1,0]])
        self.R = lambdar * np.eye(fdim[0]) 
​
        return self.A, self.P, self.H, self.R
print(2)
print(3)
    #def model(self):
'''     
class MotionModel():
    def __init__(self, A, B, Q):
        self.A = A
        self.B = B
        self.Q = Q
        
        #use when redefining the world
        '''
        T = 1
        F1 = np.array([[1, T],
                       [0, 1]])
        F2 = np.array([[1, T],
                       [0, 1]])
        zeros_2 = np.zeros((2, 2))
        self.A = np.block([[F1, zeros_2],
                      [zeros_2, F2]])
        #Not always true, ensure the dynamics are the same
        self.B = self.A
        '''
        (m, _) = Q.shape
        self.zero_mean = np.zeros(m)
​
    def __call__(self, x, u):
        new_state = self.A @ x + self.B @ u + np.random.multivariate_normal(self.zero_mean, self.Q)
        return new_state
​
​
class MeasurementModel():
    def __init__(self, H, R):
        self.H = H
        self.R = R
​
        (n, _) = R.shape
        self.zero_mean = np.zeros(n)
​
    def __call__(self, x):
        measurement = self.H @ x + np.random.multivariate_normal(self.zero_mean, self.R)
        return measurement
​
​
def create_model_parameters(T=1, s2_x=0.1 ** 2, s2_y=0.1 ** 2, lambda2=0.3 ** 2):
    # Motion model parameters
    F1 = np.array([[1, T],
                   [0, 1]])
    F2 = np.array([[1, T],
                   [0, 1]])
    
    G1 = np.array([[1, T],
                   [0, 1]])
    G2 = np.array([[1, T],
                   [0, 1]])
​
    base_sigma = np.array([[T ** 3 / 3, T ** 2 / 2],
                           [T ** 2 / 2, T]])
​
    sigma_x = s2_x * base_sigma
    sigma_y = s2_y * base_sigma
​
    zeros_2 = np.zeros((2, 2))
    #transformation matrix
    A = np.block([[F1, zeros_2],
                  [zeros_2, F2]])
    #control matrix
    B = np.block([[G1, zeros_2],
                  [zeros_2, G2]])
    B = A #not always true ensure the dynammics are the same
    #movement cov
    Q = np.block([[sigma_x, zeros_2],
                  [zeros_2, sigma_y]])
​
    # Measurement model parameters
    #defines which variables are included in the sensor data
    H = np.array([[1, 0, 0, 0],
                  [0, 0, 1, 0]])
    #sensor uncertainty
    R = lambda2 * np.eye(2)
​
    return A, B, H, Q, R
​
​
def simulate_system(K, x0, u):
    (A, B, H, Q, R) = create_model_parameters()
​
    # Create models
    
    motion_model = MotionModel(A, B, Q)
    meas_model = MeasurementModel(H, R)
​
    (m, _) = Q.shape
    (n, _) = R.shape
​
    state = np.zeros((K, m))
    meas = np.zeros((K, n))
​
    # initial state
    x = x0
    for k in range(K):
        x = motion_model(x, u[k])
        z = meas_model(x)
​
        state[k, :] = x
        meas[k, :] = z #add systematic error to sensor data /1.02 - 5
        pass
    
    #adding point errors to sensor data
#    choice = np.random.choice(50,15)
#    meas[choice, :] = meas[choice, :]*[0.8, 0.5]
    return state, meas
​
class KalmanFilter():
    def __init__(self, A, B, H, Q, R, x_0, P_0):
        
        self.A = A
        self.B = B
        self.H = H
        self.Q = Q
        self.R = R
        
        self.x = x_0
        self.P = P_0
        pass
    
    def predict(self, u):
        #update based on transition matrix and environment unvertainty
        self.x = self.A @ self.x + self.B @ u
        self.P = self.A @ self.P @ self.A.T + self.Q
        pass
​
    def update(self, z):
        #predicted measurement covariance
        self.S = self.H @ self.P @ self.H.T + self.R
        #sensor value - the predicted value pre-update
        self.V = z - self.H @ self.x
        #kalman gain
        self.K = self.P @ self.H.T @ np.linalg.inv(self.S)
​
        #update base on input
        self.x = self.x + self.K @ self.V
        self.P = self.P + self.K @ self.S @ self.K.T
        pass
​
    def state(self):
        return(self.x, self.P)
    pass
​
np.random.seed(21)
(A, B, H, Q, R) = create_model_parameters()
K = 50
# initial state
x = np.array([0, 1, 0, 1])
P = 0 * np.eye(4)
​
#controls
u = np.zeros([K,4])
u += np.array([-8, 0.5, 10, -0.5])
inter = round(K*0.6)
u[inter:K, :] = [5, -2, -3, 5]
​
meas = state
(state, meas) = simulate_system(K, x, u)
kalman_filter = KalmanFilter(A, B, H, Q, R, x, P)
​
est_state = np.zeros((K, 4))
est_cov = np.zeros((K, 4, 4))
​
for k in range(K):
    kalman_filter.predict(u[k, :])
    kalman_filter.update(meas[k, :])
    (x, P) = kalman_filter.state()
    
    est_state[k, :] = x
    est_cov[k, ...] = P
    pass
​
#print(est_state)
​
graph = 'Good Fit'
name = 'goodfit'
plt.plot(state[:,0], state[:,2])
plt.plot(est_state[:,0], est_state[:,2], '.')
plt.suptitle('Position')
plt.title(graph)
plt.xlabel('X1')
plt.ylabel('X2')
plt.savefig('position_'+ name +'.png')
plt.close()
​
plt.plot(state[:,1], state[:,3])
plt.plot(est_state[:,1], est_state[:,3], '.')
plt.suptitle('Velocity')
plt.title(graph)
plt.xlabel('X1')
plt.ylabel('X2')
plt.savefig('velocity_'+ name +'.png')
plt.close()
​
​
f, (ax1, ax2) = plt.subplots(1, 2, sharey=False)
f.suptitle(graph)
ax1.plot(state[:,0], state[:,2], '-', est_state[:,0], est_state[:,2], '.')
ax1.set_title('Position')
ax1.set_xlabel('X1')
ax1.set_ylabel('X2')
​
ax2.plot(state[:,1], state[:,3], '-', est_state[:,1], est_state[:,3], '.')
ax2.set_title('Velocity')
ax2.set_xlabel('X1')
​
plt.savefig(name + '.png')
plt.close()
print('done')

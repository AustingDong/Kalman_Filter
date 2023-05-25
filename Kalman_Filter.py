import numpy as np
import scipy

class KalmanFilter(object):

    def __init__(self):
        '''
        dimension: 6
            x, y, z, dx, dy, dz

        dt: 1
            update after 1 frame

        matrices:
            A: motion matrix
            H: update matrix
        '''

        dimension, dt = 3, 1

        #A:
        self.motion_matrix = np.eye(2*dimension, 2*dimension)

        for i in range(dimension):
            self.motion_matrix[i, dimension+i] = dt
        
        # [1 0 0 1 0 0]
        # [0 1 0 0 1 0]
        # [0 0 1 0 0 1]
        # [0 0 0 1 0 0]
        # [0 0 0 0 1 0]
        # [0 0 0 0 0 1]


        #H:
        self.update_matrix = np.eye(dimension, 2*dimension)



        #uncertainties:
        self._std_weight_position = 1./20
        self._std_weight_velocity = 1./160


    def initiate(self, measurement):
        '''
        input:
            measurement: [x, y, z]

        output:
            mean: [x, y, z, dx, dy, dz]
        '''
        #initiate xk, Pk
        mean_pos = measurement
        mean_vel = np.zeros_like(mean_pos)
        mean = np.r_[mean_pos, mean_vel]

        std = [
            2 * self._std_weight_position * measurement[0],
            2 * self._std_weight_position * measurement[1],
            2 * self._std_weight_position * measurement[2],

            10 * self._std_weight_velocity * measurement[0],
            10 * self._std_weight_velocity * measurement[1],
            10 * self._std_weight_velocity * measurement[2]
        ]

        covariance = np.diag(np.square(std))

        return mean, covariance

    
    def predict(self, mean, covariance):

        std_pos = [
            self._std_weight_position * mean[0],
            self._std_weight_position * mean[1],
            self._std_weight_position * mean[2]
        ]

        std_vel = [
            self._std_weight_velocity * mean[0],
            self._std_weight_velocity * mean[1],
            self._std_weight_velocity * mean[2]
        ]

        motion_cov = np.diag(np.square(np.r_[std_pos, std_vel]))

        mean = np.dot(mean, self.motion_matrix.T) #x^-k

        #P^-k
        covariance = np.linalg.multi_dot((self.motion_matrix, covariance, self.motion_matrix.T)) + motion_cov
        
        return mean, covariance



    def project(self, mean, covariance):
        std = [
            self._std_weight_position * mean[0],
            self._std_weight_position * mean[1],
            self._std_weight_position * mean[2]
        ]

        #Rk
        innovation_cov = np.diag(np.square(std))

        mean = np.dot(self.update_matrix, mean)
        covariance = np.linalg.multi_dot((self.update_matrix, covariance, self.update_matrix.T))

        return mean, covariance + innovation_cov



    def update(self, mean, covariance, measurement):

        projected_mean, projected_cov = self.project(mean, covariance)

        #change: A * K = P-k * Hk 
        chol_factor, lower = scipy.linalg.cho_factor(projected_cov, lower=True, check_finite=False)

        kalman_gain = scipy.linalg.cho_solve((chol_factor, lower), np.dot(covariance, self.update_matrix.T).T, check_finite=False).T

        innovation = measurement - projected_mean

        #   x^  =  x^-  +  K  *  (zk - Hk * x^-k)
        new_mean = mean + np.dot(innovation, kalman_gain.T)

        #   Pk  =  P-k  -  K  *  H  
        new_covariance = covariance - np.linalg.multi_dot((kalman_gain, projected_cov, kalman_gain.T))

        return new_mean, new_covariance



        




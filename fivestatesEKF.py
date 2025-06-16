import numpy as np

class EKF5States:
    def __init__(self, Qd, Rd, Z, h_samp, frame):
        self.Qd = Qd
        self.Rd = Rd
        self.Z = Z
        self.h_samp = h_samp
        self.frame = frame.upper()
        self.I5 = np.eye(5)

        # WGS-84 constants
        self.a = 6378137.0  #  Semi-major axis (equatorial radius)
        self.f = 1 / 298.257223563 # Flattening
        self.e = np.sqrt(2 * self.f - self.f**2) # Earth eccentricity

        self.alpha_1 = 0.01 # Singer constant, speed (CV Constant Velocity model)
        self.alpha_2 = 0.1  # Singer constant, course rate (CA Constant Acceleration model)

        # Internal state
        self.x_prd = None
        self.P_prd = None
        self.count = 0

    def ssa(self, angle_vec):
        # Small-angle wrapping for longitude, if needed
        angle_vec[1] = (angle_vec[1] + np.pi) % (2 * np.pi) - np.pi
        return angle_vec

    def step(self, GNSS1, GNSS2):
        '''
        INPUT:
        GNSS1,GNSS2: North-East pos. (m) or Latitude-Longitude (rad)
        h_samp: EKF sampling time (s)
        Z: h_samp * GNSS measurement rate (Hz) (must be integer)
        frame: 'NED' (North-East) or 'LL' (Latitude-Longitude)
        Qd: EKF 2x2 process cov. matrix for speed and course rate
        Rd: EKF 2x2 GNSS measurement cov. matrix

        OUTPUT:
        [x,y.U,chi,omega_chi] = EKF_5states(xpos,ypos,h,Z,'NED',Qd,Rd)
        [mu.l,U,chi omega_chi] = EKF_5states(mu,l,h,Z,'LL',Qd,Rd)


        '''

        Cd = np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0]
        ])
        Ed = self.h_samp * np.array([
            [0, 0],
            [0, 0],
            [1, 0],
            [0, 0],
            [0, 1]
        ])

        # Initialization
        if self.x_prd is None:
            print("Init EKF states")
            self.x_prd = np.array([GNSS1, GNSS2, 0, 0, 0], dtype=float)
            self.P_prd = self.I5.copy()
            self.count = 1

        if self.count == 1: # Update if new measurement
            y = np.array([GNSS1, # y1 = x^n or y1 = latitude
                          GNSS2]) # y2 = y^n or y2 = longitude

            K = self.P_prd @ Cd.T @ np.linalg.inv(Cd @ self.P_prd @ Cd.T + self.Rd) # KF gain
            IKC = self.I5 - K @ Cd
            P_hat = IKC @ self.P_prd @ IKC.T + K @ self.Rd @ K.T # Corrector

            eps = y - Cd @ self.x_prd
            if self.frame == 'LL':
                eps = self.ssa(eps)

            x_hat = self.x_prd + K @ eps
            self.count = self.Z
        else:                   # No update
            x_hat = self.x_prd
            P_hat = self.P_prd
            self.count -= 1

        # Compute prediction model
        if self.frame == 'NED': # x = [ x^n y^n U chi omega_chi ]'
            f = np.array([
                x_hat[2] * np.cos(x_hat[3]),
                x_hat[2] * np.sin(x_hat[3]),
                -self.alpha_1 * x_hat[2],
                x_hat[4],
                -self.alpha_2 * x_hat[4]
            ])

            Ad = self.I5 + self.h_samp * np.array([
                [0, 0, np.cos(x_hat[3]), -x_hat[2] * np.sin(x_hat[3]), 0],
                [0, 0, np.sin(x_hat[3]),  x_hat[2] * np.cos(x_hat[3]), 0],
                [0, 0, -self.alpha_1, 0, 0],
                [0, 0, 0, 0, 1],
                [0, 0, 0, 0, -self.alpha_2]
            ])
        elif self.frame == 'LL': # x = [ mu l U chi omega_chi ]'
            Rn = self.a / np.sqrt(1 - self.e**2 * np.sin(x_hat[0])**2)
            Rm = Rn * ((1 - self.e**2) / (1 - self.e**2 * np.sin(x_hat[0])**2))

            f = np.array([
                (1 / Rm) * x_hat[2] * np.cos(x_hat[3]),
                (1 / (Rn * np.cos(x_hat[0]))) * x_hat[2] * np.sin(x_hat[3]),
                -self.alpha_1 * x_hat[2],
                x_hat[4],
                -self.alpha_2 * x_hat[4]
            ])

            Ad = self.I5 + self.h_samp * np.array([
                [0, 0, (1/Rm) * np.cos(x_hat[3]), -(1/Rm) * x_hat[2] * np.sin(x_hat[3]), 0],
                [np.tan(x_hat[0])/(Rn * np.cos(x_hat[0])) * x_hat[2] * np.sin(x_hat[3]),
                 0,
                 (1/(Rn * np.cos(x_hat[0]))) * np.sin(x_hat[3]),
                 (1/(Rn * np.cos(x_hat[0]))) * x_hat[2] * np.cos(x_hat[3]),
                 0],
                [0, 0, -self.alpha_1, 0, 0],
                [0, 0, 0, 0, 1],
                [0, 0, 0, 0, -self.alpha_2]
            ])
        else:
            raise ValueError("Invalid frame; must be 'NED' or 'LL'")

        # Predictor step (k+1)
        self.x_prd = x_hat + self.h_samp * f
        self.P_prd = Ad @ P_hat @ Ad.T + Ed @ self.Qd @ Ed.T

        return self.x_prd

# === Example usage ===
if __name__ == '__main__':

    # Example parameters
    h_samp = 1/50  # Sample time (s) corresponding to 50 Hz
    Z = 10 #  GNSS measurement 10 times slower (5 Hz)
    Qd = np.diag([1E7, 1E3])  # Process co-variance matrix: speed/course rate
    Rd = np.diag([1E-8, 1E-8])    # GNSS measurement co-variance matrix
    

    ekf = EKF5States(Qd, Rd, Z, h_samp, 'LL')

    while True:
        lat = measureLatitude()   # Replace with actual data acquisition
        lon = measureLongitude()  # Replace with actual data acquisition
        state_estimate = ekf.step(lat, lon)
        print(state_estimate)

import numpy as np
import scipy.linalg as linalg

def calculate_lqr():
    
    g = 9.81     
    M = 2.9      
    m = 0.2      
    L = 0.18      
    r = 0.06      
    A = np.array([
        [0, 1, 0, 0],
        [0, 0, -M * g / M, 0], 
        [0, 0, 0, 1],
        [0, 0, (M + m) * g / (M * L), 0]
    ])
    B = np.array([
        [0],
        [1 / M],
        [0],
        [-1 / (M * L)]
    ])
    
    Q = np.diag([1.0, 1.0, 100.0, 10.0]) 
    R = np.array([[0.1]])
    P = linalg.solve_continuous_are(A, B, Q, R)
    K = linalg.inv(R).dot(B.T).dot(P)
    K = K.flatten() 
    K_formatted = [
        -abs(K[2]),  
        -abs(K[3]),  
        -abs(K[0]), 
        -abs(K[1]) 
    ]
    
    print("=========================================")
    print("MA TRẬN LQR_K TỐI ƯU CỦA BẠN LÀ:")
    print(f"[{K_formatted[0]:.4f}, {K_formatted[1]:.4f}, {K_formatted[2]:.4f}, {K_formatted[3]:.4f}]")
    print("=========================================")
    print("Hãy copy mảng trên và dán đè vào biến LQR_K trong file robot_lqr.py")

if __name__ == "__main__":
    calculate_lqr()
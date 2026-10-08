import numpy as np
import scipy.linalg as linalg

def calculate_lqr():
    
    g = 9.81     
    M = 14.0      
    m = 2.0      
    L = 0.35      
    r = 0.12      

   
    # 2. XÂY DỰNG MA TRẬN ĐỘNG LỰC HỌC (A, B)
    # Mô hình: Con lắc ngược có bánh xe (Wheeled Inverted Pendulum)
    # Vector trạng thái x = [Vị_trí, Vận_tốc, Góc_nghiêng, Vận_tốc_góc]
    # ==========================================
    
    # Xấp xỉ tuyến tính hóa (Linearization) cho hệ thống Segway cơ bản
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
    
  
    # 3. TINH CHỈNH MA TRẬN TRỌNG SỐ (Q, R)
    # Hàm mục tiêu (Cost function) J = ∫(x^T Q x + u^T R u)dt
    # Ma trận Q: Trừng phạt các trạng thái khi bị sai lệch. 
    # Vị trí thứ 3 (100.0) tương ứng với Góc nghiêng -> Phạt nặng nhất để xe luôn đứng thẳng.
    Q = np.diag([1.0, 1.0, 100.0, 10.0]) 
    # Ma trận R: Trừng phạt nếu dùng lực động cơ (Torque) quá lớn.
    R = np.array([[0.1]])
    # 4. GIẢI PHƯƠNG TRÌNH LQR
    # Sử dụng Scipy để giải phương trình đại số Riccati liên tục (CARE)
    P = linalg.solve_continuous_are(A, B, Q, R)
    
    # Tính ma trận K tối ưu: K = R^-1 * B^T * P
    K = linalg.inv(R).dot(B.T).dot(P)
    K = K.flatten() # Chuyển thành mảng 1 chiều
    
    # Sắp xếp lại thứ tự output cho khớp với biến LQR_K trong file robot_lqr.py
    # Thứ tự mong đợi: [Góc_nghiêng, Vận_tốc_góc, Vị_trí, Vận_tốc_thẳng]
    K_formatted = [
        -K[2],  # Phản hồi cho Góc nghiêng
        -K[3],  # Phản hồi cho Vận tốc góc
        -K[0],  # Phản hồi cho Vị trí
        -K[1]   # Phản hồi cho Vận tốc thẳng
    ]
    
    print("=========================================")
    print("MA TRẬN LQR_K TỐI ƯU CỦA BẠN LÀ:")
    print(f"[{K_formatted[0]:.4f}, {K_formatted[1]:.4f}, {K_formatted[2]:.4f}, {K_formatted[3]:.4f}]")
    print("=========================================")
    print("Hãy copy mảng trên và dán đè vào biến LQR_K trong file robot_lqr.py")

if __name__ == "__main__":
    calculate_lqr()
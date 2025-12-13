# 원래 계수
a = -6.5341e-9
b = 1.5165e-3
c = -0.2048

# 1️⃣ λ = 31000일 때 결과 계산
lambda_1 = 35500
y1 = a * (lambda_1 ** 2) + b * lambda_1 + c

# 2️⃣ λ = 40일 때 y = 40이 되도록 상수항 조정
lambda_ref = 41464.4424
y_target = 53.1762
c_new = y_target - (a * (lambda_ref ** 2) + b * lambda_ref)

# 새 식의 λ=40 검증
y_check = a * (lambda_ref ** 2) + b * lambda_ref + c_new

print(f"λ = {lambda_1} 일 때, 원래 식의 결과 y = {y1:.6f}")
print(f"λ = {lambda_ref} 에서 y_target이 되도록 조정된 새로운 상수항 c = {c_new:.6f}")
print(f"검증: λ={lambda_ref} 대입 시 y = {y_check:.6f}")

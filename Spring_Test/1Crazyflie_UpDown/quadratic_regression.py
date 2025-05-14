import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import re

# 실험 데이터
data_text = """
PWM	Thrust
36328 50.24
36647 50.87
37059 49.19
37309 50.91
37604 49.39
37640 50.72
37646 49.75
38419 51.127
38671 51.66
38798 51.471
38903 51.5
39134 51.78
39231 52.01
39627 55.61
39628 52.08
39827 53.5
40127 55.62
40136 53.58
40163 55.49
40221 54.84
40312 54.87
40403 55.72
40536 55.02
40613 55.38
40765 53.65
40821 53.15
41135 56.26
41414 56.3
41435 56.43
41518 56.44
42098 57.48
42163 57.66
42600 57.85
43213 57.96
43598 58.89
43990 58.89
44051 59.34
44106 60.09
44192 59.33
44256 59.94
44490 59.97
44566 59.86
"""

# 데이터 파싱
lines = data_text.strip().split('\n')[1:]
pwm = []
thrust = []
for line in lines:
    cols = re.split(r'\s+', line)
    pwm.append(int(cols[0]))
    thrust.append(float(cols[1]))

df = pd.DataFrame({'PWM': pwm, 'Thrust_g': thrust})

# 2차 다항식 회귀 피팅
coeffs = np.polyfit(df['PWM'], df['Thrust_g'], 2)
poly_func = np.poly1d(coeffs)
df['Fitted_Thrust'] = poly_func(df['PWM'])

# 2차 다항 회귀 방정식 출력
print("2차 다항 회귀 방정식: Thrust_g = ({:.6e})*PWM^2 + ({:.6e})*PWM + ({:.6e})".format(
    coeffs[0], coeffs[1], coeffs[2]
))

# 그래프
plt.figure(figsize=(10, 6))
plt.scatter(df['PWM'], df['Thrust_g'], label='Measured Data', alpha=0.7)
plt.plot(df['PWM'], df['Fitted_Thrust'], label='Quadratic Fit Curve')
plt.scatter(df['PWM'], df['Fitted_Thrust'], label='Fitted Points', s=20, marker='x')  # 피팅 점 표시

plt.xlabel('PWM')
plt.ylabel('Thrust (g)')
plt.title('PWM to Thrust Mapping (Quadratic model)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression

# Dane z portu szeregowego
pwm_values = np.array([0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230, 240, 250])
motor_1_pulses = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 34, 48, 51, 55, 59, 60, 62, 64, 66, 67, 68, 68, 69, 71, 70, 71, 73])
motor_2_pulses = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 31, 46, 52, 56, 59, 63, 65, 67, 70, 72, 73, 74, 76, 77, 78, 79, 80])

# Dopasowanie modelu regresji liniowej dla Motor 1
model_motor1 = LinearRegression()
model_motor1.fit(pwm_values.reshape(-1, 1), motor_1_pulses)

# Dopasowanie modelu regresji liniowej dla Motor 2
model_motor2 = LinearRegression()
model_motor2.fit(pwm_values.reshape(-1, 1), motor_2_pulses)

# Pobranie współczynników regresji dla Motor 1
a_motor1 = model_motor1.coef_[0]
intercept_motor1 = model_motor1.intercept_
print(f"Motor 1: y = {a_motor1:.2f} * PWM + {intercept_motor1:.2f}")

# Pobranie współczynników regresji dla Motor 2
a_motor2 = model_motor2.coef_[0]
intercept_motor2 = model_motor2.intercept_
print(f"Motor 2: y = {a_motor2:.2f} * PWM + {intercept_motor2:.2f}")

# Predykcja wartości prędkości dla Motor 1 i Motor 2
predicted_motor1 = model_motor1.predict(pwm_values.reshape(-1, 1))
predicted_motor2 = model_motor2.predict(pwm_values.reshape(-1, 1))

# Wizualizacja regresji dla Motor 1 i Motor 2
plt.scatter(pwm_values, motor_1_pulses, color='blue', label='Motor 1 - Dane')
plt.plot(pwm_values, predicted_motor1, color='red', label='Motor 1 - Regresja liniowa')

plt.scatter(pwm_values, motor_2_pulses, color='green', label='Motor 2 - Dane')
plt.plot(pwm_values, predicted_motor2, color='orange', label='Motor 2 - Regresja liniowa')

plt.xlabel('PWM')
plt.ylabel('Impulsy na sekundę')
plt.title('Dopasowanie PWM do prędkości (impulsów na sekundę) dla Motor 1 i Motor 2')
plt.legend()
plt.show()

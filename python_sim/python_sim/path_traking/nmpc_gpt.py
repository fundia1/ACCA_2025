import casadi as ca
import numpy as np


class MPCPathFollower:
    def __init__(
        self,
        N=10,
        dt=0.1,
        max_speed=10.0,
        max_acceleration=3.0,
        max_steering=np.deg2rad(30),
    ):
        """
        MPC 경로 추종 클래스 초기화
        Args:
            N: 예측 시간 구간 (Horizon)
            dt: 시간 간격 (Time step)
            max_speed: 최대 속도 (m/s)
            max_acceleration: 최대 가속도 (m/s^2)
            max_steering: 최대 조향각 (radian)
        """
        self.N = N  # Prediction horizon
        self.dt = dt  # Time step
        self.max_speed = max_speed
        self.max_acceleration = max_acceleration
        self.max_steering = max_steering
        self.wheel_base = 2.5  # 차량의 휠베이스 (차축 간 거리)

        # 차량 상태 변수: [x, y, yaw, v]
        self.x = ca.MX.sym("x")
        self.y = ca.MX.sym("y")
        self.yaw = ca.MX.sym("yaw")
        self.v = ca.MX.sym("v")
        self.states = ca.vertcat(self.x, self.y, self.yaw, self.v)

        # 제어 입력 변수: [steering, acceleration]
        self.steering = ca.MX.sym("steering")
        self.acceleration = ca.MX.sym("acceleration")
        self.controls = ca.vertcat(self.steering, self.acceleration)

        # 차량 동역학 모델 정의
        self.A = self._vehicle_dynamics()

    def _vehicle_dynamics(self):
        """
        차량의 동역학 모델 정의.
        Returns:
            CasADi 함수로 상태 업데이트 반환.
        """
        x_next = self.x + self.v * ca.cos(self.yaw) * self.dt
        y_next = self.y + self.v * ca.sin(self.yaw) * self.dt
        yaw_next = (
            self.yaw + (self.v / self.wheel_base) * ca.tan(self.steering) * self.dt
        )
        v_next = self.v + self.acceleration * self.dt
        return ca.Function(
            "A",
            [self.states, self.controls],
            [ca.vertcat(x_next, y_next, yaw_next, v_next)],
        )

    def optimize(self, initial_state, path_x, path_y):
        """
        MPC 최적화 문제를 해결하여 최적의 조향각과 가속도를 반환.
        """
        # 변수 정의
        X = ca.MX.sym("X", 4, self.N + 1)  # 상태 변수: x, y, yaw, v
        U = ca.MX.sym("U", 2, self.N)  # 제어 입력 변수: steering, acceleration

        # 비용 함수 정의
        cost = 0
        for t in range(self.N):
            # 경로 추종 오차 최소화
            cost += ca.sumsqr(X[0:2, t] - ca.vertcat(path_x[t], path_y[t]))
            # 제어 입력 최소화
            cost += 0.01 * ca.sumsqr(U[:, t])

        # 제약 조건 정의
        constraints = []
        constraints.append(X[:, 0] - initial_state)  # 초기 상태 제약

        for t in range(self.N):
            # 동역학 제약 추가
            x_next = self.A(X[:, t], U[:, t])
            constraints.append(X[:, t + 1] - x_next)

        # 제약 조건 크기 확인 및 간소화
        constraints = ca.vertcat(*constraints)

        # 최적화 문제 정의
        prob = {
            "f": cost,  # 비용 함수
            "x": ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1)),  # 최적화 변수
            "g": constraints,  # 제약 조건
        }

        # IPOPT 솔버 설정
        solver = ca.nlpsol("solver", "ipopt", prob)

        # 초기 값 설정
        x0 = np.zeros((4 * (self.N + 1) + 2 * self.N, 1))
        lbg = np.zeros(constraints.shape[0])  # 제약 조건 하한
        ubg = np.zeros(constraints.shape[0])  # 제약 조건 상한

        # 최적화 실행
        solution = solver(x0=x0, lbg=lbg, ubg=ubg)

        # 결과 추출
        u_opt = solution["x"][4 * (self.N + 1) :].full().flatten()
        steering = u_opt[0]
        acceleration = u_opt[1]
        return steering, acceleration


# 경로 추종 예제
if __name__ == "__main__":
    mpc = MPCPathFollower()

    # 초기 상태 [x, y, yaw, v]
    initial_state = np.array([0.0, 0.0, 0.0, 0.0])

    # 경로 설정
    path_x = [i for i in range(10)]
    path_y = [i * 2 for i in range(10)]

    # MPC 최적화 실행
    steering, acceleration = mpc.optimize(initial_state, path_x, path_y)
    print(f"Steering: {steering}, Acceleration: {acceleration}")

//
// Created by zhl on 6/10/26.
//

#ifndef ACEMA_CTLR_FILTROKALMAN_H
#define ACEMA_CTLR_FILTROKALMAN_H


#include <Eigen/Dense>

template <typename T>
bool es_dato_R1(T dato) {
    // check if dato is not vector

    return true; // TODO: definir como se calcula
}




class KalmanFilter {
private:
    // Matrices del filtro
    Eigen::MatrixXd A, B, H, Q, R, P, I;
    Eigen::VectorXd x;

public:
    KalmanFilter() {}

    // Inicializa las matrices con sus dimensiones
    void init(const Eigen::MatrixXd& A_in, const Eigen::MatrixXd& H_in,
              const Eigen::MatrixXd& Q_in, const Eigen::MatrixXd& R_in,
              const Eigen::MatrixXd& P_in, const Eigen::VectorXd& x_in) {
        A = A_in; H = H_in; Q = Q_in; R = R_in; P = P_in; x = x_in;
        I = Eigen::MatrixXd::Identity(x.size(), x.size());
    }

    // Paso 1: Predicción
    void predict(const Eigen::VectorXd& u) {
        x = A * x; // Si no hay entrada u, se omite B*u
        P = A * P * A.transpose() + Q;
    }

    // Paso 2: Actualización (Corrección con el sensor)
    void update(const Eigen::VectorXd& z) {
        Eigen::MatrixXd S = H * P * H.transpose() + R;
        Eigen::MatrixXd K = P * H.transpose() * S.inverse(); // Ganancia de Kalman
        x = x + K * (z - H * x);
        P = (I - K * H) * P;
    }

    // Devuelve el estado estimado actual
    Eigen::VectorXd getState() const { return x; }
};


#endif //ACEMA_CTLR_FILTROKALMAN_H

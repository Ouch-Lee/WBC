/*
 * created by Jun, 2020-07-02
 */

#include <DiamondKinematics.h>

namespace DiaKine {

//
DK2Vec& DK2Vec::operator= ( const DK2Vec& foo){
    if(this->Q1.size() == 0){
        this->Q1.resize(foo.Q1.size());
    }
    if(this->Q2.size() == 0){
        this->Q2.resize(foo.Q2.size());
    }
    this->Q1 = foo.Q1;
    this->Q2 = foo.Q2;
    return *this;
}

DK3Vec& DK3Vec::operator=(const DK3Vec &foo){
    if(this->Q1.size() == 0){
        this->Q1.resize(foo.Q1.size());
    }
    if(this->Q2.size() == 0){
        this->Q2.resize(foo.Q2.size());
    }
    if(this->Q3.size() == 0){
        this->Q3.resize(foo.Q3.size());
    }
    this->Q1 = foo.Q1;
    this->Q2 = foo.Q2;
    this->Q3 = foo.Q3;
    return *this;
}

DK2Vec fiveBarKine::ikPlanar2bar(const Eigen::Vector2d &endXY, const Eigen::Vector2d &lparamsTwo){
    DK2Vec theta;
    theta.Q1.resize(2);
    theta.Q2.resize(2);
    double x = endXY(0);
    double y = endXY(1);
    double l1 = lparamsTwo(0);
    double l2 = lparamsTwo(1);
    double r, num, den, phi;
    double flag;
    double theta2_up, theta2_down;
    double temp_x_plus, temp_y_plus, psi_plus;
    double temp_x_minus, temp_y_minus, psi_minus;
    double theta1_plus, theta1_minus;

    r = x * x + y * y;
    num = (l1 + l2) * (l1 + l2) - r;
    den = r - (l1 - l2) * (l1 - l2);

    flag = std::atan(std::sqrt(num / den));

    if (flag >= 0) {
        theta2_up = 2. * std::atan(std::sqrt(num / den));
        theta2_down = -2. * std::atan(std::sqrt(num / den));
    } else {
        theta2_up = -2 * std::atan(std::sqrt(num / den));
        theta2_down = 2 * std::atan(std::sqrt(num / den));
    }

    phi = std::atan2(y, x);
    temp_x_plus = l1 + l2 * std::cos(theta2_up);
    temp_y_plus = l2 * std::sin(theta2_up);
    psi_plus = std::atan2(temp_y_plus, temp_x_plus);
    theta1_plus = phi - psi_plus;
    theta.Q1(0) = theta1_plus;   // Q1 elbow down config
    theta.Q1(1) = theta2_up;

    temp_x_minus = l1 + l2 * std::cos(theta2_down);
    temp_y_minus = l2 * std::sin(theta2_down);
    psi_minus = std::atan2(temp_y_minus, temp_x_minus);
    theta1_minus = phi - psi_minus;
    theta.Q2(0) = theta1_minus;  // Q2 elbow up config
    theta.Q2(1) = theta2_down;

    return theta;
}

Eigen::Vector2d fiveBarKine::fkPlanar5bar(const Eigen::Vector2d &theta, const Eigen::Vector3d &lparamsThree){
    Eigen::Vector2d endPos2D;
    double l0 = lparamsThree(0);
    double l1 = lparamsThree(1);
    double l2 = lparamsThree(2);
    double s1 = std::sin(theta(0));
    double c1 = std::cos(theta(0));
    double s2 = std::sin(theta(1));
    double c2 = std::cos(theta(1));
    double c1_2 = std::cos(theta(0) - theta(1));

    double Xc = l1 / 2. * (c1 + c2);
    double Zc = -l1 / 2. * (s1 + s2);

    double H = l0*l0 + 2. * l1*l1 + 2. * l0*l1*(c1 - c2) - 2. * l1*l1*c1_2;
    double K = l2*l2 - 0.5*l1*l1 - 0.25*l0*l0 - 0.5*l0*l1*(c1 - c2) + 0.5*l1*l1*c1_2;

    double sphi = l1*(s1 - s2) / std::sqrt(H);
    double cphi = (l0 + l1*(c1 - c2)) / std::sqrt(H);

    double L_x = sqrt(K);

    endPos2D << Xc - L_x * sphi,
                Zc - L_x * cphi;
    return endPos2D;
}

Eigen::Vector2d fiveBarKine::ikPlanar5bar(const Eigen::Vector2d &endXZ, const Eigen::Vector3d &lparamsThree){
    Eigen::Vector4d thetaFour;
    DK2Vec theta1;
    DK2Vec theta2;
    double l0 = lparamsThree(0);
    double l1 = lparamsThree(1);
    double l2 = lparamsThree(2);

    Eigen::Vector2d ltwo;
    ltwo << l1,
            l2;
    Eigen::Vector2d aPos;
    Eigen::Vector2d mPos;

    aPos << endXZ(0) - l0*0.5,
            -endXZ(1);
    mPos << endXZ(0) + l0*0.5,
            -endXZ(1);

    theta1 = ikPlanar2bar(aPos, ltwo);
    thetaFour(0) = theta1.Q1(0);
    thetaFour(2) = theta1.Q1(1);
    theta2 = ikPlanar2bar(mPos, ltwo);
    thetaFour(1) = theta2.Q2(0);
    thetaFour(3) = theta2.Q2(1);
    return thetaFour.head(2);
}

Eigen::Matrix2d fiveBarKine::jacoPlanar5bar(const Eigen::Vector2d& theta, const Eigen::Vector3d& lparamsThree){
    Eigen::Matrix2d jacobian2D;
    double l0 = lparamsThree(0);
    double l1 = lparamsThree(1);
    double l2 = lparamsThree(2);
    double s1 = std::sin(theta(0));
    double c1 = std::cos(theta(0));
    double s2 = std::sin(theta(1));
    double c2 = std::cos(theta(1));
    double s1_2 = std::sin(theta(0) - theta(1));
    double c1_2 = std::cos(theta(0) - theta(1));
    double H = l0*l0 + 2 * l1*l1 + 2 * l0*l1*(c1 - c2) - 2 * l1*l1 * c1_2;
    double K = l2*l2 - 0.5*l1*l1 - 0.25*l0 * l0 - 0.5*l0*l1*(c1 - c2) + 0.5 * l1*l1 * c1_2;
    double sphi = l1*(s1 - s2) / std::sqrt(H);
    double cphi = (l0 + l1*(c1 - c2)) / std::sqrt(H);
    double L_x = std::sqrt(K);
    double L_BN = std::sqrt(H);
    double J11_1 = -0.5 * l1 * s1;
    double J11_2 = -0.5 * (0.5*l0*l1*s1 - 0.5*l1*l1 * s1_2)*sphi / L_x;
    double J11_3 = -L_x  * (l1*c1*L_BN - 0.5*l1*(s1 - s2)*(-2 * l0*l1*s1 + 2 * l1*l1 * s1_2) / L_BN) / H;
    double J12_1 = -0.5 * l1 * s2;
    double J12_2 = -0.5 * (-0.5*l0*l1*s2 + 0.5*l1*l1 * s1_2)*sphi / L_x;
    double J12_3 = -L_x * (-l1*c2*L_BN - 0.5*l1*(s1 - s2)*(2 * l0*l1*s2 - 2 * l1*l1 * s1_2) / L_BN) / H;
    double J21_1 = -0.5 * l1 * c1;
    double J21_2 = -0.5 * (0.5*l0*l1*s1 - 0.5*l1*l1 * s1_2)*cphi / L_x;
    double J21_3 = -L_x * (-l1*s1*L_BN - 0.5*(l0 + l1 *(c1 - c2))*(-2 * l0*l1*s1 + 2 * l1*l1 * s1_2) / L_BN) / H;
    double J22_1 = -0.5 * l1 * c2;
    double J22_2 = -0.5 * (-0.5*l0*l1*s2 + 0.5*l1*l1 * s1_2)*cphi / L_x;
    double J22_3 = -L_x * (l1*s2*L_BN - 0.5*(l0 + l1 *(c1 - c2))*(2 * l0*l1*s2 - 2 * l1*l1 * s1_2) / L_BN) / H;

    jacobian2D << J11_1 + J11_2 + J11_3, J12_1 + J12_2 + J12_3,
                  J21_1 + J21_2 + J21_3, J22_1 + J22_2 + J22_3;
    return jacobian2D;
}

DiamondLeg3D :: DiamondLeg3D(){
    lparams = Eigen::Vector3d(0.097, 0.16, 0.32);
}

DiamondLeg3D :: DiamondLeg3D(const Eigen::Vector3d& lengthParamsThree){
    lparams = lengthParamsThree;
}

DiamondLeg3D :: ~DiamondLeg3D(){

}

Eigen::Vector3d DiamondLeg3D::cartes2Polar3D(const Eigen::Vector3d &xyz){
    double x = xyz(0);
    double y = xyz(1);
    double z = xyz(2);

    Eigen::Vector3d spherCoord;
    spherCoord(0) = std::sqrt(x*x + y*y + z*z);

    if ((z == 0) && (y == 0)){
        spherCoord(1) = 0.;
        spherCoord(2) = 0.;
    }
    else{
        spherCoord(1) = atanZeroPI(sqrt(y*y+z*z)/x);
        spherCoord(2) = std::atan2(y,-z);
    }
    return spherCoord;
}

Eigen::Vector3d DiamondLeg3D::polar2Cartes3D(const Eigen::Vector3d &rtp){
    Eigen::Vector3d xyz;
    double r = rtp(0);
    double theta = rtp(1);
    double phi = rtp(2);
    xyz << r*std::cos(theta),
           r*std::sin(theta)*std::sin(phi),
           -r*std::sin(theta)*std::cos(phi);
    return xyz;
}

double DiamondLeg3D::atanZeroPI(double x){
    double theta = atan(x);
    if (theta < 0)
        theta = theta + pi;
    return theta;
}

DiamondLeg2D :: DiamondLeg2D(){
    pointer3D = new DiamondLeg3D();
    pointer5bar = new fiveBarKine();
    lparams = Eigen::Vector3d(0.097, 0.16, 0.32);
}


DiamondLeg2D :: DiamondLeg2D(const Eigen::Vector3d& lengthParamsThree){
    pointer3D = new DiamondLeg3D();
    pointer5bar = new fiveBarKine();
    lparams = lengthParamsThree;
}

DiamondLeg2D :: ~DiamondLeg2D(){
    delete pointer3D;
    delete pointer5bar;
}

void DiamondLeg2D :: setLinkParams(const Eigen::Vector3d& lengthParamsThree){
    lparams = lengthParamsThree;
}

Eigen::Vector2d DiamondLeg2D::cartes2Polar2D(const Eigen::Vector2d &xz){
    Eigen::Vector2d rt;
    Eigen::Vector3d xyz;
    Eigen::Vector3d rtp;
    xyz << xz(0),
           0.,
           xz(1);
    rtp = pointer3D->cartes2Polar3D(xyz);
    rt << rtp(0),
          rtp(1);
    return rt;
}

Eigen::Vector2d DiamondLeg2D::polar2Cartes2D(const Eigen::Vector2d &rt){
    Eigen::Vector3d rtp;
    Eigen::Vector3d xyz;
    Eigen::Vector2d xz;
    rtp << rt(0),
           rt(1),
           0.;
    xyz = pointer3D->polar2Cartes3D(rtp);
    xz << xyz(0),
          xyz(2);
    return xz;
}

Eigen::Vector2d DiamondLeg2D::fkPosCartes2D(const Eigen::Vector2d &theta){
    Eigen::Vector2d q;
    q << theta(0) - pi*0.5,
         theta(1) - pi*0.5;
    Eigen::Vector2d endXZ;
    endXZ = pointer5bar->fkPlanar5bar(q,lparams);
    return endXZ;
}

Eigen::Vector2d DiamondLeg2D::fkPosPolar2D(const Eigen::Vector2d &theta){
    Eigen::Vector2d endXZ;
    endXZ = fkPosCartes2D(theta);
    Eigen::Vector2d rt;
    rt = cartes2Polar2D(endXZ);
    return rt;
}

Eigen::Vector2d DiamondLeg2D::ikPosCartes2D(const Eigen::Vector2d &endXZ){
    Eigen::Vector2d theta;
    Eigen::Vector2d q;
    q = pointer5bar->ikPlanar5bar(endXZ,lparams);
    theta << q(0) + pi/2,
             q(1) + pi/2;
    return theta;
}

Eigen::Vector2d DiamondLeg2D::ikPosPolar2D(const Eigen::Vector2d &endRT){
    Eigen::Vector2d theta;
    Eigen::Vector2d endXZ;
    endXZ = polar2Cartes2D(endRT);
    theta = ikPosCartes2D(endXZ);
    return theta;
}

Eigen::Matrix2d DiamondLeg2D::jacoCartes2D(const Eigen::Vector2d &theta){
    Eigen::Vector2d q;
    q << theta(0) - pi*0.5,
         theta(1) - pi*0.5;
    Eigen::Matrix2d jacoCartes;
    jacoCartes = pointer5bar->jacoPlanar5bar(q, lparams);
    return jacoCartes;
}

Eigen::Matrix2d DiamondLeg2D::jacoPolar2D(const Eigen::Vector2d &theta){
    Eigen::Matrix2d jacoCartes;
    Eigen::Matrix2d jacoPolar;
    Eigen::Matrix2d A;
    Eigen::Vector2d rt;

    jacoCartes = jacoCartes2D(theta);
    rt = fkPosPolar2D(theta);
    double r = rt(0);
    double t = rt(1);
    A << std::cos(t), -r*std::sin(t),
         -std::sin(t), -r*std::cos(t);
    jacoPolar = A.inverse() * jacoCartes;
    return jacoPolar;
}

Eigen::Vector2d DiamondLeg2D::fkPosCartes2D_HipKnee(const Eigen::Vector2d &q_HipKnee){
    Eigen::Vector2d xz;
    double qH = q_HipKnee(0);
    double qK = q_HipKnee(1);
    double l1 = lparams(1);
    double l2 = lparams(2);
    xz << l1*std::sin(qH) + l2*std::sin(qH + qK),
          l1*std::cos(qH) + l2*std::cos(qH + qK);
    return xz;
}

Eigen::Matrix2d DiamondLeg2D::jacoCartes2D_HipKnee(const Eigen::Vector2d& q_HipKnee){
    Eigen::Matrix2d jacoCartes_HipKnee;
    double qH = q_HipKnee(0);
    double qK = q_HipKnee(1);
    double l1 = lparams(1);
    double l2 = lparams(2);
    jacoCartes_HipKnee << l1*std::cos(qH) + l2*std::cos(qH + qK), l2*std::cos(qH + qK),
                          -l1*std::sin(qH) - l2*std::sin(qH + qK), -l2*std::sin(qH + qK);
    return jacoCartes_HipKnee;
}

/*
 * Since we already know the angle value of the HIP joint, for inverse kinematics the KNEE joint has a unique solution,
 * and there is no multi-solution problem of inverse kinematics about the up-elbow-position and the low-elbow-position.
 */
void DiamondLeg2D::calHipKnee2D(const Eigen::Vector2d &q_hip_ForeRear, const Eigen::Vector2d &qdot_hip_ForeRear,
                                Eigen::Vector2d &q_HipKnee_fore, Eigen::Vector2d &q_HipKnee_rear,
                                Eigen::Vector2d &qdot_HipKnee_fore, Eigen::Vector2d &qdot_HipKnee_rear){
    // position
    double l0 = lparams(0);
    double l1 = lparams(1);
    double q_hip_fore = q_hip_ForeRear(0);
    double q_hip_rear = q_hip_ForeRear(1);
    Eigen::Vector2d xz_foot = fkPosCartes2D(q_hip_ForeRear);
    double x_foot = xz_foot(0);
    double z_foot = xz_foot(1);
    double x_foot_fore = x_foot - 0.5*l0;
    double x_foot_rear = x_foot + 0.5*l0;
    double q_knee_fore = pi/2 - q_hip_fore - std::atan2(z_foot - l1*cos(q_hip_fore), x_foot_fore - l1*sin(q_hip_fore));
    double q_knee_rear = pi/2 - q_hip_rear - std::atan2(z_foot - l1*cos(q_hip_rear), x_foot_rear - l1*sin(q_hip_rear));
    q_HipKnee_fore << q_hip_fore,
                      q_knee_fore;
    q_HipKnee_rear << q_hip_rear,
                      q_knee_rear;
    // velocity
    Eigen::Vector2d xzDot_foot = jacoCartes2D(q_hip_ForeRear)*qdot_hip_ForeRear;
    // fore branch-tree
    Eigen::Matrix2d jacoCartes_HipKnee_fore = jacoCartes2D_HipKnee(Eigen::Vector2d(q_hip_fore, q_knee_fore));
    qdot_HipKnee_fore = jacoCartes_HipKnee_fore.inverse()*xzDot_foot;
    // rear branch-tree
    Eigen::Matrix2d jacoCartes_HipKnee_rear = jacoCartes2D_HipKnee(Eigen::Vector2d(q_hip_rear, q_knee_rear));
    qdot_HipKnee_rear = jacoCartes_HipKnee_rear.inverse()*xzDot_foot;
}

void DiamondLeg2D::calHipKnee2D(const Eigen::Vector2d &q_hip_ForeRear, const Eigen::Vector2d &qdot_hip_ForeRear,
                                const Eigen::Vector2d &xz_foot, const Eigen::Vector2d &xzDot_foot,
                                double &q_Knee_fore, double &q_Knee_rear,
                                double &qdot_Knee_fore, double &qdot_Knee_rear){
    // position
    double l0 = lparams(0);
    double l1 = lparams(1);
    double q_hip_fore = q_hip_ForeRear(0);
    double q_hip_rear = q_hip_ForeRear(1);
//    Eigen::Vector2d xz_foot = fkPosCartes2D(q_hip_ForeRear);
    double x_foot = xz_foot(0);
    double z_foot = xz_foot(1);
    double x_foot_fore = x_foot - 0.5*l0;
    double x_foot_rear = x_foot + 0.5*l0;
    q_Knee_fore = pi/2 - q_hip_fore - std::atan2(z_foot - l1*cos(q_hip_fore), x_foot_fore - l1*sin(q_hip_fore));
    q_Knee_rear = pi/2 - q_hip_rear - std::atan2(z_foot - l1*cos(q_hip_rear), x_foot_rear - l1*sin(q_hip_rear));

    // velocity
//    Eigen::Vector2d xzDot_foot = jacoCartes2D(q_hip_ForeRear)*qdot_hip_ForeRear;
    // fore branch-tree
    Eigen::Matrix2d jacoCartes_HipKnee_fore = jacoCartes2D_HipKnee(Eigen::Vector2d(q_hip_fore, q_Knee_fore));
    qdot_Knee_fore = (jacoCartes_HipKnee_fore.inverse()*xzDot_foot)(1);
    // rear branch-tree
    Eigen::Matrix2d jacoCartes_HipKnee_rear = jacoCartes2D_HipKnee(Eigen::Vector2d(q_hip_rear, q_Knee_rear));
    qdot_Knee_rear = (jacoCartes_HipKnee_rear.inverse()*xzDot_foot)(1);
}

void DiamondLeg2D::Cart2Config(const Eigen::Vector3d &xzp_base, const Eigen::Vector2d& xz_pfR, const Eigen::Vector2d& xz_pfL,
                               const Eigen::Vector3d &xzp_dot_base, const Eigen::Vector2d& xz_dot_pfR, const Eigen::Vector2d& xz_dot_pfL,
                               Eigen::VectorXd& q_Actuated, Eigen::VectorXd &q, Eigen::VectorXd &qDot){
    // update Ry
    double pitch = xzp_base(2);
    Eigen::Matrix2d Ry;
    Ry << std::cos(pitch), std::sin(pitch),
          -std::sin(pitch), std::cos(pitch);
    // update dRy
    double pitchDot = xzp_dot_base(2);
    Eigen::Matrix2d RyDot;
    RyDot << -std::sin(pitch)*pitchDot, std::cos(pitch)*pitchDot,
             -std::cos(pitch)*pitchDot, -std::sin(pitch)*pitchDot;

    /* cS <--> cB:
     *      cS = Ry*cB + dSB ;
     *      cB = Ry^T*(cS - dSB) ;
     * cDotS <--> cDotB:
     *      cDotS = Ry*cDotB + RyDot*cB + dSB_Dot ;
     *      cDotB = Ry^T*(cDotS - dSB_Dot - RyDot*cB) ;
     */
    Eigen::Vector2d xz_pfR_B = Ry.transpose() * (xz_pfR - xzp_base.head(2));
    Eigen::Vector2d xz_pfL_B = Ry.transpose() * (xz_pfL - xzp_base.head(2));
    Eigen::Vector2d xz_dot_pfR_B = Ry.transpose() * (xz_dot_pfR - xzp_dot_base.head(2) - RyDot * xz_pfR_B);
    Eigen::Vector2d xz_dot_pfL_B = Ry.transpose() * (xz_dot_pfL - xzp_dot_base.head(2) - RyDot * xz_pfL_B);

    // right leg
    Eigen::Vector2d q_hip_ForeRear_R,
                    qdot_hip_ForeRear_R,
                    q_HipKnee_fore_R, q_HipKnee_rear_R,
                    qdot_HipKnee_fore_R, qdot_HipKnee_rear_R;
    q_hip_ForeRear_R = ikPosCartes2D(xz_pfR_B);
    qdot_hip_ForeRear_R = jacoCartes2D(q_hip_ForeRear_R).inverse() * xz_dot_pfR_B;
    calHipKnee2D(q_hip_ForeRear_R, qdot_hip_ForeRear_R,
                q_HipKnee_fore_R, q_HipKnee_rear_R,
                qdot_HipKnee_fore_R, qdot_HipKnee_rear_R);
    // left leg
    Eigen::Vector2d q_hip_ForeRear_L,
                    qdot_hip_ForeRear_L,
                    q_HipKnee_fore_L, q_HipKnee_rear_L,
                    qdot_HipKnee_fore_L, qdot_HipKnee_rear_L;
    q_hip_ForeRear_L = ikPosCartes2D(xz_pfL_B);
    qdot_hip_ForeRear_L = jacoCartes2D(q_hip_ForeRear_L).inverse() * xz_dot_pfL_B;
    calHipKnee2D(q_hip_ForeRear_L, qdot_hip_ForeRear_L,
                q_HipKnee_fore_L, q_HipKnee_rear_L,
                qdot_HipKnee_fore_L, qdot_HipKnee_rear_L);

    // Debug
//    std::cout << "jacobianFLpf_hk = " << std::endl;
//    std::cout << jacoCartes2D_HipKnee(q_HipKnee_fore_L) << std::endl;

    q_Actuated.resize(4);
    q_Actuated << q_hip_ForeRear_R(0),
                  q_hip_ForeRear_R(1),
                  q_hip_ForeRear_L(0),
                  q_hip_ForeRear_L(1);

    q.resize(11);
    q <<    xzp_base(0), xzp_base(1), xzp_base(2),
            q_HipKnee_fore_R(0), q_HipKnee_fore_R(1),
            q_HipKnee_rear_R(0), q_HipKnee_rear_R(1),
            q_HipKnee_fore_L(0), q_HipKnee_fore_L(1),
            q_HipKnee_rear_L(0), q_HipKnee_rear_L(1);
    qDot.resize(11);
    qDot << xzp_dot_base(0), xzp_dot_base(1), xzp_dot_base(2),
            qdot_HipKnee_fore_R(0), qdot_HipKnee_fore_R(1),
            qdot_HipKnee_rear_R(0), qdot_HipKnee_rear_R(1),
            qdot_HipKnee_fore_L(0), qdot_HipKnee_fore_L(1),
            qdot_HipKnee_rear_L(0), qdot_HipKnee_rear_L(1);
}

void DiamondLeg2D::Force2Torque(double pitch, const Eigen::Vector2d &q_hip_ForeRear, const Eigen::Vector2d &Fc_W,
                                Eigen::Vector2d &tau_ForeRear){

    Eigen::Matrix2d Ry;
    Ry << std::cos(pitch), std::sin(pitch),
          -std::sin(pitch), std::cos(pitch);
    Eigen::Vector2d Fc_B = Ry.transpose() * Fc_W;

    tau_ForeRear = jacoCartes2D(q_hip_ForeRear).transpose() * Fc_B;
}

// ==================================== Test Functions ==========================================
bool KinematicsTest1(){

    double DEG2RAD = 0.0174532925;
    double EPSILON = 1e-6;

    Eigen::Vector2d qAct, qdAct;
    qAct << DEG2RAD*60, DEG2RAD*190;
    qdAct << DEG2RAD*50, DEG2RAD*100;

    Eigen::Vector2d xzFK, xdzdFK;
    Eigen::Vector2d qIK, qdIK;

    std::cout << "Actuated Joints Kinematics Test begin."  << std::endl;
    auto start0 = std::chrono::system_clock::now();
    // Actuated Joints Kinematics Test
    fkAct2Cart(qAct, qdAct, xzFK, xdzdFK);
    ikCart2Des(xzFK, xdzdFK, qIK, qdIK);
    auto end0 = std::chrono::system_clock::now();
    auto duration0 = std::chrono::duration_cast<std::chrono::nanoseconds>(end0 - start0);
    std::cout << "Actuated Joints Kinematics Test Time Escaped: " << double(duration0.count()) << " (ns) " << std::endl;
    std::cout << "Actuated Joints Kinematics Test finished."  << std::endl;

    std::cout << "qdqAct : " << std::endl;
    std::cout << qAct.transpose() << std::endl;
    std::cout << qdAct.transpose() << std::endl;
    std::cout << "qdqIK : " << std::endl;
    std::cout << qIK.transpose() << std::endl;
    std::cout << qdIK.transpose() << std::endl;
    std::cout << "Cart x z : " << std::endl;
    std::cout << xzFK.transpose() << std::endl;
    std::cout << xdzdFK.transpose() << std::endl;

    if (std::fabs(qAct(0) - qIK(0)) > EPSILON || std::fabs(qAct(1) - qIK(1)) > EPSILON
            || std::fabs(qdAct(0) - qdIK(0)) > EPSILON || std::fabs(qdAct(1) - qdIK(1)) > EPSILON
            ){
        std::cout << "Actuated Joints Kinematics Test failed." << std::endl;
    }else{
        std::cout << "Actuated Joints Kinematics Test passed." << std::endl;
    }

    // Passive-Tree HipKnee Kinematics Test
    Eigen::Vector2d q_hip_ForeRear = qAct, qdot_hip_ForeRear = qdAct;
    Eigen::Vector2d q_HipKnee_fore, q_HipKnee_rear, qdot_HipKnee_fore, qdot_HipKnee_rear;
    DiamondLeg2D leg2D(Eigen::Vector3d(0.097, 0.16, 0.32));
    Eigen::Vector2d xzFK_hk_fore;
    Eigen::Vector2d xzFK_hk_rear;
    Eigen::Vector2d xdzdFK_hk_fore;
    Eigen::Vector2d xdzdFK_hk_rear;

    std::cout << "Passive-Tree HipKnee Kinematics Test begin."  << std::endl;
    auto start = std::chrono::system_clock::now();
    // IK of HipKnee
    leg2D.calHipKnee2D(q_hip_ForeRear,qdot_hip_ForeRear,q_HipKnee_fore,q_HipKnee_rear,qdot_HipKnee_fore,qdot_HipKnee_rear);
    // FK of HipKnee
    xzFK_hk_fore = leg2D.fkPosCartes2D_HipKnee(q_HipKnee_fore);
    xzFK_hk_rear = leg2D.fkPosCartes2D_HipKnee(q_HipKnee_rear);
    xdzdFK_hk_fore = leg2D.jacoCartes2D_HipKnee(q_HipKnee_fore) * qdot_HipKnee_fore;
    xdzdFK_hk_rear = leg2D.jacoCartes2D_HipKnee(q_HipKnee_rear) * qdot_HipKnee_rear;
    auto end = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    std::cout << "Passive-Tree HipKnee Kinematics Test Time Escaped: " << double(duration.count()) << " (ns) " << std::endl;
    std::cout << "Passive-Tree HipKnee Kinematics Test finished."  << std::endl;

    std::cout << "Cart x z of HipKnee_fore: " << std::endl;
    std::cout << xzFK_hk_fore.transpose() + Eigen::Vector2d(0.0485, 0.).transpose() << std::endl;
    std::cout << xdzdFK_hk_fore.transpose() << std::endl;
    std::cout << "Cart x z of HipKnee_rear: " << std::endl;
    std::cout << xzFK_hk_rear.transpose() - Eigen::Vector2d(0.0485, 0.).transpose() << std::endl;
    std::cout << xdzdFK_hk_rear.transpose() << std::endl;

    if (std::fabs(xzFK_hk_fore(0) + 0.0485 - xzFK(0)) > EPSILON || std::fabs(xzFK_hk_fore(1) - xzFK(1)) > EPSILON
            || std::fabs(xdzdFK_hk_fore(0) - xdzdFK(0)) > EPSILON || std::fabs(xdzdFK_hk_fore(1) - xdzdFK(1)) > EPSILON
            || std::fabs(xzFK_hk_fore(0) + 0.0485 - xzFK_hk_rear(0) + 0.0485) > EPSILON || std::fabs(xzFK_hk_fore(1) - xzFK_hk_rear(1)) > EPSILON
            || std::fabs(xdzdFK_hk_fore(0) - xdzdFK_hk_rear(0)) > EPSILON || std::fabs(xdzdFK_hk_fore(1) - xdzdFK_hk_rear(1)) > EPSILON
            ){
        std::cout << "Passive-Tree HipKnee Kinematics Test failed." << std::endl;
    }else{
        std::cout << "Passive-Tree HipKnee Kinematics Test passed." << std::endl;
    }
    if (std::fabs(qdot_hip_ForeRear(0) - qdot_HipKnee_fore(0)) > EPSILON
            || std::fabs(qdot_hip_ForeRear(1) - qdot_HipKnee_rear(0)) > EPSILON
            ){
        std::cout << "Passive-Tree HipKnee Kinematics Test for qdot failed." << std::endl;
    }else{
        std::cout << "Passive-Tree HipKnee Kinematics Test for qdot passed." << std::endl;
    }

    return true;
}

bool KinematicsTest2(int num){

    double DEG2RAD = 0.0174532925;
    double EPSILON = 1e-6;

    Eigen::Vector2d qAct, qdAct;
    int Num = num;
    int num_Err = 0;
    double time = 0.0;
    std::srand((unsigned)std::time(NULL));    // Initialize random number seed
    for (int i = 1 ; i <= Num; ++i){
//        double qFore_init = DEG2RAD*(70. + std::rand()/double(RAND_MAX)*(240. - 70.));
//        double qRear_init = DEG2RAD*(std::max(130., qFore_init + 20.) + std::rand()/double(RAND_MAX)*(280. - std::max(130., qFore_init + 20.)));
        double qFore_init = DEG2RAD*(90. + std::rand()/double(RAND_MAX)*(180. - 90.));
        double qRear_init = DEG2RAD*(180 + std::rand()/double(RAND_MAX)*(270. - 180.));
        qAct << qFore_init,
                qRear_init;
        qdAct << DEG2RAD*(-540. + std::rand()/double(RAND_MAX)*(540.*2)),
                 DEG2RAD*(-540. + std::rand()/double(RAND_MAX)*(540.*2));

        std::cout << "the " << i << " th test begin:" << std::endl;
        Eigen::Vector2d xzFK, xdzdFK;
        Eigen::Vector2d qIK, qdIK;
        auto start = std::chrono::system_clock::now();
        // Actuated Joints Kinematics Test
        fkAct2Cart(qAct, qdAct, xzFK, xdzdFK);  
        ikCart2Des(xzFK, xdzdFK, qIK, qdIK);
        auto end = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
        time += double(duration.count());
        std::cout << "Time Escaped: " << double(duration.count()) << " (ns) " << std::endl;
        if (std::fabs(qAct(0) - qIK(0)) > EPSILON || std::fabs(qAct(1) - qIK(1)) > EPSILON
                || std::fabs(qdAct(0) - qdIK(0)) > EPSILON || std::fabs(qdAct(1) - qdIK(1)) > EPSILON
                ){
            num_Err++;
        }

        std::cout << "the " << i << " th test finished."  << std::endl;
    }
    std::cout << "Actuated Joints Kinematics Test Time Escaped Average: " << time/Num << " (ns)" << std::endl;
    std::cout << "There are " << num_Err << " Actuated Joints Kinematics tests failed." << std::endl;
    return true;
}

bool KinematicsTest3(int num){

    double DEG2RAD = 0.0174532925;
    double EPSILON = 1e-6;

    Eigen::Vector2d qAct, qdAct;
    int Num = num;
    int num_Err_prepare = 0;
    int num_Err = 0;
    int num_Err_dot = 0;
    int num_Err_knee = 0;
    double time0 = 0.0;
    double time = 0.0;
    std::srand((unsigned)std::time(NULL));    // Initialize random number seed
    for (int i = 1 ; i <= Num; ++i){
//        double qFore_init = DEG2RAD*(70. + std::rand()/double(RAND_MAX)*(240. - 70.));
//        double qRear_init = DEG2RAD*(std::max(130., qFore_init + 20.) + std::rand()/double(RAND_MAX)*(280. - std::max(130., qFore_init + 20.)));
        double qFore_init = DEG2RAD*(90. + std::rand()/double(RAND_MAX)*(180. - 90.));
        double qRear_init = DEG2RAD*(180 + std::rand()/double(RAND_MAX)*(270. - 180.));
        qAct << qFore_init,
                qRear_init;
        qdAct << DEG2RAD*(-540. + std::rand()/double(RAND_MAX)*(540.*2)),
                 DEG2RAD*(-540. + std::rand()/double(RAND_MAX)*(540.*2));

        std::cout << "the " << i << " th test begin:" << std::endl;

        // Actuated Joints Kinematics Test
        Eigen::Vector2d xzFK, xdzdFK;
        Eigen::Vector2d qIK, qdIK;
        auto start0 = std::chrono::system_clock::now();
        fkAct2Cart(qAct, qdAct, xzFK, xdzdFK);
        ikCart2Des(xzFK, xdzdFK, qIK, qdIK);
        auto end0 = std::chrono::system_clock::now();
        auto duration0 = std::chrono::duration_cast<std::chrono::nanoseconds>(end0 - start0);
        time0 += double(duration0.count());
        std::cout << "Actuated Joints Kinematics Test Time Escaped: " << double(duration0.count()) << " (ns) " << std::endl;
        if (std::fabs(qAct(0) - qIK(0)) > EPSILON || std::fabs(qAct(1) - qIK(1)) > EPSILON
                || std::fabs(qdAct(0) - qdIK(0)) > EPSILON || std::fabs(qdAct(1) - qdIK(1)) > EPSILON
                ){
            num_Err_prepare++;
        }

        // Passive-Tree HipKnee Kinematics Test
        Eigen::Vector2d q_hip_ForeRear = qAct, qdot_hip_ForeRear = qdAct;
        Eigen::Vector2d q_HipKnee_fore, q_HipKnee_rear, qdot_HipKnee_fore, qdot_HipKnee_rear;
        DiamondLeg2D leg2D(Eigen::Vector3d(0.097, 0.16, 0.32));
        Eigen::Vector2d xzFK_hk_fore;
        Eigen::Vector2d xzFK_hk_rear;
        Eigen::Vector2d xdzdFK_hk_fore;
        Eigen::Vector2d xdzdFK_hk_rear;

        auto start = std::chrono::system_clock::now();
        // IK of HipKnee
        leg2D.calHipKnee2D(q_hip_ForeRear,qdot_hip_ForeRear,q_HipKnee_fore,q_HipKnee_rear,qdot_HipKnee_fore,qdot_HipKnee_rear);
        double q_Knee_fore; double q_Knee_rear;
        double qdot_Knee_fore; double qdot_Knee_rear;
        leg2D.calHipKnee2D(q_hip_ForeRear,qdot_hip_ForeRear,xzFK,xdzdFK,q_Knee_fore,q_Knee_rear,qdot_Knee_fore,qdot_Knee_rear);
        // FK of HipKnee
        xzFK_hk_fore = leg2D.fkPosCartes2D_HipKnee(q_HipKnee_fore);
        xzFK_hk_rear = leg2D.fkPosCartes2D_HipKnee(q_HipKnee_rear);
        xdzdFK_hk_fore = leg2D.jacoCartes2D_HipKnee(q_HipKnee_fore) * qdot_HipKnee_fore;
        xdzdFK_hk_rear = leg2D.jacoCartes2D_HipKnee(q_HipKnee_rear) * qdot_HipKnee_rear;
        auto end = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
        time += double(duration.count());
        std::cout << "Passive-Tree HipKnee Kinematics Test Time Escaped: " << double(duration.count()) << " (ns) " << std::endl;
        if (std::fabs(xzFK_hk_fore(0) + 0.0485 - xzFK(0)) > EPSILON || std::fabs(xzFK_hk_fore(1) - xzFK(1)) > EPSILON
                || std::fabs(xdzdFK_hk_fore(0) - xdzdFK(0)) > EPSILON || std::fabs(xdzdFK_hk_fore(1) - xdzdFK(1)) > EPSILON
                || std::fabs(xzFK_hk_fore(0) + 0.0485 - xzFK_hk_rear(0) + 0.0485) > EPSILON || std::fabs(xzFK_hk_fore(1) - xzFK_hk_rear(1)) > EPSILON
                || std::fabs(xdzdFK_hk_fore(0) - xdzdFK_hk_rear(0)) > EPSILON || std::fabs(xdzdFK_hk_fore(1) - xdzdFK_hk_rear(1)) > EPSILON
                ){
            num_Err++;
        }
        if (std::fabs(qdot_hip_ForeRear(0) - qdot_HipKnee_fore(0)) > EPSILON
                || std::fabs(qdot_hip_ForeRear(1) - qdot_HipKnee_rear(0)) > EPSILON
                ){
            num_Err_dot++;
        }
        if (std::fabs(qdot_HipKnee_fore(1) - qdot_Knee_fore) > EPSILON
                || std::fabs(qdot_HipKnee_rear(1) - qdot_Knee_rear) > EPSILON
                || std::fabs(q_HipKnee_fore(1) - q_Knee_fore) > EPSILON
                || std::fabs(q_HipKnee_rear(1) - q_Knee_rear) > EPSILON){
            num_Err_knee++;
        }

        std::cout << "the " << i << " th test finished."  << std::endl;
    }
    std::cout << "There are " << num_Err_prepare << " Actuated Joints Kinematics tests failed." << std::endl;
    std::cout << "Actuated Joints Kinematics Test Time Escaped Average: " << time0/Num << " (ns)" << std::endl;
    std::cout << "There are " << num_Err << " Passive-Tree HipKnee Kinematics tests failed." << std::endl;
    std::cout << "There are " << num_Err_dot << " Passive-Tree HipKnee Kinematics tests failed for qdot_hip." << std::endl;
    std::cout << "Passive-Tree HipKnee Kinematics Test Time Escaped Average: " << time/Num << " (ns)" << std::endl;
    std::cout << "There are " << num_Err_knee << " Passive-Tree HipKnee Kinematics tests failed for q_knee, qdot_knee." << std::endl;

    return true;
}

bool fkAct2Cart(const Eigen::Vector2d &qAct, const Eigen::Vector2d &qdAct,
                Eigen::Vector2d &xz, Eigen::Vector2d &xdzd){
    DiamondLeg2D leg2D(Eigen::Vector3d(0.097, 0.16, 0.32));
    xz = leg2D.fkPosCartes2D(qAct);
    xdzd = leg2D.jacoCartes2D(qAct) * qdAct;
    return true;
}

bool ikCart2Des(const Eigen::Vector2d &xz, const Eigen::Vector2d &xdzd,
                Eigen::Vector2d &q, Eigen::Vector2d &qd){
    DiamondLeg2D leg2D(Eigen::Vector3d(0.097, 0.16, 0.32));
    q = leg2D.ikPosCartes2D(xz);
    qd = leg2D.jacoCartes2D(q).inverse() * xdzd;
    return true;
}

bool isVecEqual(const Eigen::VectorXd & vec_a, const Eigen::VectorXd & vec_b, const double epsilon){
    if (vec_a.size() != vec_b.size()){
        return false;
    }else{
        for(int i=0; i!=vec_a.size(); ++i){
            if(std::fabs(vec_a(i) - vec_b(i)) >= epsilon){
                return false;
            }
        }
        return true;
    }
}

// =================================End of Test Functions ==========================================

}

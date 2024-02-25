#ifndef TRACKER_H
#define TRACKER_H

#include <math.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include "image_utils/detect_process.h"
#include "print_utils.h"
#include "image_utils/lap/lapjv.h"


namespace LAP {

    void lapjv(const Eigen::MatrixXd& costMatrix, double costLimit,
                std::vector<int>& x, std::vector<int>& y) {
        //
        int n_rows = costMatrix.rows();
        int n_cols = costMatrix.cols();
        x.clear();
        y.clear();
        x.resize(n_rows);
        y.resize(n_cols);
        const int n = n_rows + n_cols;

        costLimit /= 2.;
        
        double **cost_ptr = new double*[n];
        for (int i=0; i<n; i++) {
            cost_ptr[i] = new double[n];
            for(int j=0; j<n; j++) {
                if (i<n_rows && j<n_cols) {
                    cost_ptr[i][j] = costMatrix(i, j);
                }
                else if (i>=n_rows && j>=n_cols) {
                    cost_ptr[i][j] = 0.;
                }
                else {
                    cost_ptr[i][j] = costLimit;
                }
            }
        }

        int_t x_c[n], y_c[n];
        lapjv_internal(n, cost_ptr, &x_c[0], &y_c[0]);

        for (int i=0; i<n; i++) delete[] cost_ptr[i];
        delete[] cost_ptr;

        for (int i=0;i<n_rows;i++) {
            x[i] = (x_c[i] >= n_cols)?-1:x_c[i];
        }

        for (int i=0;i<n_cols;i++) {
            y[i] = (y_c[i] >= n_rows)?-1:y_c[i];
        }

    }


    void linearAssignment(const Eigen::MatrixXd& costMatrix, double thresh,
                          std::vector<std::vector<int>>& matches,
                          std::vector<int>& unmatched_a, 
                          std::vector<int>& unmatched_b) {

        // INFO << "cost matrix:(" << costMatrix.cols()  << "," << costMatrix.rows() << ")"
        //      << "\n" << costMatrix << "\n" << ENDL;
        
        
        matches.clear();
        unmatched_a.clear();
        unmatched_b.clear();

        int n = costMatrix.rows();
        int m = costMatrix.cols();

        if (n == 0 || m == 0) {
            for (int i=0;i<n;i++) unmatched_a.push_back(i);
            for (int i=0;i<m;i++) unmatched_b.push_back(i);
            return;
        }

        std::vector<int> x, y;

        lapjv(costMatrix, thresh, x, y);
        
        for (int i = 0; i < n; ++i) {
            if (x[i] < 0) {
                unmatched_a.push_back(i);
            }
            else {
                std::vector<int> this_match;
                this_match.push_back(i);
                this_match.push_back(x[i]);
                matches.push_back(this_match);
            }
        }
        for (int j = 0; j < m; ++j) {
            if (y[j] < 0) {
                unmatched_b.push_back(j);
            }
        }
    }
}



enum struct TrackStatus { Tentative, Confirmed, Coasted, Deleted };

namespace Kalman {

    class Filter {
    public:
        Filter(int dim_x, int dim_z, int dim_u = 0)
            : dim_x(dim_x), dim_z(dim_z), dim_u(dim_u),
            x(dim_x, 1), P(dim_x, dim_x), Q(dim_x, dim_x),
            F(dim_x, dim_x), H(dim_z, dim_x), R(dim_z, dim_z),
            _alpha_sq(1.0), M(dim_z, dim_z), z(dim_z, 1),
            K(dim_x, dim_z), y(dim_z, 1), S(dim_z, dim_z),
            SI(dim_z, dim_z), _I(dim_x, dim_x),
            x_prior(dim_x, 1), P_prior(dim_x, dim_x),
            x_post(dim_x, 1), P_post(dim_x, dim_x) {
            x.setZero();
            P.setIdentity();
            Q.setIdentity();
            F.setIdentity();
            H.setZero();
            R.setIdentity();
            M.setZero();
            z.setOnes() * std::numeric_limits<float>::quiet_NaN();
            K.setZero();
            y.setZero();
            S.setZero();
            SI.setZero();
            _I.setIdentity();
            x_prior = x;
            P_prior = P;
            x_post = x;
            P_post = P;
        }

        void predict() {
            x = F * x;
            P = _alpha_sq * (F * P * F.transpose()) + Q;
            x_prior = x;
            P_prior = P;
        }

        void update(const Eigen::VectorXd& z,
                    const Eigen::MatrixXd& R) {
            if (R.rows() != dim_z || R.cols() != dim_z) {
                throw std::invalid_argument("R matrix has incorrect dimensions");
            }

            // Update state
            y = z - H * x;
            Eigen::MatrixXd PHT = P_prior * H.transpose();
            S = H * PHT + R;
            SI = S.inverse();
            K = PHT * S.inverse();
            x += K * y;

            // Update covariance
            Eigen::MatrixXd I_KH = _I - K * H;
            P = (I_KH * P_prior) * I_KH.transpose() + (K * R) * K.transpose();

            // Prepare for next prediction
            this->z = z;
            x_post = x;
            P_post = P;

        }

        Eigen::VectorXd getState() const {
            return x;
        }

        Eigen::MatrixXd getCovariance() const {
            return P;
        }

    // private:

        int dim_x, dim_z, dim_u;
        Eigen::VectorXd x, x_prior, x_post;
        Eigen::MatrixXd F, P, Q, H, R, M, z, K, y, S, SI, _I, P_prior, P_post;
        double _alpha_sq;
    };

    

    class Tracker {
    public:
        Tracker(Eigen::Vector2d y, Eigen::Matrix2d R, double wx, double wy, double vmax, 
                double w, double h, double dt = 1.0 / 30.0, int tracker_count = 0);
        void update(const Eigen::Vector2d& y, Eigen::Matrix2d& R);
        Eigen::Vector2d predict();
        Eigen::Vector4d getState() const ;
        double distance(const Eigen::Vector2d& y, Eigen::Matrix2d& R, bool show) const ;

        // 其他成员变量
        int id;
        int age;
        int death_count;
        int birth_count;
        int detidx;
        double w;
        double h;
        TrackStatus status;

    private:
        Filter kf;
    };

    
    Tracker::Tracker(Eigen::Vector2d y, Eigen::Matrix2d R, double wx, double wy, 
                     double vmax, double w, double h, double dt, int tracker_count)
        : id(tracker_count), age(0), death_count(0), birth_count(0), detidx(-1),
        w(w), h(h), status(TrackStatus::Tentative), kf(4, 2) {
        kf.F = Eigen::Matrix4d::Identity();
        kf.F(0, 1) = dt;
        kf.F(2, 3) = dt;
        kf.H = Eigen::Matrix<double, 2, 4>::Zero();
        kf.H(0, 0) = 1;
        kf.H(1, 2) = 1;
        kf.R = R;
        kf.P.setZero(); //= Eigen::Matrix<double, 4, 4>::Zero();
        kf.P << 1, 0, 0, 0,
                0, vmax * vmax / 3.0, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, vmax * vmax / 3.0;

        // INFO << "P:\n" << kf.P << ENDL;

        Eigen::Matrix<double, 4, 2> G;
        G << 0.5 * dt * dt, 0, 
             dt, 0,
             0, 0.5 * dt * dt, 
             0, dt;
        Eigen::Matrix<double, 2, 2> Q0;
        Q0.setZero();
        Q0(0, 0) = wx;
        Q0(1, 1) = wy;

        Eigen::MatrixXd _Q = (G * Q0) * G.transpose();
        kf.Q = _Q;

        kf.x << y(0), 0, y(1), 0;
    }

    void Tracker::update(const Eigen::Vector2d& y, Eigen::Matrix2d& R) {
        kf.update(y, R);
    }

    Eigen::Vector2d Tracker::predict() {
        kf.predict();
        this->age += 1;
        return kf.H * kf.x;
    }

    Eigen::Vector4d Tracker::getState() const {
        return kf.x;
    }

    double Tracker::distance(const Eigen::Vector2d& y, Eigen::Matrix2d& R, bool show=false) const {
        Eigen::Vector2d diff = y - (kf.H * kf.x);
        Eigen::Matrix2d S = kf.H * (kf.P * kf.H.transpose()) + R;
        Eigen::Matrix2d SI = S.inverse();
        auto mahalanobis = diff.transpose() * (SI * diff);
        double logdet = log(S.determinant());

        if (show) {
            // debug
            INFO << "y:\n" << y << ENDL;
            INFO << "R:\n" << R << ENDL;

            INFO << "w:\n" << w << ENDL;
            INFO << "h:\n" << h << ENDL;
            INFO << "kf.H:\n" << kf.H << ENDL;
            INFO << "kf.P:\n" << kf.P << ENDL;
            INFO << "kf.x:\n" << kf.x << ENDL;

            INFO << "diff:\n" << diff << ENDL;
            INFO << "S:\n" << S << ENDL;
            INFO << "SI:\n" << SI << ENDL;
            INFO << "mahalanobis:\n" << mahalanobis << ENDL;
            INFO << "logdet:\n" << logdet << ENDL;
        }

        return mahalanobis(0, 0) + logdet;
    }
};


namespace UCMC {

    struct Obj {
        int id=0;
        int track_idx=0;
        detect::Object obj;
        Eigen::Matrix<double, 2, 1> y;
        Eigen::Matrix2d R;
    };

    class Mapper {

        Eigen::Matrix3d A, InvA;
        Eigen::MatrixXd KiKo;

    public:
        
        bool debug=false;
        
        Mapper() {};

        Mapper(std::vector<double>& _Ki, std::vector<double>& _Ko) {
            Eigen::Map<Eigen::Matrix<double, 4, 3>> KiT(_Ki.data());
            Eigen::Map<Eigen::Matrix4d> KoT(_Ko.data());

            Eigen::Matrix<double, 3, 4> Ki = KiT.transpose();
            Eigen::Matrix4d Ko = KoT.transpose();

            A.setZero();
            InvA.setZero();
            KiKo = Ki * Ko;

            for (int now_row=0;now_row<3;now_row++)
            for (int now_col=0;now_col<3;now_col++) {
                A(now_col, now_row) = KiKo(now_col, now_row);
                if (now_row==2) {
                    A(now_col, now_row) = KiKo(now_col, 3);
                }
            }

            InvA = A.inverse();

            if (debug) {
                INFO << "A: " << A << ENDL;
                INFO << "InvA: " << InvA << ENDL;
                INFO << "Ki: " << Ki << ENDL;
                INFO << "Ko: " << Ko << ENDL;
                INFO << "KiKo: " << KiKo << ENDL;
            }
        }

        std::vector<double> uvError(cv::Rect box) {
            std::vector<double> uv;
            uv.resize(2);
            uv[0] = MAX(MIN(13., 0.05 * box.width), 2.);
            uv[1] = MAX(MIN(10., 0.05 * box.height), 2.);
            return uv;
        }

        void uv2xy(Eigen::MatrixXd uv, Eigen::MatrixXd sigma_uv, 
                   Eigen::Matrix<double, 2, 1>& xy, Eigen::Matrix2d& sigma_xy) {
            Eigen::Matrix<double, 3, 1> uv1;
            uv1(0, 0) = uv(0, 0);
            uv1(1, 0) = uv(1, 0);
            uv1(2, 0) = 1.;


            Eigen::MatrixXd b = InvA * uv1;

            // INFO << "A " << A << ENDL;
            // INFO << "InvA:\n" << InvA << ENDL;
            // INFO << "uv1:\n" << uv1 << ENDL;
            // INFO << "b:\n" << b << ENDL;


            double gamma = 1. / b(2, 0);

            Eigen::Matrix2d C = gamma * InvA.block(0, 0, 2, 2)
                              - (gamma * gamma) * b.block(0, 0, 2, 1) * InvA.block(2, 0, 1, 2);
            
            // INFO << "C:\n" << C << ENDL;
            // INFO << "sigma_uv:\n" << sigma_uv << ENDL;
            
            xy = b.block(0, 0, 2, 1) * gamma;
            sigma_xy = (C * sigma_uv) * C.transpose();

        }

        // void xy2uv(double x, double y, double& u, double& v);

        void map_to(cv::Rect box, Eigen::Matrix<double, 2, 1>& y, Eigen::Matrix2d& R){
            Eigen::Matrix<double, 2, 1> uv;
            uv(0, 0) = box.x + 0.5 * box.width;
            uv(1, 0) = box.y + box.height;
            std::vector<double> uv_err = uvError(box);
            Eigen::MatrixXd sigma_uv = Eigen::Matrix2d::Identity();
            sigma_uv(0, 0) = uv_err[1] * uv_err[1];
            sigma_uv(1, 1) = uv_err[1] * uv_err[1];
            uv2xy(uv, sigma_uv, y, R);
            

            // INFO << "y: " << y << ENDL;
            // INFO << "R: " << R << ENDL;
        }

    };

    struct Params {
        double a1 = 100.;
        double a2 = 100.;
        double wx = 5.;
        double wy = 5.;
        double vmax = 10.;
        double max_age = 10.;
        double high_score = 0.5;
        double conf_threshold = 0.01;
        double dt = 0.033;
        std::string dataset = "MOT";
        std::vector<double> Ki;
        std::vector<double> Ko;
    };

    class Tracker
    {
    private:
        Params params;
        std::vector<Kalman::Tracker> trackers;
        int frame_idx=0;
        int tracker_count = 0;
        std::vector<int> confirmed_idx, coasted_idx, tentative_idx, detidx_remain;
    public:

        Mapper mapper;
        bool debug=false;

        Tracker(Params& params);

        std::vector<Obj> update(std::vector<detect::Object> &det_results) {
            std::vector<Obj> dets;
            int id=0;
            for (detect::Object obj: det_results) {
                Obj this_obj;
                this_obj.id = id++;
                this_obj.obj = obj;
                mapper.map_to(obj.rect, this_obj.y, this_obj.R);

                dets.push_back(this_obj);
            }
            this->update(dets, frame_idx++);
            return dets;
        }

        void update(std::vector<Obj> &dets, int frame_id);

        void data_association(std::vector<Obj> &dets, int frame_id);

        void associate_tentative(std::vector<Obj> &dets);

        void initial_tentative(std::vector<Obj> &dets);

        void delete_old_trackers();

        void update_status(std::vector<Obj> &dets);

        ~Tracker();
    };
    
    Tracker::Tracker(Params& params)
    {
        this->params = params;
        mapper = Mapper(params.Ki, params.Ko);
    }

    void Tracker::update(std::vector<Obj> &dets, int frame_id) {
        this->data_association(dets, frame_id);
        this->associate_tentative(dets);
        this->initial_tentative(dets);
        this->delete_old_trackers();
        this->update_status(dets);
    }

    void Tracker::data_association(std::vector<Obj> &dets, int frame_id) {
        std::vector<int> detidx_high, detidx_low;
        for (size_t i = 0; i < dets.size(); ++i) {
            if (dets[i].obj.prob >= params.high_score) {
                detidx_high.push_back(i);
            } else {
                detidx_low.push_back(i);
            }
        }

        for (auto& track : trackers) {
            track.predict();
        }

        std::vector<int> trackidx_remain;
        detidx_remain.clear();

        // 关联高分检测与轨迹
        // INFO << "try high score mapping" << ENDL;
        std::vector<int> trackidx = confirmed_idx;
        trackidx.insert(trackidx.end(), coasted_idx.begin(), coasted_idx.end());

        int num_det = detidx_high.size();
        int num_trk = trackidx.size();

        // 初始化轨迹的detidx为-1
        for (auto& track : trackers) {
            track.detidx = -1;
        }
        
        if (num_det * num_trk > 0) {
            Eigen::MatrixXd cost_matrix(num_det, num_trk);
            cost_matrix.setZero();

            for (int i = 0; i < num_det; ++i) {
                int det_idx = detidx_high[i];
                for (int j = 0; j < num_trk; ++j) {
                    cost_matrix(i, j) = trackers[trackidx[j]].distance(
                        dets[det_idx].y, 
                        dets[det_idx].R,
                        i+j==0 && debug
                    );
                }
            }

            // INFO << "det[0].y:\n" << dets[0].y << ENDL;
            // INFO << "det[0].R:\n" << dets[0].R << ENDL;


            // INFO << "cost matrix: " << cost_matrix << ENDL;
            
            std::vector<std::vector<int>> matched_indices;
            std::vector<int> unmatched_a;
            std::vector<int> unmatched_b;
            LAP::linearAssignment(cost_matrix, this->params.a1, 
                                  matched_indices, unmatched_a, unmatched_b);

            // INFO << "high: " << detidx_remain.size() << " " << unmatched_a.size() << " "
            //      << unmatched_b.size() << ENDL;

            // 处理未匹配的检测和轨迹
            for (int i : unmatched_a) {
                detidx_remain.push_back(detidx_high[i]);
            }
            for (int i : unmatched_b) {
                trackidx_remain.push_back(trackidx[i]);
            }

            for (size_t i=0; i<matched_indices.size(); ++i) {
                int det_idx = detidx_high[matched_indices[i][0]];
                int trk_idx = trackidx[matched_indices[i][1]];

                trackers[trk_idx].update(dets[det_idx].y, dets[det_idx].R);
                trackers[trk_idx].death_count = 0;
                trackers[trk_idx].detidx = det_idx;
                trackers[trk_idx].status = TrackStatus::Confirmed;

                dets[det_idx].track_idx = trackers[trk_idx].id;

            }
        }
        else {
            detidx_remain = detidx_high;
            trackidx_remain = trackidx;
        }

        
        // 将低分检测与剩余轨迹关联
        int num_det_low = detidx_low.size();
        int num_trk_remain = trackidx_remain.size();

        // INFO << "try low score mapping" << ENDL;
        if (num_det_low * num_trk_remain > 0) {
            Eigen::MatrixXd cost_matrix(num_det_low, num_trk_remain);
            cost_matrix.setZero();

            for (int i = 0; i < num_det_low; ++i) {
                int det_idx = detidx_low[i];
                for (int j = 0; j < num_trk_remain; ++j) {
                    int trk_idx = trackidx_remain[j];
                    cost_matrix(i, j) = trackers[trk_idx].distance(
                        dets[det_idx].y, 
                        dets[det_idx].R,
                        i+j==0 && debug
                    );
                }
            }

            // INFO << "det[0].y:\n" << dets[0].y << ENDL;
            // INFO << "det[0].R:\n" << dets[0].R << ENDL;

            std::vector<std::vector<int>> matched_indices;
            std::vector<int> unmatched_a;
            std::vector<int> unmatched_b;
            LAP::linearAssignment(cost_matrix, this->params.a2,
                                  matched_indices, unmatched_a, unmatched_b);
            
            // 处理未匹配的轨迹
            for (int i : unmatched_b) {
                int trk_idx = trackidx_remain[i];
                trackers[trk_idx].status = TrackStatus::Coasted;
                // trackers[trk_idx].death_count += 1; // 如果需要的话
                trackers[trk_idx].detidx = -1;
            }

            // 更新匹配的轨迹
            for (size_t i=0; i<matched_indices.size(); ++i) {
                int det_idx = detidx_low[matched_indices[i][0]];
                int trk_idx = trackidx_remain[matched_indices[i][1]];

                trackers[trk_idx].update(dets[det_idx].y, dets[det_idx].R);
                trackers[trk_idx].death_count = 0;
                trackers[trk_idx].detidx = det_idx;
                trackers[trk_idx].status = TrackStatus::Confirmed;
                dets[det_idx].track_idx = trackers[trk_idx].id;
            }
        }

    }

    void Tracker::associate_tentative(std::vector<Obj> &dets) {
        // INFO << "try associate_tentative mapping" << ENDL;
        size_t num_det = detidx_remain.size();
        size_t num_trk = tentative_idx.size();

        Eigen::MatrixXd cost_matrix(num_det, num_trk);
        cost_matrix.setZero();

        for (int i=0;i<num_det;++i) {
            int det_idx = detidx_remain[i];
            for (int j=0;j<num_trk;++j) {
                int trk_idx = tentative_idx[j];
                cost_matrix(i, j) = trackers[trk_idx].distance(
                    dets[det_idx].y, 
                    dets[det_idx].R,
                    i+j==0 && debug
                );
            }
        }

        // INFO << "det[0].y:\n" << dets[0].y << ENDL;
        // INFO << "det[0].R:\n" << dets[0].R << ENDL;

        std::vector<std::vector<int>> matched_indices;
        std::vector<int> unmatched_a;
        std::vector<int> unmatched_b;
        LAP::linearAssignment(cost_matrix, this->params.a1,
                              matched_indices, unmatched_a, unmatched_b);

        // INFO << detidx_remain.size() << " " << unmatched_a.size() << " "
        //      << unmatched_b.size() << ENDL;
        
        for (size_t i=0; i<matched_indices.size(); ++i) {
            int det_idx = detidx_remain[matched_indices[i][0]];
            int trk_idx = tentative_idx[matched_indices[i][1]];

            trackers[trk_idx].update(dets[det_idx].y, dets[det_idx].R);
            trackers[trk_idx].death_count = 0;
            trackers[trk_idx].birth_count++;
            trackers[trk_idx].detidx = det_idx;
            dets[det_idx].track_idx = trackers[trk_idx].id;
            if (trackers[trk_idx].birth_count >= 2) {
                trackers[trk_idx].birth_count = 0;
                trackers[trk_idx].status = TrackStatus::Confirmed;
            }
        }

        for (int i: unmatched_b) {
            int trk_idx = tentative_idx[i];
            trackers[trk_idx].detidx--;
        }

        std::vector<int> unmatched_detidx;
        for (int i: unmatched_a) {
            unmatched_detidx.push_back(detidx_remain[i]);
        }
        detidx_remain = unmatched_detidx;
    }

    void Tracker::initial_tentative(std::vector<Obj> &dets) {
        // INFO << detidx_remain.size() << ENDL;
        for (int i: detidx_remain) {
            // INFO << "detidx:" << i << "" << dets[i].obj.rect << ENDL;
            Kalman::Tracker new_obj(
                dets[i].y,
                dets[i].R,
                this->params.wx,
                this->params.wy,
                this->params.vmax,
                dets[i].obj.rect.width,
                dets[i].obj.rect.height,
                this->params.dt,
                ++this->tracker_count
            );
            new_obj.status = TrackStatus::Tentative;
            new_obj.detidx = i;
            trackers.push_back(new_obj);
        }
        detidx_remain.clear();
    }

    void Tracker::delete_old_trackers() {
        std::vector<int> idx_reserve;
        std::vector<Kalman::Tracker> reserve_trackers;
        for (int trk_idx=0; trk_idx<trackers.size(); trk_idx++) {
            trackers[trk_idx].death_count++;
            if (!((trackers[trk_idx].status == TrackStatus::Coasted && 
                 trackers[trk_idx].death_count >= this->params.max_age) || 
                (trackers[trk_idx].status == TrackStatus::Tentative &&
                 trackers[trk_idx].death_count >= 2))) {
                reserve_trackers.push_back(trackers.at(trk_idx));
            }
        }
        trackers = reserve_trackers;
    }

    void Tracker::update_status(std::vector<Obj> &dets) {
        confirmed_idx.clear();
        coasted_idx.clear();
        tentative_idx.clear();

        for (int i=0;i<trackers.size();i++) {
            int detidx = trackers[i].detidx;

            if (detidx >= 0 && detidx < dets.size()) {
                trackers[i].h = dets[detidx].obj.rect.height;
                trackers[i].w = dets[detidx].obj.rect.width;
            }
            
            switch (trackers[i].status)
            {
            case TrackStatus::Confirmed:
                confirmed_idx.push_back(i);
                break;
            case TrackStatus::Coasted:
                coasted_idx.push_back(i);
                break;
            case TrackStatus::Tentative:
                tentative_idx.push_back(i);
                break;
            default:
                break;
            }
            
        }
    }
    
    Tracker::~Tracker() {
        trackers.clear();
        confirmed_idx.clear();
        coasted_idx.clear();
        tentative_idx.clear();
        detidx_remain.clear();
    }
    
}


namespace Track {

    static cv::Mat draw_boxes(cv::Mat image, 
                       std::vector<UCMC::Obj> &objects, 
                       std::vector<std::string>& names,
                       int draw_size=20, 
                       bool draw_label=true) {
        cv::Mat d_img = image.clone();
        cv::Scalar color;
        cv::Scalar txt_color;
        cv::Scalar txt_bk_color;
        cv::Size label_size;
        int baseLine = 0;
        int x, y, out_point_y, w, h;
        int line_thickness = std::round((double)draw_size / 10.0);
        
        for(int k=0; k<objects.size(); k++){

            if (!objects.at(k).track_idx) continue;
            color = detect::get_color(objects.at(k).obj.label);

            x = objects.at(k).obj.rect.x;
            y = objects.at(k).obj.rect.y;
            w = objects.at(k).obj.rect.width;
            h = objects.at(k).obj.rect.height;

            cv::rectangle(d_img,
                        objects.at(k).obj.rect,
                        color,
                        line_thickness);
            
            if (draw_label){
                txt_color = (cv::mean(color)[0] > 127)?cv::Scalar(0, 0, 0):cv::Scalar(255, 255, 255);
                std::string label = std::to_string(objects.at(k).track_idx) + " " + names.at(objects.at(k).obj.label) + " " + std::to_string(objects.at(k).obj.prob).substr(0, 4);
                label_size = cv::getTextSize(label.c_str(), cv::LINE_AA, double(draw_size) / 30.0, (line_thickness>1)?line_thickness-1:1, &baseLine);
                txt_bk_color = color; // * 0.7;
                y = (y > d_img.rows)?d_img.rows:y + 1;
                out_point_y = y - label_size.height - baseLine;
                if (out_point_y >= 0) y = out_point_y;
                cv::rectangle(d_img, cv::Rect(cv::Point(x - (line_thickness - 1), y), cv::Size(label_size.width, label_size.height + baseLine)),
                            txt_bk_color, -1);
                cv::putText(d_img, label, cv::Point(x, y + label_size.height),
                            cv::LINE_AA, double(draw_size) / 30.0, txt_color, (line_thickness>1)?line_thickness-1:1);
            }

        }
        return d_img;
    }
}


#endif

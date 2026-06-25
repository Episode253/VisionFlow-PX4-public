#include "clik_cplus.h"
#include "mylib.h"


Eigen::MatrixXd pinv(Eigen::MatrixXd  A)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double  pinvtoler = 1.e-8; //tolerance
    int row = A.rows();
    int col = A.cols();
    int k = std::min(row,col);
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col,row);
    Eigen::MatrixXd singularValues_inv = svd.singularValues();//奇异值
    Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
    for (long i = 0; i<k; ++i) {
        if (singularValues_inv(i) > pinvtoler)
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        else singularValues_inv(i) = 0;
    }
    for (long i = 0; i < k; ++i) 
    {
        singularValues_inv_mat(i, i) = singularValues_inv(i);
    }
    X=(svd.matrixV())*(singularValues_inv_mat)*(svd.matrixU().transpose());
 
    return X;
}

void anti_symmetry(Eigen::Vector3d  input, Eigen::Matrix3d&  output){
    output<< 0, -input(2),input(1),
             input(2), 0, -input(0),
             -input(1), input(0), 0;
}

// 欧拉角转旋转矩阵
void euler_to_rotation(const Eigen::Vector3d& euler, Eigen::Matrix3d& rotation)
{
    rotation(0,0) = cos(euler(1))*cos(euler(2));
    rotation(0,1) = sin(euler(1))*sin(euler(0))*cos(euler(2))-cos(euler(0))*sin(euler(2));
    rotation(0,2) = sin(euler(1))*cos(euler(0))*cos(euler(2))+sin(euler(0))*sin(euler(2));
    rotation(1,0) = cos(euler(1))*sin(euler(2));
    rotation(1,1) = sin(euler(1))*sin(euler(0))*sin(euler(2))+cos(euler(0))*cos(euler(2));
    rotation(1,2) = -sin(euler(0))*cos(euler(2))+cos(euler(0))*sin(euler(1))*sin(euler(2));
    rotation(2,0) = -sin(euler(1));
    rotation(2,1) = -sin(euler(0))*cos(euler(1));
    rotation(2,2) = cos(euler(0))*cos(euler(1));
}

void clik_c_::clik_solver(){
    Eigen::Matrix3d rotation = CLIK_U.rotation_d2i;
    Eigen::MatrixXd Jacob(3, 6);
    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            if (i==j)
            {
                Jacob(i,j) = 1.0;
            }
            else
            {
                Jacob(i,j) = 0;
            }
        }  
    }
    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 3; j < 6; j++)
        {
            Jacob(i,j) = rotation(i,j-3);
        } 
    }
    Eigen::Vector3d pe(CLIK_U.Pe[0],CLIK_U.Pe[1],CLIK_U.Pe[2]);
    Eigen::Vector3d pb(CLIK_U.state[0],CLIK_U.state[1],CLIK_U.state[2]);
    Eigen::Matrix3d S_R_peb;
    anti_symmetry(pe - pb, S_R_peb);
    Eigen::Vector3d wb(CLIK_U.state[6],CLIK_U.state[7],CLIK_U.state[8]);
    Eigen::Vector3d vcmd(CLIK_U.Pe_vcmd[0],CLIK_U.Pe_vcmd[1],CLIK_U.Pe_vcmd[2]);
    Eigen::Vector3d pecmd(CLIK_U.Pe_cmd[0],CLIK_U.Pe_cmd[1],CLIK_U.Pe_cmd[2]);
    double kineKp = 0.9;
    Eigen::Vector3d pv_spr = vcmd - kineKp*(pe - pecmd);// + S_R_peb*wb;

    if (CLIK_U.distance_manip < r_s)// 如果当前位置位于两个操纵点之间，输入的CLIK_U.distance_manip应为0
    {
        if (CLIK_U.distance_manip>r_w)
        {
            d_s_ = d_max_ * ( CLIK_U.distance_manip - r_s )/(r_w -r_s);
        }
        else
        {
            d_s_ = d_max_;
        } 
    }
    else
    {
        d_s_ = 0;
    }
    // 【构建P矩阵】
    c_int P_obj_max_num = 6;
    c_float  u_var = 0.010;
    c_float P_obj_var[6] = {u_var /( xi_dot_max[0]*xi_dot_max[0]),u_var /( xi_dot_max[1]*xi_dot_max[1]),u_var / (xi_dot_max[2]*xi_dot_max[2]),
                1.0/(xi_dot_max[3]*xi_dot_max[3]) ,  1.0/(xi_dot_max[4]*xi_dot_max[4]),  1.0/(xi_dot_max[5]*xi_dot_max[5])};
    c_int P_obj_raw[6] = {0,1,2,3,4,5};
    c_int P_obj_con[7] = {0,1,2,3,4,5,6};
    // 【构建q向量】
    c_float q_obj[6] = {0,0,0,0,0,0};
    // 【构建A矩阵】
    c_int A_cons_max_num = 18;
    c_float A_cons_var[18] = {1,1,1,1,1,1,rotation(0,0),rotation(1,0),rotation(2,0),1,
    rotation(0,1),rotation(1,1),rotation(2,1),1,
    rotation(0,2),rotation(1,2),rotation(2,2),1};
    c_int A_cons_raw[18] = {0,3,1,4,2,5,0,1,2,6,0,1,2,7,0,1,2,8};
    c_int A_cons_con[19] = {0,2,4,6,10,14,18};
    c_int A_cons_m = 9;
    c_int A_cons_n = 6;
    // 【构建l和u向量】
    float xi[6] = {CLIK_U.state[0],CLIK_U.state[1],CLIK_U.state[2],CLIK_U.Ped[0],CLIK_U.Ped[1],CLIK_U.Ped[2]};
    // printf("----------------xi--------------\n");
    // fvector_show(xi,6);
    double obser_cons_low[6],obser_cons_up[6];
    for (size_t i = 0; i < 6; i++)
    {
        obser_cons_low[i] = 2* xi_ddot_max[i]*xi[i]/xi_dot_min[i]-2*xi_ddot_max[i]*xi_min[i]/xi_dot_min[i];
        obser_cons_up[i]  = 2* xi_ddot_min[i]*xi[i]/xi_dot_max[i]-2*xi_ddot_min[i]*xi_max[i]/xi_dot_max[i];
    }
    double xi_cons_low[6],xi_cons_up[6];
    for (size_t i = 0; i < 6; i++)
    {
        xi_cons_low[i] = (xi_min[i]-xi[i])/CLIK_U.dtime;
        xi_cons_up[i]  = (xi_max[i]-xi[i])/CLIK_U.dtime;
    }

    c_float l_cons[9],u_cons[9];
    l_cons[0]  = pv_spr(0);
    l_cons[1]  = pv_spr(1);
    l_cons[2]  = pv_spr(2);
    u_cons[0]  = pv_spr(0);
    u_cons[1]  = pv_spr(1);
    u_cons[2]  = pv_spr(2);
    for (size_t i = 0; i < 6; i++)
    {
        l_cons[3+i] = max_3(xi_cons_low[i],xi_dot_min[i],obser_cons_low[i]);
        u_cons[3+i] = min_3(xi_cons_up[i],xi_dot_max[i],obser_cons_up[i]);
    }
    printf("------------upper cons 5 ----------\n");
    printf("xi_cons_up = %f\n",xi_cons_up[2]);
    printf("xi_dot_max = %f\n",xi_dot_max[2]);
    printf("obser_cons_up = %f\n",obser_cons_up[2]);

    printf("-----------------l--------------\n");
    dvector_show(l_cons,6);
    printf("-----------------u--------------\n");
    dvector_show(u_cons,6);
    c_int exitflag = 0;
    // Workspace structures
    OSQPWorkspace *work;
    OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

    // Populate data
    if (data) {
        data->n = A_cons_n;
        data->m = A_cons_m;
        data->P = csc_matrix(data->n, data->n, P_obj_max_num, P_obj_var, P_obj_raw, P_obj_con);
        data->q = q_obj;
        data->A = csc_matrix(data->m, data->n, A_cons_max_num, A_cons_var, A_cons_raw, A_cons_con);
        data->l = l_cons;
        data->u = u_cons;
    }
    // csc_show(data->P);
    // Define solver settings as default
    if (settings) osqp_set_default_settings(settings);

    // Setup workspace
    exitflag = osqp_setup(&work, data, settings);

    // Solve Problem
    osqp_solve(work);
    
    for (size_t i = 0; i < 3; i++)
    {
        CLIK_Y.quad_out[i] =work->solution->x[i];
    }
    for (size_t i = 0; i < 3; i++)
    {
        CLIK_Y.delta_out[i] = work->solution->x[i+3];
    }

    // Clean workspace
    osqp_cleanup(work);
    if (data) {
        if (data->A) c_free(data->A);
        if (data->P) c_free(data->P);
        c_free(data);
    }
    if (settings)  c_free(settings);
}

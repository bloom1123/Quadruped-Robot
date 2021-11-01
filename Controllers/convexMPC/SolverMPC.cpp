#include "SolverMPC.h"
#include "common_types.h"
#include "convexMPC_interface.h"
#include "RobotState.h"
#include "../../third-party/eigen3/Eigen/Dense"
#include <cmath>
#include "../../third-party/eigen3/unsupported/Eigen/MatrixFunctions"
//#include <unsupported/Eigen/MatrixFunctions>
#include <qpOASES.hpp>
// #include <sys/time.h>

// #define BIG_NUMBER 5e10
//big enough to act like infinity, small enough to avoid numerical weirdness.

RobotState rs;
using std::cout;
using std::endl;
using Eigen::Dynamic;
using Eigen::Matrix;
using Eigen::Array;

//qpOASES::real_t a;
Matrix<fpt,Dynamic,13> A_qp;
Matrix<fpt,Dynamic,Dynamic> B_qp;
Matrix<fpt,Dynamic,1> C_qp;
Matrix<fpt,Dynamic,1> Xsolin;
Matrix<fpt,13,12> Bdt;
Matrix<fpt,13,13> Adt;
Matrix<fpt,13,1> Cdt;
Matrix<fpt,26,26> ABc,expmm;
Matrix<fpt,Dynamic,Dynamic> S;
Matrix<fpt,Dynamic,1> X_d;
Matrix<fpt,Dynamic,Dynamic> fmat;
Matrix<fpt,Dynamic,Dynamic> qH;
Matrix<fpt,Dynamic,1> qg;
Matrix<fpt,Dynamic,Dynamic> eye_12h;
Matrix<fpt,Dynamic,1> U_sol;
qpOASES::real_t* H_qpoases;
qpOASES::real_t* g_qpoases;
qpOASES::real_t* A_qpoases;
qpOASES::real_t* lb_qpoases;
qpOASES::real_t* q_soln;
u8 real_allocated = 0;


Matrix<int,Dynamic,1> vNH;
Array<Matrix<int,Dynamic,1>,Dynamic,1> acH;
int NModes;
//int* vNH;
//int** acH;
Matrix<fpt,Dynamic,1> get_Xsolin()
{return Xsolin;}
mfp* get_q_soln()
{return q_soln;}
s8 near_zero(fpt a)
{return (a < 0.01 && a > -.01);}
s8 near_one(fpt a)
{return near_zero(a-1);}

void real_t_to_matrix(Matrix<fpt,Dynamic,1> &dst, qpOASES::real_t* src, s32 n_items){
  for(s32 i = 0; i < n_items; i++)
    dst(i) = src[i];
}

void matrix_to_real(qpOASES::real_t* dst, Matrix<fpt,Dynamic,Dynamic> src, s16 rows, s16 cols){
  s32 a = 0;
  for(s16 r = 0; r < rows; r++)
  {
    for(s16 c = 0; c < cols; c++)
    {
      dst[a] = src(r,c);
      a++;
    }
  }
}
void getvNHacH(unsigned char* gait, int horiz){
  vNH.resize(horiz, Eigen::NoChange);
  s16 HC = 0;
  for(int i = 1; i < horiz; i++)
  {
    for(s16 j = 0; j < 4; j++)
    {
      if (gait[4*i+j]!=gait[4*(i-1)+j]){
        vNH(HC)=i-1;
        HC++;
        break;
      }
    }
  }
  NModes=HC+1;
  vNH(HC)=horiz-1;
  vNH.conservativeResize(NModes, Eigen::NoChange);
  acH.conservativeResize(NModes, Eigen::NoChange);
  for(int HCi = 0; HCi<HC+1; HCi++){
        acH(HCi).conservativeResize(4,Eigen::NoChange);
          s16 lki = 0;
        for(int ki = 0;ki < 4; ki++){
          if ((int)gait[4*vNH(HCi)+ki]==1){
            acH(HCi)(lki)=ki;
            lki++;
          }
        }
        acH(HCi).conservativeResize(lki,Eigen::NoChange);
  }
}

void c2qp(Matrix<fpt,13,13> Ac, Matrix<fpt,13,12> Bc, Matrix<fpt,13,1> Cc,problem_setup* setup){
  fpt dt=setup->dt;
  s16 horizon=setup->horizon;
  Matrix<fpt,3,3> Rplac2qp[4];
  for(s16 i=0;i<4;i++){
    Rplac2qp[i]=setup->Rpla[i];}

  ABc.setZero();
  ABc.block(0,0,13,13) = Ac;
  ABc.block(0,13,13,12) = Bc;
  ABc.block(0,25,13,1) = Cc;
  ABc = dt*ABc;
  expmm = ABc.exp();
  Adt = expmm.block(0,0,13,13);
  Bdt = expmm.block(0,13,13,12);
  Cdt = expmm.block(0,25,13,1);

  if(horizon > 29) {
    throw std::runtime_error("horizon is too long!");
  }
  Matrix<fpt,13,13> powerMats[30];
  Matrix<fpt,13,1> sumpowerMats[30];
  powerMats[0].setIdentity();
  sumpowerMats[0]=Cdt;
  for(int i = 1; i < horizon+1; i++) {
    powerMats[i] = Adt * powerMats[i-1];
    sumpowerMats[i]=Adt*sumpowerMats[i-1]+Cdt;
  }
  Eigen::Array<Matrix<fpt,13,Dynamic>,Dynamic,1> BdtH;
  BdtH.resize(NModes,1);

  fpt mu = 1.f/setup->mu[0];
  Matrix<fpt,6,3> f_block;
  f_block <<  mu, 0,  1.f,
    -mu, 0,  1.f,
    0,  mu, 1.f,
    0, -mu, 1.f,
    0,   0, 1.f,
    0,   0, -1.f;

  u32 ifm=0;
    for (u16 iN = 0; iN < vNH(0)+1;iN++){  
      for (u16 N3 = 0; N3 < acH(0).rows();N3++){
        fmat.block(ifm*6,ifm*3,6,3) = f_block*(Rplac2qp[acH(0)(N3)].transpose());
        ifm++;
      }
    }
  for (u16 N = 1; N < NModes;N++){
    for (u16 iN = vNH(N-1)+1; iN < vNH(N)+1;iN++){  
      for (u16 N3 = 0; N3 < acH(N).rows();N3++){
        fmat.block(ifm*6,ifm*3,6,3) = f_block*(Rplac2qp[acH(N)(N3)].transpose());
        ifm++;
      }
    }
  }

  for (u16 N = 0; N < NModes;N++){
    BdtH(N).resize(13,acH(N).rows()*3);
    for (u16 N3 = 0; N3 < acH(N).rows();N3++){
      BdtH(N).block(0,3*N3,13,3)=Bdt.block(0,(acH(N)(N3))*3,13,3);
    }
  }
  for(s16 r = 0; r < horizon; r++)
  {
    A_qp.block(13*r,0,13,13) = powerMats[r+1];//Adt.pow(r+1);
    C_qp.block(13*r,0,13,1) = sumpowerMats[r];//Adt.pow(r+1);
    int aB_qp =0;
    
    for (int N2 = 0; N2 < (int)fmin(vNH(0)+1,r+1);N2++){
      B_qp.block(13*r,aB_qp,13,3*acH(0).rows()) = powerMats[r-N2] /*Adt.pow(a_num)*/ * BdtH(0);
        aB_qp+=3*acH(0).rows();
      }
    for (u16 N = 1; N < NModes;N++){
      for (int N2 = vNH(N-1)+1; N2 < (int) fmin(vNH(N)+1,r+1);N2++){
      B_qp.block(13*r,aB_qp,13,3*acH(N).rows()) = powerMats[r-N2] /*Adt.pow(a_num)*/ * BdtH(N);
        aB_qp+=3*acH(N).rows();
      }
    }
  }
}

void resize_qp_mats(s16 horizon, int sga)
{
  int mcount = 0;
  int h2 = horizon*horizon;
  int sghor = sga*horizon;
  int sga2 = sga*sga;


  A_qp.resize(13*horizon, Eigen::NoChange);
  mcount += 13*horizon;//??
  U_sol.resize(3*sga, Eigen::NoChange);
  mcount += 3*sga;//??
  Xsolin.resize(13*horizon, Eigen::NoChange);
  mcount += 13*horizon;
  B_qp.resize(13*horizon, 3*sga);
  mcount += 13*3*sghor;
  C_qp.resize(13*horizon, Eigen::NoChange);
  mcount += 13*horizon;
  S.resize(13*horizon, 13*horizon);
  mcount += 13*13*h2;
  X_d.resize(13*horizon, Eigen::NoChange);
  mcount += 13*horizon;
  fmat.resize(6*sga, 3*sga);
  mcount += 6*3*sga2;
  qH.resize(3*sga, 3*sga);
  mcount += 3*3*sga2;
  qg.resize(3*sga, Eigen::NoChange);
  mcount += 3*sga;
  eye_12h.resize(3*sga, 3*sga);
  mcount += 3*3*sga2;
  //printf("realloc'd %d floating point numbers.\n",mcount);
  mcount = 0;
  A_qp.setZero();
  B_qp.setZero();
  C_qp.setZero();
  S.setZero();
  X_d.setZero();
  fmat.setZero();
  qH.setZero();
  U_sol.setZero();
  Xsolin.setZero();
  eye_12h.setIdentity();

  //TODO: use realloc instead of free/malloc on size changes

  if(real_allocated){
    free(H_qpoases);
    free(g_qpoases);
    free(A_qpoases);
    free(lb_qpoases);
    free(q_soln);
  }

  H_qpoases = (qpOASES::real_t*)malloc(3*3*sga2*sizeof(qpOASES::real_t));
  mcount += 3*3*sga2;
  g_qpoases = (qpOASES::real_t*)malloc(3*sga*sizeof(qpOASES::real_t));
  mcount += 3*sga;
  A_qpoases = (qpOASES::real_t*)malloc(3*6*sga2*sizeof(qpOASES::real_t));
  mcount += 3*6*sga2;
  lb_qpoases = (qpOASES::real_t*)malloc(6*sga*sizeof(qpOASES::real_t));
  mcount += 6*sga;
  q_soln = (qpOASES::real_t*)malloc(3*sga*sizeof(qpOASES::real_t));
  mcount += 3*sga;
  real_allocated = 1;
}

inline Matrix<fpt,3,3> cross_mat(Matrix<fpt,3,1> r){
  Matrix<fpt,3,3> cm;
  cm << 0.f, -r(2), r(1),
    r(2), 0.f, -r(0),
    -r(1), r(0), 0.f;
  return cm;
}

Matrix<fpt,4,3> QMATquat(Quaternionf q){
  Matrix<fpt,4,3> ouQ;
  Matrix<fpt,3,1> ouQ2(q.x(), q.y(), q.z());
  ouQ << -q.x(), -q.y(), -q.z(),
    0,0,0,
    0,0,0,
    0,0,0;
  ouQ.block(1,0,3,3) = -cross_mat(ouQ2)+q.w()*(Matrix<fpt,3,3>::Identity());
  return (ouQ/2);
}

//continuous time state space matrices.
void ct_ss_mats(Matrix<fpt,3,3> I_world, fpt m, Matrix<fpt,3,4> r_feet, Quaternionf q, Matrix<fpt,13,13>& A, Matrix<fpt,13,12>& B, Matrix<fpt,13,1>& C){
  Matrix<fpt,3,3> I_inv = I_world.inverse();
  A.setZero();
  A(4,10) = 1.f;
  //A(11,9) = x_drag;
  A(5,11) = 1.f;
  A(6,12) = 1.f;
  A.block(0,7,4,3) = QMATquat(q)*I_inv;
  B.setZero();
  for(s16 b = 0; b < 4; b++){
    B.block(7,b*3,3,3) = cross_mat(r_feet.col(b));
    B.block(10,b*3,3,3) = Matrix<fpt,3,3>::Identity() / m;
  }
  C.setZero();
  C(12,0)=-9.81f;//maybe
}

Matrix<fpt,13,1> x_0;
Matrix<fpt,3,3> I_world;
Matrix<fpt,13,13> A_ct;
Matrix<fpt,13,12> B_ct_r;
Matrix<fpt,13,1> c_ct;

void solve_mpc(update_data_t* update, problem_setup* setup){
  rs.set(update->p, update->v, update->q, update->w, update->r, update->mass);

  I_world = rs.R * rs.I_body * rs.R.transpose();
  x_0 << rs.q.w(), rs.q.x(), rs.q.y(), rs.q.z(), rs.p, I_world*rs.w, rs.v; 
  //cout<<rs.R_yaw<<endl;
  ct_ss_mats(I_world,rs.m,rs.r_feet,rs.q,A_ct,B_ct_r,c_ct);

  getvNHacH(update->gait, setup->horizon);
  c2qp(A_ct,B_ct_r,c_ct,setup);//****needed??
  Matrix<fpt,13,1> full_weight;
  for(u8 i = 0; i < 13; i++)
    full_weight(i) = (float)update->weights[i];
  S.diagonal() = full_weight.replicate(setup->horizon,1);

  for(s16 i = 0; i < setup->horizon; i++){
    for(s16 j = 0; j < 13; j++)
      X_d(13*i+j,0) = update->traj[13*i+j];
  }

  qH = 2*(B_qp.transpose()*S*B_qp+ update->alpha*eye_12h*0.001);//
  qg = 2*B_qp.transpose()*S*(A_qp*x_0 - (X_d-C_qp));
  //QpProblem<double> jcqp(setup->horizon*12, setup->horizon*20);
  matrix_to_real(H_qpoases,qH,qH.rows(), qH.cols());
  matrix_to_real(g_qpoases,qg,qg.rows(), 1);
  matrix_to_real(A_qpoases,fmat,fmat.rows(), fmat.cols());
  
  float fmaxQP = - (setup->f_max);
  for(s16 i = 0; i < fmat.rows(); i++)
    lb_qpoases[i] = (i%6==5)*fmaxQP;

  //printf("Timeitlbtime: %.4f ms\n", itlbtime.getMs());
  s16 num_constraints = fmat.rows();
  s16 num_variables = fmat.cols();
  qpOASES::int_t nWSR = 100;
  qpOASES::QProblem problem_red (num_variables, num_constraints);
  qpOASES::Options op;
  op.setToMPC();
  op.printLevel = qpOASES::PL_NONE;
  problem_red.setOptions(op);
 
  //int rval = problem_red.init(&H_SDM, g_qpoases,&A_SDM, NULL, NULL, lb_qpoases, NULL, nWSR, 0);//q_soln_guess
  int rval = problem_red.init(H_qpoases, g_qpoases, A_qpoases, NULL, NULL, lb_qpoases, NULL, nWSR);
  (void)rval;
  int rval2 = problem_red.getPrimalSolution(q_soln);
  real_t_to_matrix(U_sol,q_soln,U_sol.rows());

  Xsolin=A_qp*x_0+B_qp*U_sol+C_qp;
  //printf("CompTimer2m: %.3f ms.\n", timer_rtm.getMs());
  for(s16 itra = 0; itra < (13*(setup->horizon-1)); itra++)
    update->traj[itra] = Xsolin(13+itra);
  for(s16 itra = 0; itra < (13); itra++)
    update->traj[13*(setup->horizon-1)+itra] = Xsolin(13*(setup->horizon-1)+itra);
}
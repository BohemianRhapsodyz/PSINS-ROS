#define g_0           9.78049           //�������ٶȳ�ֵ
#define R_e           6378393.0         //����뾶
#define Rad_Degree   0.01745329        /*����-�Ƕ�ת��*/
#define pi           3.14159265358979
#define cyzq         0.008  //sample period 
//�¼Ӷ���
#define Degree_Rad   1/Rad_Degree
#define e_            3.367e-3          /*�������*/
#define w_ie          7.27220417e-5      //������ת���ٶ�
/*#define phi0          34*Rad_Degree   //γ��
#define lon0          108*Rad_Degree  //����*/
#define phi_0         0.596441460000000   //γ��
#define lon_0         1.901509250000000  //����
#define high0         4.624540000000000e+02            //�߶�
#define dl            0.0001           //����ߴ�
#define biaodingtime  3   //�궨ʱ��
#define ln            90000 //�������ݳ���
#define lon_ref       1.919862177193763   //·��㾭��   
#define phi_ref       0.349292468420434   //·���γ��
#define high_ref      380              //·���߶�


#define windows_len 11     //���ڳ���
#define Gateb_min   0.06   //������ģֵ��ֵ
#define Gateb_max   0.20   //������ģֵ��ֵ
#define Gatev_min   0.06   //������ֵ
#define Gatev_max   0.20   //������ֵ
#define gravity      9.78049

#define FRQ				125
#define TS				0.008
#define PathIn			"/home/hx/od_sins/"
#define PathOut			"/home/hx/od_sins/"
//#define PathOut			"C:\\Users\\zhang\\Desktop\\chengxu\\od_sins_kalman\\kalmanv2\\kalmanv2\\kalmanv2\\"
#define FileIn			"imu.txt"
#define FileOut1		"align.bin"
#define FileOut2		"navigation.bin"
#define FileOut3		"kf.bin"
#define FileOut4		"kfpk.bin"
//#define SkipT			20
#define Aligni0T		60
#define AlignkfT        250

int jj;
double data1, data2, data3, data4, data5, data6, data7;
//�¼Ӷ���
double Ryp, Rxp, g, phi, hit_x, hit_y, hit_z;
double q[4], T[9], Tmb[9], attitude_old[3], attitude[3], wiep[3], wepp[3], wipp[3], wipb[3], wpbb1[3], wpbb2[3], wpbb3[3], hit_v[3], wib1[3], wib2[3], wib3[3], fb1[3], fb2[3], fb3[3], wibs[3], fbs[3], od1, od2, od3;
double pitcherr, rollerr, yawerr, kod;
double alignpitch, alignroll, alignyaw;
double S1[3], S2[3];
int inter = -1, t1 = 10;
double bias, var;
double dist = 0;
//���ٴ�����
double speedbump_x[] = { 110 };
double speedbump_y[] = { 20 };
int brinv(double *a, int n);
int MultMatr(double*A, double*B, double*C, int line, int k, int column);
void strapdown(double hit_shuju1, double hit_shuju2, double hit_shuju3, double hit_shuju4, double hit_shuju5, double hit_shuju6, double hit_shuju7, double hit_shuju8, double hit_shuju9, double hit_shuju10);
int SumMatr(double*A, double*B, double*C, int line, int column, int sign);
void SumVect(double*A, double*B, double*C, int n, int sign);
int Transpon(double*A, double*B, int line, int column);
//�¼Ӻ���
void TransMatrix(double *a, double *b, int m, int n);
void sum(double*a, double*b, int line, int column, int sign, double*c);
void mult(double *a, double *b, int m, int n, int k, double *c);
void Update_q();
void Rgktf_q(double *omega, double *q, double *fq);
void Attitude();
void Calculate_T();
void Calculate_Tmb();
void Normalize();
void Calculate_wpbb();
void Rgkt4_q(double *x);
void phiwest2east(double phi);


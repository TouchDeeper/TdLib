#include <iostream>
#include <stdio.h>
#include <omp.h>// OpenMP编程需要包含的头文件
#include "TdLibrary/tic_toc.h"
#include <Eigen/Core>
#include <sys/time.h>
#include <complex>
#include <algorithm>
#include <functional>
#include <vector>
typedef std::vector< std::complex<float> > TCmplxVec;

#pragma omp declare reduction( + : TCmplxVec : \
        std::transform(omp_in.begin( ),  omp_in.end( ), \
                       omp_out.begin( ), omp_out.begin( ), \
                       std::plus< std::complex<float> >( )) ) \
                       initializer (omp_priv(omp_orig))
int main()
{
//    std::cout<<"CPU number:"<<omp_get_num_procs()<<std::endl;
//    std::cout<<"Paraleele area 1"<<std::endl;
//#pragma omp parallel for
//    for(int i = 0; i < 6; ++i){
//        for (int j = 0; j < 6; ++j) {
//            std::cout<<i<<","<<j<<" "<<"This thread ID is "<<omp_get_thread_num()<<std::endl;
//        }
//    }
//    std::cout<<"Paraleele area 2"<<std::endl;
//    omp_set_num_threads(6);
//    int c = 3;
//#pragma omp parallel shared(c)
//    {
////        std::cout << "Num of threads is " << omp_get_num_threads();
////        std::cout << "the thread ID is " << omp_get_thread_num() << std::endl;
//        int A =1;
////        c=0;
////#pragma omp for firstprivate(c) lastprivate(c)
//#pragma omp for
//        for(int i = 0; i < 6; ++i){
//            for (int j = 0; j < 6; ++j) {
////                A = omp_get_thread_num();
////                std::cout<<i<<","<<j<<" "<<"This thread ID is "<<omp_get_thread_num()<<std::endl;
////                std::cout<<"A="<<A<<std::endl;
////                #pragma omp atomic
//                #pragma omp critical
//                c++;
//                std::cout<<c<<std::endl;
//            }
//
//        }
//    }
//    std::cout<<"finish c= "<<c<<std::endl;
//    std::cout<<"Parallel area 3"<<std::endl;
//    int shared_to_private = 1;
//
//
//#pragma omp parallel for firstprivate(shared_to_private) lastprivate(shared_to_private)
//    for (int k = 0; k < 10; ++k) {
////        shared_to_private = 2;
//        std::cout<<shared_to_private<<std::endl;
//    }
//    std::cout<<"shared_to_private="<<shared_to_private<<std::endl;
//    std::cout<<"Parallel area 4"<<std::endl;
//    int sum = 0;
//    int i = 0;
//#pragma omp parallel for shared(sum) shared(i)
//    for (i = 0; i < 10; ++i) {
//        #pragma omp atomic
//        sum = sum+1;
//    }
//    std::cout<<"sum="<<sum<<std::endl;
//    std::cout<<"Parallel area 5"<<std::endl;
//
//    int sum2 =10;
//    omp_set_num_threads(2);
//#pragma omp parallel for reduction(*:sum2)//注意这里是乘
//    for (int l = 0; l < 10; ++l) {
//        sum2 = sum2 + 1;
////        std::cout<<"sum2="<<sum2<<" l="<<l<<std::endl;
//
//    }
//    std::cout<<"sum2:"<<sum2<<std::endl;
//    std::cout<<"Parallel area 6"<<std::endl;
//
#pragma omp parallel num_threads(6)
    {
        #pragma omp critical
        {
            std::cout << omp_get_thread_num() << " ";
            std::cout << omp_get_thread_num() << " ";
            std::cout << omp_get_thread_num() << " ";
            std::cout << omp_get_thread_num() << " ";
        };


        #pragma omp critical
        {
            std::cout << omp_get_thread_num()+10 << " ";
            std::cout << omp_get_thread_num()+10 << " ";
            std::cout << omp_get_thread_num()+10 << " ";
            std::cout << omp_get_thread_num()+10 << " ";
        };


    }
    std::cout<<"\n Parallel area 7"<<std::endl;

//#pragma omp parallel num_threads(6)
//    {
//#pragma omp for nowait
////#pragma omp for
////#pragma omp critical
//        for (int i = 0;i < 10; ++i)
//        {
//            std::cout <<i << "+" << std::endl;
//        }
////#pragma omp barrier
//#pragma omp for
////#pragma omp critical
//        for (int j = 0; j < 10; ++j)
//        {
//            std::cout << j << "-" << std::endl;
//        }
//#pragma omp for
//        for (int k = 0;k < 10; ++k)
//        {
//            std::cout <<k << "+" << std::endl;
//        }
//    }
//    std::cout<<"\nParallel area 8"<<std::endl;

//    struct timeval start, end;
//    gettimeofday(&start, NULL);
//
//    int edge = 100;
//    int vertice = 100;
//    int vertice_size = 6;
//    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(vertice * vertice_size,vertice * vertice_size);
//    TicToc t_make_h;
//
//    for (int m = 0; m < edge; ++m) {
//        Eigen::MatrixXd H_temp = Eigen::MatrixXd::Zero(vertice * vertice_size,vertice * vertice_size);
//
//        for (int j = 0; j < vertice; ++j) {
//            Eigen::MatrixXd J1 = Eigen::MatrixXd::Random(vertice_size,1);
//            if(j < 0.7 * vertice)
//            {
//#pragma omp for
//                for (int k = j; k < vertice; ++k) {
//                    Eigen::MatrixXd J2 = Eigen::MatrixXd::Random(1,vertice_size);
//                    int row = vertice_size * j;
//                    int col = vertice_size * k;
//                    H_temp.block(row,col,vertice_size,vertice_size) = J1 * J2;
//                }
//            } else{
//                for (int k = j; k < vertice; ++k) {
//                    Eigen::MatrixXd J2 = Eigen::MatrixXd::Random(1,vertice_size);
//                    int row = vertice_size * j;
//                    int col = vertice_size * k;
//                    H_temp.block(row,col,vertice_size,vertice_size) = J1 * J2;
//                }
//            }
//
////            for (int k = j; k < vertice; ++k) {
////                Eigen::MatrixXd J2 = Eigen::MatrixXd::Random(1,vertice_size);
////                int row = vertice_size * j;
////                int col = vertice_size * k;
////                H_temp.block(row,col,vertice_size,vertice_size) = J1 * J2;
////            }
//
//        }
//        H += H_temp;
//
//    }
////    std::cout<<"****** H ******* \n"<<H<<std::endl;
//    gettimeofday(&end, NULL);
//    double delta;
//    delta = ((end.tv_sec  - start.tv_sec) * 1000000u +
//             end.tv_usec - start.tv_usec) / 1.e6;
////    std::cout<<"make hessian cost "<<t_make_h.toc()<<"ms"<<std::endl;
//    std::cout<<"make hessian cost "<<delta<<"s"<<std::endl;
//
////    for (int n = 0; n < vertice; ++n) {
////        for (int j = n; j < n; ++j) {
////            assert(H(n,j) == n*j);
////        }
////    }


    int size = 10;

    TCmplxVec result(size,0);
    omp_set_num_threads(6);
#pragma omp parallel reduction( + : result )
    {
        int tid=omp_get_thread_num();

        for (int i=0; i<std::min(tid+1,size); i++)
            result[i] += tid;
        std::cout<<"thread "<<tid<<" result:"<<std::endl;
        for (int i=0; i<size; i++)
            std::cout << i << "\t" << result[i] << std::endl;
    }
    std::cout<<std::endl;
    for (int i=0; i<size; i++)
        std::cout << i << "\t" << result[i] << std::endl;
    std::cout<<"parallel area 9 \n"<<std::endl;

    Eigen::Matrix4d a = Eigen::Matrix4d::Zero();


    std::cout<<"thread "<<omp_get_thread_num()<<std::endl;
    omp_set_num_threads(6);
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> a_vec(6);

#pragma omp parallel for
    for (int k = 0; k < 100; ++k) {
        for (int i = 0; i < a.rows(); ++i) {
            for (int j = 0; j < a.cols(); ++j) {
                a_vec[omp_get_thread_num()](i,j)++;
            }
        }
    }

    for (int l = 0; l < a_vec.size(); ++l) {
        a += a_vec[l];
    }


    std::cout<<a<<std::endl;
    return 0;

};